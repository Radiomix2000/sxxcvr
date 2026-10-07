// SPDX-License-Identifier: MIT

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Registry.hpp>
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Time.hpp>

#include <string.h>
#include <algorithm>
#include <cassert>
#include <climits>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <thread>
#include <mutex>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <linux/types.h>
#include <linux/gpio.h>
#include <linux/spi/spidev.h>

#include <alsa/asoundlib.h>
#include <alsa/control.h>

extern const char *SoapySX_tag;
extern const char *SoapySX_commit;

// Streaming mode, affecting how starting, stopping, overruns and underruns
// are handled.
enum stream_mode {
    // Behave like most SDRs: RX overrun or TX underrun
    // may cause samples to be dropped, but streams will keep running.
    // Application can use timestamps to maintain correct timing.
    STREAM_MODE_NORMAL,
    // Behave like linked ALSA PCMs with default software parameters.
    // Overrun or underrun causes both RX and TX streams to stop.
    // TX buffer must be always kept filled to avoid underrun.
    // First write to TX stream starts both streams.
    // This was a stop-gap solution before implementation of timestamps
    // and is primarily kept here for compatibility with applications
    // that have not been modified to use timestamps yet.
    STREAM_MODE_LINK,
};

// Clamp, offset, scale and quantize a value based on a SoapySDR::Range
// and convert it to an integer.
// The value is offset so that the minimum value becomes 0
// and scaled so that step becomes 1.
static int32_t scale_from_range(SoapySDR::Range range, double value)
{
    return (int)std::round(
        (std::min(std::max(value, range.minimum()), range.maximum())
         - range.minimum()) / range.step());
}

// Inverse of scale_from_range.
static double scale_to_range(SoapySDR::Range range, int32_t value)
{
    return std::min(std::max(
        range.minimum() + range.step() * (double)value,
        range.minimum()), range.maximum());
}

// Read a number from a file in format 0x1234.
static uint16_t read_hex_file(const char *filename)
{
    std::ifstream file(filename);
    std::string str;
    std::getline(file, str);
    return std::stoul(str, NULL, 16);
}

struct hat_info {
    uint16_t product_id;
    uint16_t product_ver;
    bool read_success;
};

static struct hat_info read_hat_info(void)
{
    const uint16_t product_id_expected = 0x1255;
    // Default assumption if ID cannot be read
    struct hat_info info = { product_id_expected, 0x0101, false };
    try {
        info.product_id  = read_hex_file("/proc/device-tree/hat/product_id");
        info.product_ver = read_hex_file("/proc/device-tree/hat/product_ver");
        info.read_success = true;
        SoapySDR_logf(SOAPY_SDR_INFO, "Hardware version %d.%d",
            info.product_ver >> 8, info.product_ver & 0xFF);
    } catch(std::exception &e) {
        SoapySDR_logf(SOAPY_SDR_WARNING, "Could not read HAT ID. Assuming hardware version %d.%d",
            info.product_ver >> 8, info.product_ver & 0xFF);
    }
    if (info.product_id != product_id_expected) {
        SoapySDR_logf(SOAPY_SDR_WARNING, "Unexpected product ID 0x%04x. Are you sure the correct HAT is connected?", info.product_id);
    }
    return info;
}

// Corrections applied to received samples.
struct rx_corrections {
    // Remove DC offset using a high-pass filter
    bool dc_removal;
    // Coefficient of the DC removal filter
    double dc_alpha;
    // IQ balance correction: y = x + iq * conj(x)
    std::complex<float> iq;
};

// Corrections applied to transmitted samples.
struct tx_corrections {
    // Added to each sample to cancel LO leakage
    std::complex<float> dc;
    // IQ balance predistortion: y = x + iq * conj(x)
    std::complex<float> iq;
};

// Convert raw received samples to CF32.
// TODO: Support other formats and add format as a parameter.
static inline void convert_rx_buffer(const void *src, size_t src_offset, void *dest, size_t dest_offset, size_t length, const struct rx_corrections &corr, std::complex<double> &dc_state)
{
    const int32_t *src_ = (const int32_t*)src + src_offset*2;
    float *dest_ = (float*)dest + dest_offset*2;
    const float scaling = 1.0f / 0x80000000L;
    const float br = corr.iq.real(), bi = corr.iq.imag();
    double dc_i = dc_state.real(), dc_q = dc_state.imag();
    for (size_t i = 0; i < length*2; i+=2)
    {
        float fi = scaling * (float)src_[i], fq = scaling * (float)src_[i+1];
        if (corr.dc_removal) {
            // First order DC blocker. State is kept in double precision
            // since the filter coefficient is very small.
            double di = (double)fi - dc_i, dq = (double)fq - dc_q;
            dc_i += corr.dc_alpha * di;
            dc_q += corr.dc_alpha * dq;
            fi = (float)di;
            fq = (float)dq;
        }
        // y = x + iq * conj(x)
        dest_[i  ] = fi + br * fi + bi * fq;
        dest_[i+1] = fq + bi * fi - br * fq;
    }
    dc_state = std::complex<double>(dc_i, dc_q);
}

// Convert CF32 to raw transmit samples.
// TODO: Support other formats and add format as a parameter.
static inline void convert_tx_buffer(const void *src, size_t src_offset, void *dest, size_t dest_offset, size_t length, float tx_threshold2, const struct tx_corrections &corr)
{
    const float *src_ = (const float*)src + src_offset*2;
    int32_t *dest_ = (int32_t*)dest + dest_offset*2;
    const float scaling = (float)0x7FFFFFFFL;
    const float br = corr.iq.real(), bi = corr.iq.imag();
    const float dr = corr.dc.real(), di = corr.dc.imag();
    for (size_t i = 0; i < length*2; i+=2)
    {
        float fi = src_[i], fq = src_[i+1];
        // y = x + iq * conj(x) + dc
        float ci = fi + br * fi + bi * fq + dr;
        float cq = fq + bi * fi - br * fq + di;
        int32_t vi = scaling * std::max(std::min(ci, 1.0f), -1.0f);
        int32_t vq = scaling * std::max(std::min(cq, 1.0f), -1.0f);
        // Second lowest bit of each "I" sample controls RX/TX switching.
        // Set the lowest bit to the same value just in case.
        // Let's also reserve the 2 lowest bits of "Q" samples
        // for future extensions and keep them as 0.
        vi &= 0xFFFFFFFCL;
        vq &= 0xFFFFFFFCL;
        // Threshold is compared to the signal before corrections,
        // so that DC offset correction alone does not turn transmitter on.
        if (fi*fi + fq*fq >= tx_threshold2)
            vi |= 0b11L;
        dest_[i  ] = vi;
        dest_[i+1] = vq;
    }
}

// One point of a calibration table.
struct calibration_point {
    double frequency;
    std::complex<double> tx_dc;
    std::complex<double> tx_iq;
    std::complex<double> rx_iq;
    // TX gains the point was calibrated with.
    // NAN if unknown (older tables), in which case the point
    // is used with any TX gains.
    double tx_dac, tx_mixer;
    // Only RX IQ balance is valid
    bool rx_only;
};

// Directory of SoapySX configuration files:
// $XDG_CONFIG_HOME/SoapySX or ~/.config/SoapySX
static std::string config_directory(void)
{
    const char *xdg = getenv("XDG_CONFIG_HOME");
    if (xdg != NULL && xdg[0] != '\0')
        return std::string(xdg) + "/SoapySX";
    const char *home = getenv("HOME");
    if (home != NULL && home[0] != '\0')
        return std::string(home) + "/.config/SoapySX";
    return "";
}

// Default calibration file path
static std::string default_calibration_path(void)
{
    const std::string dir = config_directory();
    return dir.empty() ? "" : dir + "/calibration.txt";
}

// Read configuration file soapysx.conf containing key=value lines.
// Settings there are used as defaults for device arguments,
// for applications which do not allow giving device arguments.
static SoapySDR::Kwargs read_config_file(void)
{
    SoapySDR::Kwargs config;
    const std::string dir = config_directory();
    if (dir.empty())
        return config;
    std::ifstream file(dir + "/soapysx.conf");
    std::string line;
    while (std::getline(file, line)) {
        size_t comment = line.find('#');
        if (comment != std::string::npos)
            line.erase(comment);
        size_t eq = line.find('=');
        if (eq == std::string::npos)
            continue;
        auto trim = [](std::string v) {
            const char *ws = " \t\r\n";
            v.erase(0, v.find_first_not_of(ws));
            v.erase(v.find_last_not_of(ws) + 1);
            return v;
        };
        config[trim(line.substr(0, eq))] = trim(line.substr(eq + 1));
    }
    return config;
}

// Create directory of a file and its parents if they do not exist.
static void create_parent_directories(const std::string &path)
{
    for (size_t i = 1; i < path.size(); i++) {
        if (path[i] == '/')
            mkdir(path.substr(0, i).c_str(), 0755);
    }
}

static void write_calibration_point(std::ostream &out, const struct calibration_point &p)
{
    char line[256];
    snprintf(line, sizeof(line), "%.0f %.7f %.7f %.7f %.7f %.7f %.7f",
        p.frequency,
        p.tx_dc.real(), p.tx_dc.imag(),
        p.tx_iq.real(), p.tx_iq.imag(),
        p.rx_iq.real(), p.rx_iq.imag());
    out << line;
    if (p.rx_only) {
        out << " -1 -1";
    } else if (!std::isnan(p.tx_dac)) {
        snprintf(line, sizeof(line), " %.1f %.1f", p.tx_dac, p.tx_mixer);
        out << line;
    }
    out << "\n";
}

// Read a calibration table written by tools/calibrate.py or SoapySX.
// Each non-comment line contains:
// frequency_hz tx_dc_re tx_dc_im tx_iq_re tx_iq_im rx_iq_re rx_iq_im [tx_dac tx_mixer]
// TX gains -1 mark a point where only RX IQ balance is valid.
// Points without TX gains are used with any TX gains.
// Returns points sorted by frequency. Throws if the file is malformed.
static std::vector<struct calibration_point> read_calibration_file(const std::string &path)
{
    std::vector<struct calibration_point> table;
    std::ifstream file(path);
    if (!file.is_open())
        return table;
    std::string line;
    size_t line_number = 0;
    while (std::getline(file, line)) {
        line_number++;
        size_t comment = line.find('#');
        if (comment != std::string::npos)
            line.erase(comment);
        std::istringstream ss(line);
        double f, v[6];
        if (!(ss >> f))
            continue; // Empty line
        for (size_t i = 0; i < 6; i++) {
            if (!(ss >> v[i]))
                throw std::runtime_error("Malformed calibration file " + path + " at line " + std::to_string(line_number));
        }
        double dac = NAN, mixer = NAN, value;
        if (ss >> value) {
            dac = value;
            if (!(ss >> value))
                throw std::runtime_error("Malformed calibration file " + path + " at line " + std::to_string(line_number));
            mixer = value;
        }
        table.push_back({
            f,
            std::complex<double>(v[0], v[1]),
            std::complex<double>(v[2], v[3]),
            std::complex<double>(v[4], v[5]),
            dac, mixer,
            dac < 0.0,
        });
    }
    std::sort(table.begin(), table.end(),
        [](const struct calibration_point &a, const struct calibration_point &b) {
            return a.frequency < b.frequency;
        });
    return table;
}

// Interpolate calibration table linearly at a given frequency.
// Values outside the table are clamped to the nearest point.
// Table must not be empty.
static struct calibration_point interpolate_calibration(const std::vector<struct calibration_point> &table, double frequency)
{
    assert(!table.empty());
    if (frequency <= table.front().frequency)
        return table.front();
    if (frequency >= table.back().frequency)
        return table.back();
    size_t i = 1;
    while (table[i].frequency < frequency)
        i++;
    const struct calibration_point &a = table[i-1], &b = table[i];
    const double t = (frequency - a.frequency) / (b.frequency - a.frequency);
    return {
        frequency,
        a.tx_dc + t * (b.tx_dc - a.tx_dc),
        a.tx_iq + t * (b.tx_iq - a.tx_iq),
        a.rx_iq + t * (b.rx_iq - a.rx_iq),
        a.tx_dac, a.tx_mixer, false,
    };
}

// Select points of calibration table valid for TX with given gains.
// If any_gains is set, points with any TX gains are selected.
static std::vector<struct calibration_point> select_tx_points(
    const std::vector<struct calibration_point> &table, double dac, double mixer, bool any_gains)
{
    std::vector<struct calibration_point> selected;
    for (const auto &p : table) {
        if (p.rx_only)
            continue;
        if (any_gains || std::isnan(p.tx_dac)
            || (std::abs(p.tx_dac - dac) < 0.5 && std::abs(p.tx_mixer - mixer) < 0.5))
            selected.push_back(p);
    }
    return selected;
}

// Distance from frequency to the nearest point of a table
static double nearest_point_distance(const std::vector<struct calibration_point> &table, double frequency)
{
    double d = INFINITY;
    for (const auto &p : table)
        d = std::min(d, std::abs(p.frequency - frequency));
    return d;
}

#define MAX_REGS 0x80

// Number of initial register values for SX1255.
// Only write the documented registers from 0x00 to 0x13.
#define N_INIT_REGISTERS 0x14
// Initial register values for SX1255.
static const uint8_t init_registers[N_INIT_REGISTERS] = {
    // 0x00: default value from datasheet (enable oscillator)
    0b00000001,
    // 0x01-0x06: RX and TX frequencies: 433.92 MHz
    0xD8, 0xF5, 0xC3, 0xD8, 0xF5, 0xC3,
    // 0x07: read-only register, written value does not matter
    0x11,
    // 0x08: TX gains, default value from datasheet
    0b00101110,
    // 0x09: default value from datasheet
    0b00100100,
    // 0x0A: default value from datasheet
    0b00110000,
    // 0x0B: default value from datasheet
    0b00000010,
    // 0x0C: RX gains, default value from datasheet
    0b00111111,
    // 0x0D: RX filters, make them narrow
    // ADCTRIM value of 7 minimized ADC spurs
    // but maybe 6 worked better with 38.4 MHz clock in some cases.
    0b00111011,
    // 0x0E: default value from datasheet
    0b00000110,
    // 0x0F: IO_MAP, default value from datasheet
    0b00000000,
    // 0x10, CK_SEL, default value from datasheet
    0b00000010,
    // 0x11: read-only register(?), written value does not matter
    0b00000000,
    // 0x12-0x13: I2S at 125 kHz: CLKOUT divider 4, decimate by 256
    0b00100010, 0b00101100,
};


// Register values for a given sample rate
struct sampleRateRegs {
    // Ratio of reference clock to sample rate
    uint16_t div;
    // iism_clk_div value (register 0x12 bits 3-0)
    uint8_t clkout : 4;
    // int_dec_mantisse value (register 0x13 bit 7)
    uint8_t mant : 1;
    // int_dec_m_parameter value (register 0x13 bit 6)
    uint8_t m : 1;
    // int_dec_n_parameter value (register 0x13 bits 5-3)
    uint8_t n : 3;
};

#define N_SAMPLE_RATES 6

// Register values for different sample rates
static const struct sampleRateRegs sample_rates[N_SAMPLE_RATES] = {
    {1536, 0b0110, 0, 1, 6 },
    { 768, 0b0100, 0, 1, 5 },
    { 512, 0b0011, 0, 0, 6 },
    //{ 384, 0b0011, 0, 1, 4 }, // 24 bit samples (did not work correctly)
    { 256, 0b0010, 0, 0, 5 },
    //{ 192, 0b0010, 0, 1, 3 }, // 24 bit samples (did not work correctly)
    { 128, 0b0001, 0, 0, 4 },
    //{  96, 0b0001, 0, 1, 2 }, // 24 bit samples (did not work correctly)
    {  64, 0b0000, 0, 0, 3 },
    //{  48, 0b0000, 0, 1, 1 }, // 24 bit samples (did not work correctly)
    //{  32, 0b0000, 0, 0, 2 }, // 16 bit samples (did not work)
};


// Class to use SPI through the SPI userspace API (SPIDEV) in Linux.
// Putting it in a separate class helps use RAII to ensure
// the file descriptor gets closed in all situations.
class Spi {
private:
    // SPIDEV file descriptor
    int fd;

public:
    Spi(const char *spidev_path)
    {
        assert(spidev_path != NULL);
        fd = open(spidev_path, O_RDWR);
        SoapySDR_logf(SOAPY_SDR_DEBUG, "SPIDEV opened: %d", fd);
        if (fd < 0) {
            // TODO: add more detailed error messages from strerror or something
            throw std::runtime_error("Failed to open SPI");
        }
    }

    ~Spi()
    {
        SoapySDR_logf(SOAPY_SDR_DEBUG, "Closing SPIDEV fd %d", fd);
        close(fd);
    }

    int transfer(const uint8_t *tx_data, uint8_t *rx_data, const size_t len) const
    {
        struct spi_ioc_transfer transfer;
        memset(&transfer, 0, sizeof(transfer));

        transfer.tx_buf = (__u64)tx_data;
        transfer.rx_buf = (__u64)rx_data;
        transfer.len = (__u32)len;
        transfer.speed_hz = 10000000;

        int ret = ioctl(fd, SPI_IOC_MESSAGE(1), &transfer);
        SoapySDR_logf(SOAPY_SDR_DEBUG, "SPIDEV ioctl: %d", ret);

        if ((size_t)ret != len)
            throw std::runtime_error("SPI transfer failed");

        return ret;
    }

    int transfer(const std::vector<uint8_t> &tx_data, std::vector<uint8_t> &rx_data) const
    {
        // Make sure data fits in both buffers
        const size_t transfer_len = std::min(tx_data.size(), rx_data.size());
        return transfer(tx_data.data(), rx_data.data(), transfer_len);
    }
};



// Similar to class Spi, but for GPIO pins.
class GpioChip {
public:
    int chip_fd;

    GpioChip(const char *gpiochip_path)
    {
        assert(gpiochip_path != NULL);
        chip_fd = open(gpiochip_path, O_RDWR);
        if (chip_fd < 0) {
            throw std::runtime_error("Failed to open GPIO");
        }
    }

    ~GpioChip()
    {
        close(chip_fd);
    }
};

class GpioLine {
public:
    int line_fd;

    GpioLine(class GpioChip &chip, __u32 pin_number, const char *name, __u64 flags, bool initial_value)
    {
        struct gpio_v2_line_request req = {
            .offsets = { pin_number },
            .consumer = "",
            .config = (struct gpio_v2_line_config) {
                .flags = flags,
                .num_attrs = 0,
                .padding = { },
                .attrs = { },
            },
            .num_lines = 1,
            .event_buffer_size = 0,
            .padding = { },
            .fd = 0,
        };
        strncpy(req.consumer, name, GPIO_MAX_NAME_SIZE-1);

        int ret = ioctl(chip.chip_fd, GPIO_V2_GET_LINE_IOCTL, &req);
        if (ret < 0) {
            throw std::runtime_error("Failed to request GPIO line");
        }

        line_fd = req.fd;

        set_value(initial_value);
    }

    ~GpioLine()
    {
        close(line_fd);
    }

    void set_value(bool value)
    {
        struct gpio_v2_line_values val = {
            .bits = value,
            .mask = 1,
        };
        int ret = ioctl(line_fd, GPIO_V2_LINE_SET_VALUES_IOCTL, &val);
        if (ret < 0) {
            throw std::runtime_error("Failed to write GPIO line");
        }
    }
};



// Convert an ALSA recording error to a corresponding SoapySDR readStream error.
static int alsa_error_to_soapy_rx(int alsa_error_code)
{
    if (alsa_error_code == -EPIPE) {
        // Overrun
        return SOAPY_SDR_OVERFLOW;
    } else {
        // Some other error
        return SOAPY_SDR_STREAM_ERROR;
    }
}

// Convert an ALSA playback error to a corresponding SoapySDR writeStream error.
static int alsa_error_to_soapy_tx(int alsa_error_code)
{
    if (alsa_error_code == -EPIPE) {
        // Underrun
        return SOAPY_SDR_UNDERFLOW;
    } else {
        // Some other error
        return SOAPY_SDR_STREAM_ERROR;
    }
}


#define ALSACHECK(a) do {\
int retcheck = (a); \
if (retcheck < 0) { \
SoapySDR_logf(SOAPY_SDR_ERROR, "\nALSA error in %s: %s\n", #a, snd_strerror(retcheck)); \
goto alsa_error; } } while(0)

class AlsaPcm {
public:
    const char *name;
    snd_pcm_t *pcm;
    mutable std::mutex mutex;
    snd_pcm_stream_t dir;
    enum stream_mode stream_mode;
    bool setup_done;
    bool activated;
    int64_t position;
    snd_pcm_uframes_t hwp_period_size;
    snd_pcm_uframes_t hwp_buffer_size;

    AlsaPcm(const char *name, snd_pcm_stream_t dir):
        name(name),
        pcm(NULL),
        dir(dir),
        stream_mode(STREAM_MODE_NORMAL),
        setup_done(0),
        activated(0),
        position(0),
        hwp_period_size(0),
        hwp_buffer_size(0)
    {
    }

    void open(void)
    {
        ALSACHECK(snd_pcm_open(&pcm, name, dir, 0));
        return;

        alsa_error:
        if (pcm != NULL)
            snd_pcm_close(pcm);

        // TODO more detailed error messages?
        throw std::runtime_error("Error opening ALSA device");
    }

    ~AlsaPcm()
    {
        if (pcm != NULL)
            snd_pcm_close(pcm);
    }

    bool is_tx(void)
    {
        return dir == SND_PCM_STREAM_PLAYBACK;
    }

    int reset()
    {
        int ret = 0;
        // Make sure stream is stopped. If it was stopped already,
        // drop will fail but that is fine, so do not check the return value.
        snd_pcm_drop(pcm);

        ret = snd_pcm_prepare(pcm);
        if (ret < 0)
            return ret;
        position = 0;
        ret = snd_pcm_reset(pcm);
        return ret;
    }

    void configure(snd_pcm_uframes_t period)
    {
        if (pcm == NULL)
            return;
        snd_pcm_hw_params_t *hwp = NULL;
        snd_pcm_sw_params_t *swp = NULL;

        unsigned int hwp_periods = 0;

        // Period size basically determines the I2S DMA interrupt rate.
        // A higher period size somewhat reduces CPU use
        // but increases the minimum latency.
        // If an application reads samples in fixed size blocks,
        // it is usually best to use a period a size equal to the block size
        // to minimize latency while avoiding unnecessary CPU use.
        // Period size can configured using the "period" stream argument.
        // Use default value if 0.
        hwp_period_size = period > 0 ? period : 256;

        // Based on some tests requesting various period and buffer sizes,
        // it seems like, on a Raspberry Pi:
        // * Maximum buffer size is 65536.
        // * Buffer size should be a multiple of period size.
        //   Otherwise ALSA forces period size to be a submultiple of buffer size.
        // * There should be at least 2 periods per buffer, but we do not really need
        //   to check for this. ALSA handles it nicely.
        // * Minimum period size is 2 but this does not really need
        //   a separate check either.
        // Given these constraints, find the maximum possible buffer size
        // to achieve the period requested.
        const snd_pcm_uframes_t max_buffer_size = 65536;
        hwp_period_size = std::min(hwp_period_size, max_buffer_size);
        hwp_buffer_size = max_buffer_size / hwp_period_size * hwp_period_size;


        snd_pcm_uframes_t swp_boundary = 0;

        ALSACHECK(snd_pcm_hw_params_malloc(&hwp));
        ALSACHECK(snd_pcm_hw_params_any(pcm, hwp));
        ALSACHECK(snd_pcm_hw_params_set_access(pcm, hwp, SND_PCM_ACCESS_RW_INTERLEAVED));
        ALSACHECK(snd_pcm_hw_params_set_format(pcm, hwp, SND_PCM_FORMAT_S32_LE));
        // Sample rate given to ALSA does not affect the actual sample rate,
        // so just use a fixed "dummy" value.
        ALSACHECK(snd_pcm_hw_params_set_rate(pcm, hwp, 192000, 0));
        ALSACHECK(snd_pcm_hw_params_set_channels(pcm, hwp, 2));
        ALSACHECK(snd_pcm_hw_params_set_buffer_size_near(pcm, hwp, &hwp_buffer_size));
        ALSACHECK(snd_pcm_hw_params_set_period_size_near(pcm, hwp, &hwp_period_size, 0));
        ALSACHECK(snd_pcm_hw_params_get_periods(hwp, &hwp_periods, 0));

        ALSACHECK(snd_pcm_hw_params(pcm, hwp));
        snd_pcm_hw_params_free(hwp);

        SoapySDR_logf(SOAPY_SDR_DEBUG, "ALSA parameters: buffer_size=%d, period_size=%d, periods=%d", hwp_buffer_size, hwp_period_size, hwp_periods);

        ALSACHECK(snd_pcm_sw_params_malloc(&swp));
        ALSACHECK(snd_pcm_sw_params_current(pcm, swp));
        ALSACHECK(snd_pcm_sw_params_get_boundary(swp, &swp_boundary));
        SoapySDR_logf(SOAPY_SDR_DEBUG, "ALSA SW parameters: boundary=%lu", swp_boundary);
        if (stream_mode == STREAM_MODE_NORMAL) {
            ALSACHECK(snd_pcm_sw_params_set_stop_threshold(pcm, swp, swp_boundary));
            // https://stackoverflow.com/a/20515251
            ALSACHECK(snd_pcm_sw_params_set_silence_threshold(pcm, swp, 0));
            ALSACHECK(snd_pcm_sw_params_set_silence_size(pcm, swp, swp_boundary));
        } else {
            ALSACHECK(snd_pcm_sw_params_set_stop_threshold(pcm, swp, hwp_buffer_size));
            ALSACHECK(snd_pcm_sw_params_set_silence_threshold(pcm, swp, 0));
            ALSACHECK(snd_pcm_sw_params_set_silence_size(pcm, swp, 0));
        }

        ALSACHECK(snd_pcm_sw_params(pcm, swp));
        snd_pcm_sw_params_free(swp);

        ALSACHECK(reset());

        return;

    alsa_error:
        if (hwp != NULL)
            snd_pcm_hw_params_free(hwp);
        if (swp != NULL)
            snd_pcm_sw_params_free(swp);
        // TODO more detailed error messages?
        throw std::runtime_error("Error configuring ALSA device");
    }
};


/***********************************************************************
 * Built-in calibration helpers
 **********************************************************************/

// Frequencies used by calibration, in units of sample rate.
// TX LO is at the calibrated frequency and TX sends a tone at CAL_TONE.
// RX LO is tuned CAL_LO_OFFSET below TX LO, so in received signal:
//   CAL_LO_OFFSET + CAL_TONE   wanted tone
//   CAL_LO_OFFSET              TX LO leakage
//   CAL_LO_OFFSET - CAL_TONE   TX image
//   -(CAL_LO_OFFSET+CAL_TONE)  RX image
// The same plan is used by tools/calibrate.py.
static const double CAL_LO_OFFSET = 1.0 / 32.0;
static const double CAL_TONE = 3.0 / 32.0;
// Frequencies where no signal is expected, for estimating noise level
static const double CAL_NOISE_FREQS[] = { 5.5/32, -5.5/32, 7.5/32, -7.5/32, 9.5/32, -9.5/32 };

struct cal_measurement {
    std::complex<double> tone, tx_lo, tx_image, rx_image;
    double noise, peak;
};

// Complex amplitude of a tone at normalized frequency freq in signal x,
// whose first sample has index n0. Uses a Hann window.
static std::complex<double> correlate_tone(const std::vector<std::complex<float>> &x, int64_t n0, double freq)
{
    const size_t n = x.size();
    std::complex<double> acc = 0.0;
    double wsum = 0.0;
    // Rotate a phasor instead of computing it for every sample
    std::complex<double> phasor = std::polar(1.0, -2.0 * M_PI * std::fmod(freq * (double)n0, 1.0));
    const std::complex<double> rotation = std::polar(1.0, -2.0 * M_PI * freq);
    const std::complex<double> w_rotation = std::polar(1.0, 2.0 * M_PI / (double)(n - 1));
    std::complex<double> w_phasor = 1.0;
    for (size_t i = 0; i < n; i++) {
        // Hann window
        double w = 0.5 - 0.5 * w_phasor.real();
        acc += std::complex<double>(x[i]) * w * phasor;
        wsum += w;
        phasor *= rotation;
        w_phasor *= w_rotation;
    }
    return acc / wsum;
}

static struct cal_measurement cal_analyze(const std::vector<std::complex<float>> &x, int64_t n0, double lo_offset)
{
    struct cal_measurement m;
    m.tone     = correlate_tone(x, n0, lo_offset + CAL_TONE);
    m.tx_lo    = correlate_tone(x, n0, lo_offset);
    m.tx_image = correlate_tone(x, n0, lo_offset - CAL_TONE);
    m.rx_image = correlate_tone(x, n0, -(lo_offset + CAL_TONE));
    double p = 0.0;
    for (double f : CAL_NOISE_FREQS)
        p += std::norm(correlate_tone(x, n0, f));
    m.noise = std::sqrt(p / (sizeof(CAL_NOISE_FREQS) / sizeof(CAL_NOISE_FREQS[0])));
    m.peak = 0.0;
    for (auto v : x)
        m.peak = std::max(m.peak, (double)std::abs(v));
    return m;
}

static double to_db(double v)
{
    return 20.0 * std::log10(std::max(v, 1e-15));
}

/***********************************************************************
 * Device interface
 **********************************************************************/
class SoapySX : public SoapySDR::Device
{
private:
    double masterClock;
    double sampleRate;

    // Mutex for anything involving SX1255 registers
    // to avoid problems if an application calls methods from multiple threads.
    mutable std::recursive_mutex reg_mutex;

    Spi spi;
    GpioChip gpio;
    GpioLine gpio_reset, gpio_rx, gpio_tx;
    AlsaPcm alsa_rx;
    AlsaPcm alsa_tx;

    // Transmitter is turned on when squared magnitude of a TX sample
    // exceeds this threshold.
    float tx_threshold2;
    // If true, RX and TX streams have been linked using snd_pcm_link
    bool linked;

    // Values of registers (to be) written to the chip.
    // Storing them here makes it easier and faster to change
    // specific bits since they do not need to be read
    // from the chip every time.
    uint8_t regs[MAX_REGS];

    // Buffer for RX samples before type conversion.
    // Data is not actually uint64_t but it simplifies code a bit
    // by making one vector element correspond to one 32+32 bit complex sample.
    std::vector<uint64_t> buffer_rx;
    // Buffer for TX samples after type conversion.
    std::vector<uint64_t> buffer_tx;

    struct hat_info hat_info;

    // Mutex for DC offset and IQ balance corrections,
    // so they can be changed while streams are running.
    mutable std::mutex corr_mutex;
    // Corrections actually applied to samples.
    // IQ balance and TX DC offset are the sum of values interpolated
    // from the calibration table and values set by the application,
    // so that applications setting them to zero (as many do by default)
    // do not override the calibration.
    struct rx_corrections rx_corr;
    struct tx_corrections tx_corr;
    // Values from calibration table at current frequencies
    std::complex<double> table_rx_iq, table_tx_iq, table_tx_dc;
    // Values set by setIQBalance and setDCOffset
    std::complex<double> user_rx_iq, user_tx_iq, user_tx_dc;
    // Cutoff frequency of RX DC removal filter in Hz
    double rx_dc_cutoff;
    // State of RX DC removal filter. Only accessed from readStream.
    std::complex<double> rx_dc_state;

    // Calibration table and the file it was read from.
    // When the table is not empty, corrections are updated from it
    // every time frequency is changed.
    std::vector<struct calibration_point> calibration;
    // Calibration file. Results of built-in calibration are appended to it.
    // Empty if calibration file is disabled.
    std::string calibration_path;

    // Calibrate automatically when streams are activated
    // if calibration table has no point for current frequencies
    // and TX gains.
    bool auto_calibrate;
    // A calibration point is considered to be for the current frequency
    // if it is within this distance in Hz.
    double calibration_tolerance;
    // Current value of PA setting
    std::string pa_mode;

    // Convert a SoapySDR nanosecond timestamp to a sample counter.
    int64_t timestamp_to_samples(long long timestamp) const
    {
        return SoapySDR::timeNsToTicks(timestamp, sampleRate);
    }

    // Convert a sample counter to a SoapySDR nanosecond timestamp.
    long long samples_to_timestamp(int64_t samples) const
    {
        return SoapySDR::ticksToTimeNs(samples, sampleRate);
    }

    // Set given bits of a register.
    // The registers are not actually written to the chip.
    void set_register_bits(size_t address, unsigned lowestbit, unsigned nbits, unsigned value)
    {
        if (address >= MAX_REGS)
            throw std::runtime_error("Invalid register address");
        unsigned mask = ((1 << nbits) - 1) << lowestbit;
        regs[address] = (regs[address] & (~mask)) | ((value << lowestbit) & mask);
    }

    // Get given bits of a cached register.
    // The registers are not actually read from the chip.
    unsigned get_cached_register_bits(size_t address, unsigned lowestbit, unsigned nbits) const
    {
        if (address >= MAX_REGS)
            throw std::runtime_error("Invalid register address");
        unsigned mask = ((1 << nbits) - 1) << lowestbit;
        return (regs[address] & mask) >> lowestbit;
    }

    // Write a range of registers to the chip.
    void write_registers_to_chip(size_t firstreg, size_t nregs)
    {
        if ((firstreg >= MAX_REGS) || (nregs > MAX_REGS) || (firstreg > MAX_REGS - nregs))
            throw std::runtime_error("Invalid register address");

        size_t transfer_len = nregs + 1;
        // Buffer for SPI transfer
        std::vector<uint8_t> buf(transfer_len, 0);

        buf[0] = firstreg | 0x80;
        for (size_t i = 1; i < transfer_len; i++)
            buf[i] = regs[firstreg + i - 1];

        spi.transfer(buf, buf);
    }

    void reset_chip(void)
    {
        SoapySDR_logf(SOAPY_SDR_DEBUG, "Resetting chip");
        // Timing from datasheet Figure 6-2: Manual Reset Timing Diagram
        gpio_reset.set_value(1);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        gpio_reset.set_value(0);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    void init_chip(void)
    {
        for (size_t i = 0; i < N_INIT_REGISTERS; i++)
            set_register_bits(i, 0, 8, init_registers[i]);
        // Enable RX and TX, just for initial testing. This should be done somewhere else.
        set_register_bits(0, 1, 3, 0b111);
        write_registers_to_chip(0, N_INIT_REGISTERS);
    }

    // Update RX DC removal filter coefficient
    // after a change of sample rate or cutoff frequency.
    void update_rx_dc_alpha(void)
    {
        std::scoped_lock lock(corr_mutex);
        rx_corr.dc_alpha = 1.0 - std::exp(-2.0 * M_PI * rx_dc_cutoff / sampleRate);
    }

    // Update corrections applied to samples.
    // Must be called with corr_mutex locked.
    void update_corrections(void)
    {
        rx_corr.iq = std::complex<float>(table_rx_iq + user_rx_iq);
        tx_corr.iq = std::complex<float>(table_tx_iq + user_tx_iq);
        tx_corr.dc = std::complex<float>(table_tx_dc + user_tx_dc);
    }

    // Load calibration table from a file.
    // Empty path or "none" disables use of a calibration table.
    void load_calibration(const std::string &path)
    {
        std::scoped_lock lock(reg_mutex);
        calibration.clear();
        calibration_path = "";
        {
            std::scoped_lock corr_lock(corr_mutex);
            table_rx_iq = table_tx_iq = table_tx_dc = 0.0;
            update_corrections();
        }
        if (path == "" || path == "none")
            return;
        calibration_path = path;
        calibration = read_calibration_file(path);
        if (calibration.empty()) {
            SoapySDR_logf(SOAPY_SDR_INFO, "No calibration table found in %s", path.c_str());
        } else {
            SoapySDR_logf(SOAPY_SDR_INFO, "Loaded %zu calibration points from %s",
                calibration.size(), path.c_str());
            apply_calibration(SOAPY_SDR_RX);
            apply_calibration(SOAPY_SDR_TX);
        }
    }

    // Update corrections from calibration table
    // for the current frequency of given direction.
    // For TX, points calibrated with the current TX gains are used.
    // If there are none, points with any gains are used.
    void apply_calibration(const int direction)
    {
        std::scoped_lock lock(reg_mutex);
        if (calibration.empty())
            return;
        const double frequency = getFrequency(direction, 0);
        struct calibration_point point;
        if (direction == SOAPY_SDR_RX) {
            point = interpolate_calibration(calibration, frequency);
        } else {
            auto points = select_tx_points(calibration,
                getGain(SOAPY_SDR_TX, 0, "DAC"), getGain(SOAPY_SDR_TX, 0, "MIXER"), false);
            if (points.empty())
                points = select_tx_points(calibration, 0.0, 0.0, true);
            if (points.empty())
                return;
            point = interpolate_calibration(points, frequency);
        }
        std::scoped_lock corr_lock(corr_mutex);
        if (direction == SOAPY_SDR_RX) {
            table_rx_iq = point.rx_iq;
        } else {
            table_tx_dc = point.tx_dc;
            table_tx_iq = point.tx_iq;
        }
        update_corrections();
        SoapySDR_logf(SOAPY_SDR_DEBUG, "Calibration at %.0f Hz: tx_dc=%f%+fj tx_iq=%f%+fj rx_iq=%f%+fj",
            point.frequency,
            table_tx_dc.real(), table_tx_dc.imag(),
            table_tx_iq.real(), table_tx_iq.imag(),
            table_rx_iq.real(), table_rx_iq.imag());
    }

    bool does_synth_tune(double frequency)
    {
        setFrequency(SOAPY_SDR_RX, 0, frequency, {});
        setFrequency(SOAPY_SDR_TX, 0, frequency, {});
        // Give some time for PLL to lock
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        auto status_register = readRegister("", 0x11);
        return (status_register & 3) == 3;
    }

    void detect_clock(void)
    {
        // Try to detect whether SX1255 clock is 32 MHz or 38.4 MHz
        // by tuning near edges of tuning range.
        // First assume it is 32 MHz and try tuning accordingly.
        masterClock = 32.0e6;
        // If clock is 38.4 MHz instead, 510 MHz ends up at 612 MHz
        // where synthesizers (hopefully) do not lock anymore.
        bool tunes_high = does_synth_tune(510e6);
        // Synthesizers do not (hopefully) go down to 330 MHz, but
        // if clock is 38.4 MHz instead, they end up at 396 MHz.
        bool tunes_low  = does_synth_tune(330e6);
        if (tunes_low && (!tunes_high)) {
            SoapySDR_logf(SOAPY_SDR_INFO, "Detected clock as 38.4 MHz");
            masterClock = 38.4e6;
        } else if (tunes_high && (!tunes_low)) {
            SoapySDR_logf(SOAPY_SDR_INFO, "Detected clock as 32.0 MHz");
        } else {
            SoapySDR_logf(SOAPY_SDR_INFO, "Clock detection failed, assuming 38.4 MHz");
            masterClock = 38.4e6;
        }

        // Update default values for new masterClock value
        sampleRate = masterClock / 256.0;
        setFrequency(SOAPY_SDR_RX, 0, 433.92e6, {});
        setFrequency(SOAPY_SDR_TX, 0, 433.92e6, {});
    }

/***********************************************************************
 * Initialization and destruction
 **********************************************************************/

public:
    SoapySX(const SoapySDR::Kwargs &args, struct hat_info hat_info):
        masterClock(32.0e6),
        sampleRate(125.0e3),

        spi("/dev/spidev0.0"),

        gpio("/dev/gpiochip0"),
        gpio_reset(gpio,
            5,
            "SX reset",
            GPIO_V2_LINE_FLAG_OUTPUT | GPIO_V2_LINE_FLAG_OPEN_SOURCE,
            0
        ),
        gpio_rx(gpio,
            hat_info.product_ver == 0x0100 ? 13 : 23,
            "SX RX",
            GPIO_V2_LINE_FLAG_OUTPUT,
            1
        ),
        gpio_tx(gpio,
            hat_info.product_ver == 0x0100 ? 12 : 22,
            "SX TX",
            GPIO_V2_LINE_FLAG_OUTPUT,
            1
        ),

        alsa_rx(AlsaPcm("hw:CARD=SX1255,DEV=1", SND_PCM_STREAM_CAPTURE)),
        alsa_tx(AlsaPcm("hw:CARD=SX1255,DEV=0", SND_PCM_STREAM_PLAYBACK)),
        tx_threshold2(0.0f),
        linked(false),
        regs{0},

        // Allocate reasonably large buffers by default.
        // Their size will be increased if needed.
        buffer_rx(8192),
        buffer_tx(8192),

        hat_info(hat_info),

        rx_corr{true, 0.0, {0.0f, 0.0f}},
        tx_corr{{0.0f, 0.0f}, {0.0f, 0.0f}},
        table_rx_iq(0.0), table_tx_iq(0.0), table_tx_dc(0.0),
        user_rx_iq(0.0), user_tx_iq(0.0), user_tx_dc(0.0),
        rx_dc_cutoff(10.0),
        rx_dc_state(0.0, 0.0),
        auto_calibrate(false),
        calibration_tolerance(500e3),
        pa_mode("AUTO")
    {
        SoapySDR_logf(SOAPY_SDR_INFO, "Initializing SoapySX");

        // Settings from configuration file, overridden by device arguments
        SoapySDR::Kwargs config = read_config_file();
        for (const auto &it : args)
            config[it.first] = it.second;
        auto_calibrate = config.count("auto_calibrate") > 0 && config.at("auto_calibrate") == "1";
        if (config.count("calibration_tolerance") > 0)
            calibration_tolerance = std::stod(config.at("calibration_tolerance"));
        if (auto_calibrate)
            SoapySDR_logf(SOAPY_SDR_INFO, "Automatic calibration enabled");

        reset_chip();
        init_chip();
        // Load calibration before detect_clock
        // so that it gets applied to the initial frequency.
        // Calibration file path can be given as a device argument
        // calibration=/path/to/file. calibration=none disables it.
        try {
            load_calibration(config.count("calibration") > 0
                ? config.at("calibration")
                : default_calibration_path());
        } catch (std::exception &e) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "Calibration not used: %s", e.what());
        }
        detect_clock();
        update_rx_dc_alpha();
        // Open ALSA devices now when I2S clocks are already running.
        // I am not sure if this makes any difference but just in case.
        alsa_rx.open();
        alsa_tx.open();
    }

    ~SoapySX(void)
    {
        SoapySDR_logf(SOAPY_SDR_INFO, "Uninitializing SoapySX");

        // Put SX1255 to sleep
        set_register_bits(0, 0, 4, 0);
        write_registers_to_chip(0, 1);

        // Make sure PA is turned off
        writeSetting("PA", "OFF");
    }

/***********************************************************************
 * Sample streams
 **********************************************************************/

    SoapySDR::Stream *setupStream(
        const int direction,
        const std::string & format,
        const std::vector<size_t> & channels,
        const SoapySDR::Kwargs & args
    )
    {
        (void)channels; // Only one channel
        (void)args; // Unused for now

        std::scoped_lock lock(alsa_rx.mutex, alsa_tx.mutex);

        if (format != "CF32")
            throw std::runtime_error("Only CF32 format is currently supported");
        if (
            (snd_pcm_state(alsa_rx.pcm) == SND_PCM_STATE_RUNNING) ||
            (snd_pcm_state(alsa_tx.pcm) == SND_PCM_STATE_RUNNING)
        )
            throw std::runtime_error("Streams can be setup only if none of the streams are running");

        auto *stream = direction == SOAPY_SDR_RX ? &alsa_rx : &alsa_tx;

        // Allow setting up only one stream per direction.
        if (stream->setup_done)
            throw std::runtime_error("Stream has been setup already");

        if (stream->is_tx()) {
            const float tx_threshold_default = 1.0e-3;
            // Enable TX when magnitude of a sample exceeds a given threshold.
            float tx_threshold =
                (args.count("threshold") > 0)
                ? std::stof(args.at("threshold"))
                : tx_threshold_default;
            tx_threshold2 = tx_threshold * tx_threshold;
        }

        bool arg_link = (args.count("link") > 0 && args.at("link") == "1");
        stream->stream_mode = arg_link ? STREAM_MODE_LINK : STREAM_MODE_NORMAL;

        stream->configure(args.count("period") > 0 ? std::stoul(args.at("period")) : 0);

        stream->setup_done = 1;

        // Always link RX and TX PCMs if both have been setup.
        if ((!linked) && alsa_rx.setup_done && alsa_tx.setup_done) {
            SoapySDR_logf(SOAPY_SDR_DEBUG, "Linking streams");
            ALSACHECK(snd_pcm_link(alsa_rx.pcm, alsa_tx.pcm));
            linked = 1;
        }

        return reinterpret_cast<SoapySDR::Stream *>(stream);
    alsa_error:
        // TODO more detailed error messages?
        throw std::runtime_error("ALSA error");
    }

    void closeStream(SoapySDR::Stream * handle)
    {
        auto *stream = reinterpret_cast<AlsaPcm *>(handle);
        std::scoped_lock lock(stream->mutex);
        stream->setup_done = 0;
    }

    int activateStream(
        SoapySDR::Stream * handle,
        const int flags,
        const long long timeNs,
        const size_t numElems
    )
    {
        (void)flags; (void)timeNs; (void)numElems;

        std::scoped_lock rx_lock(alsa_rx.mutex, alsa_tx.mutex);

        auto *stream = reinterpret_cast<AlsaPcm *>(handle);
        if (stream->activated) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "Stream was already activated");
            return SOAPY_SDR_STREAM_ERROR;
        }

        // Run automatic calibration before streams start
        if (auto_calibrate && !alsa_rx.activated && !alsa_tx.activated && calibration_needed())
            run_calibration();

        stream->activated = 1;

        if (stream->stream_mode == STREAM_MODE_NORMAL) {
            if (snd_pcm_state(stream->pcm) == SND_PCM_STATE_PREPARED) {
                ALSACHECK(snd_pcm_start(stream->pcm));
            }
        }

        return 0;
    alsa_error:
        return SOAPY_SDR_STREAM_ERROR;
    }

    int deactivateStream(
        SoapySDR::Stream * handle,
        const int flags,
        const long long timeNs
    )
    {
        (void)flags; (void)timeNs;

        std::scoped_lock rx_lock(alsa_rx.mutex, alsa_tx.mutex);

        auto *stream = reinterpret_cast<AlsaPcm *>(handle);
        if (!stream->activated) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "Stream was already deactivated");
            return SOAPY_SDR_STREAM_ERROR;
        }
        stream->activated = 0;

        // If both streams have been deactivated, stop them.
        if ((!alsa_rx.activated) && (!alsa_tx.activated)) {
            SoapySDR_logf(SOAPY_SDR_INFO, "Stopping and resetting streams");
            ALSACHECK(alsa_rx.reset());
            ALSACHECK(alsa_tx.reset());
        }

        return 0;
    alsa_error:
        return SOAPY_SDR_STREAM_ERROR;
    }

    size_t getStreamMTU(SoapySDR::Stream * handle) const
    {
        auto *stream = reinterpret_cast<AlsaPcm *>(handle);
        std::scoped_lock lock(stream->mutex);
        return stream->hwp_period_size;
    }

    int readStream(
        SoapySDR::Stream * handle,
        void *const * buffs,
        const size_t numElems,
        int & flags,
        long long & timeNs,
        const long timeoutUs
    )
    {
        auto *stream = reinterpret_cast<AlsaPcm *>(handle);
        std::scoped_lock lock(stream->mutex);

        flags = 0;

        if (stream->is_tx())
            throw std::runtime_error("Wrong direction");

        snd_pcm_t *pcm = stream->pcm;

        // If readStream is called before ALSA stream has been started,
        // it will get stuck forever in snd_pcm_wait.
        // Looks like this was the reason SoapyRemote did not work before.
        // Proper implementation of timeout might fix this too but that
        // might need replacing snd_pcm_wait with something more complicated.
        // For now, handle it by returning if the stream is not active.
        if (!stream->activated)
            return 0;


        snd_pcm_sframes_t pcm_avail = 0, pcm_delay = 0;
        int avail_ret = snd_pcm_avail_delay(pcm, &pcm_avail, &pcm_delay);
        if (avail_ret < 0) {
            // TODO: check if it can fail and think what to actually do here
            SoapySDR_logf(SOAPY_SDR_ERROR, "rx snd_pcm_avail_delay: %d", avail_ret);
            return alsa_error_to_soapy_rx(avail_ret);
        }
        SoapySDR_logf(SOAPY_SDR_DEBUG, "rx snd_pcm_avail_delay: %d %ld %ld", avail_ret, pcm_avail, pcm_delay);


        // If number of available samples is more than the ALSA buffer size,
        // it means the buffer has overrun and old samples have been overwritten.
        // Skip these samples.
        if (pcm_avail > (snd_pcm_sframes_t)stream->hwp_buffer_size) {
            snd_pcm_uframes_t overwritten = (snd_pcm_uframes_t)pcm_avail - stream->hwp_buffer_size;
            // Round the number of samples to skip to a multiple of period size,
            // so applications that use reads aligned to periods will stay aligned.
            // Add 1-2 extra periods for some margin.
            snd_pcm_uframes_t samples_to_skip = (overwritten / stream->hwp_period_size + 2) * stream->hwp_period_size;
            snd_pcm_sframes_t forwarded = snd_pcm_forward(pcm, samples_to_skip);
            if (forwarded >= 0) {
                stream->position += forwarded;
                pcm_avail -= forwarded;

                SoapySDR_logf(SOAPY_SDR_WARNING, "RX buffer overrun. Skipped %u samples", forwarded);
            } else {
                // TODO: check if it can fail and think what to actually do here
                SoapySDR_logf(SOAPY_SDR_ERROR, "rx snd_pcm_forward: %d", forwarded);
                return alsa_error_to_soapy_rx((int)forwarded);
            }
        }


        // Number of samples to read.
        // snd_pcm_uframes_t is unsigned long. Clamp to its maximum value just in case.
        snd_pcm_uframes_t length = (snd_pcm_uframes_t)std::min(numElems, (size_t)ULONG_MAX);

        if (timeoutUs <= 0) {
            // Hack to make non-blocking read work:
            // limit number of samples to what is available right now.
            if (pcm_avail <= 0) {
                length = 0;
            } else if ((snd_pcm_uframes_t)pcm_avail < length) {
                length = (snd_pcm_uframes_t)pcm_avail;
            }
        }

        if (length > buffer_rx.size())
            buffer_rx.resize(length);

        if (length > 0) {
            snd_pcm_sframes_t samples_read = snd_pcm_readi(pcm, buffer_rx.data(), length);
            if (samples_read >= 0) {
                timeNs = samples_to_timestamp(stream->position);
                flags |= SOAPY_SDR_HAS_TIME;

                stream->position += samples_read;

                assert((size_t)samples_read <= buffer_rx.size());
                assert((size_t)samples_read <= numElems);
                struct rx_corrections corr;
                {
                    std::scoped_lock corr_lock(corr_mutex);
                    corr = rx_corr;
                }
                convert_rx_buffer(buffer_rx.data(), 0, buffs[0], 0, samples_read, corr, rx_dc_state);

                return (int)samples_read;
            } else {
                return alsa_error_to_soapy_rx((int)samples_read);
            }
        } else {
            // Nothing to read
            return 0;
        }
    }

    int writeStream(
        SoapySDR::Stream * handle,
        const void *const * buffs,
        const size_t numElems,
        int & flags,
        long long timeNs,
        const long timeoutUs
    )
    {
        auto *stream = reinterpret_cast<AlsaPcm *>(handle);
        std::scoped_lock lock(stream->mutex);

        if (!stream->is_tx())
            throw std::runtime_error("Wrong direction");

        if (!stream->activated)
            return 0;

        snd_pcm_t *pcm = stream->pcm;

        snd_pcm_sframes_t pcm_avail = 0, pcm_delay = 0;
        int avail_ret = snd_pcm_avail_delay(pcm, &pcm_avail, &pcm_delay);
        if (avail_ret < 0) {
            // TODO: check if it can fail and think what to actually do here
            SoapySDR_logf(SOAPY_SDR_ERROR, "tx snd_pcm_avail_delay: %d", avail_ret);
            return alsa_error_to_soapy_tx(avail_ret);
        }
        SoapySDR_logf(SOAPY_SDR_DEBUG, "tx snd_pcm_avail_delay: %d %ld %ld", avail_ret, pcm_avail, pcm_delay);

        // Current position where stream is being "played",
        // similar to getHardwareTime:
        int64_t playback_position = stream->position - (int64_t)pcm_delay;

        // Position where samples are going to be written
        int64_t write_position;

        // Number of samples to write.
        // snd_pcm_uframes_t is unsigned long. Clamp to its maximum value just in case.
        snd_pcm_uframes_t length = (snd_pcm_uframes_t)std::min(numElems, (size_t)ULONG_MAX);

        if (flags & SOAPY_SDR_HAS_TIME) {
            // Timestamp provided.
            // Write to the position corresponding to the timestamp.
            write_position = timestamp_to_samples(timeNs);
            // If timestamp is in the past, quietly discard the samples.
            // It might be more "correct" to return SOAPY_SDR_TIME_ERROR instead,
            // but seems like discarding is a common behavior with other SDRs, so
            // I am worried some applications might not properly handle the error code.
            int64_t diff = playback_position - write_position;
            if (diff > 0) {
                // Timestamp is in the past.
                // Do nothing but pretend all samples were written.
                SoapySDR_logf(SOAPY_SDR_WARNING, "Discarding TX %d samples in the past", diff);
                return (int)length;
            }
        } else {
            // No timestamp provided.
            // Continue writing from where last write ended.
            write_position = stream->position;
            // If TX buffer has underrun, skip to a future position.
            // Round the number of samples to skip to a multiple of period size,
            // so applications that use writes aligned to periods will stay aligned.
            // Add 1-2 extra periods for some margin.
            int64_t diff = playback_position - write_position;
            if (diff > 0) {
                diff = (diff / (int64_t)stream->hwp_period_size + 2) * (int64_t)stream->hwp_period_size;
                write_position += diff;
                SoapySDR_logf(SOAPY_SDR_WARNING, "TX buffer underrun. Forwarding TX stream by %d samples", diff);
            }
        }


        // How much the stream should be forwarded
        // to get samples written to write_position.
        int64_t posdiff = write_position - stream->position;

        while (posdiff > 0) {
            // snd_pcm_sframes_t is long. Clamp to its maximum value just in case.
            snd_pcm_sframes_t samples_to_forward = (snd_pcm_sframes_t)std::min(posdiff, (int64_t)LONG_MAX);

            snd_pcm_sframes_t forwardable = snd_pcm_forwardable(pcm);
            if (forwardable < 0) {
                // TODO: check if it can fail and think what to actually do here
                SoapySDR_logf(SOAPY_SDR_ERROR, "tx snd_pcm_forwardable: %d", forwardable);
                return alsa_error_to_soapy_tx((int)forwardable);
            }

            snd_pcm_sframes_t forwarded;
            if (samples_to_forward < forwardable) {
                forwarded = snd_pcm_forward(pcm, samples_to_forward);
            } else {
                // Forward as much as possible, then wait for more space
                forwarded = snd_pcm_forward(pcm, forwardable);
                snd_pcm_wait(pcm, -10001);
            }

            if (forwarded < 0) {
                // TODO: check if it can fail and think what to actually do here
                SoapySDR_logf(SOAPY_SDR_ERROR, "tx snd_pcm_forward: %d", forwarded);
                return alsa_error_to_soapy_tx((int)forwarded);
            }
            stream->position += forwarded;
            posdiff -= forwarded;
            pcm_avail -= forwarded;
        }


        if (timeoutUs <= 0) {
            // Hack to make non-blocking write work:
            // limit number of samples to the amount of space in the buffer now.

            if (pcm_avail <= 0) {
                length = 0;
            } else if ((snd_pcm_uframes_t)pcm_avail < length) {
                length = (snd_pcm_uframes_t)pcm_avail;
            }
        }

        if (length > buffer_tx.size())
            buffer_tx.resize(length);

        struct tx_corrections corr;
        {
            std::scoped_lock corr_lock(corr_mutex);
            corr = tx_corr;
        }
        convert_tx_buffer(buffs[0], 0, buffer_tx.data(), 0, length, tx_threshold2, corr);

        if (length > 0) {
            snd_pcm_sframes_t samples_written = snd_pcm_writei(pcm, buffer_tx.data(), length);
            if (samples_written >= 0) {
                // Successful write
                stream->position += samples_written;
                return (int)samples_written;
            } else {
                return alsa_error_to_soapy_tx((int)samples_written);
            }
        } else {
            // Nothing to write
            return 0;
        }
    }

    long long getHardwareTime(const std::string &what) const
    {
        if (what == "") {
            // RX and TX streams should give similar results here.
            //
            // Use TX stream to avoid locking the RX stream mutex.
            // Reasoning is that getHardwareTime seems most useful
            // for determining timing of producing TX signals,
            // and some applications might run RX and TX in different threads,
            // we want to avoid locks between them, and I guess
            // getHardwareTime is most likely to be called from a TX thread.
            //
            // The mutex is needed to avoid race conditions between updating
            // our stream->position and the ALSA internal stream position,
            // in case getHardwareTime gets called from some other thread
            // while writeStream is running.

            auto *stream = &alsa_tx;
            std::scoped_lock lock(stream->mutex);
            snd_pcm_t *pcm = stream->pcm;

            snd_pcm_sframes_t pcm_avail = 0, pcm_delay = 0;
            int ret = snd_pcm_avail_delay(pcm, &pcm_avail, &pcm_delay);
            SoapySDR_logf(SOAPY_SDR_DEBUG, "hwt snd_pcm_avail_delay: %d %ld %ld  pos: %ld", ret, pcm_avail, pcm_delay, stream->position);
            if (ret < 0) {
                throw std::runtime_error("ALSA error");
            } else {
                return samples_to_timestamp(stream->position - (int64_t)pcm_delay);
            }
        } else {
            throw std::runtime_error("Unsupported time");
        }
    }

/***********************************************************************
 * Sample rates
 **********************************************************************/

    std::vector<double> listSampleRates(const int direction, const size_t channel) const
    {
        (void)direction; (void)channel;
        std::vector<double> sampleRates;
        for (size_t i = 0; i < N_SAMPLE_RATES; i++) {
            sampleRates.push_back(masterClock / (double)sample_rates[i].div);
        }
        return sampleRates;
    }

    // Wrapper for listSampleRates to support both methods
    SoapySDR::RangeList getSampleRateRange(const int direction, const size_t channel) const
    {
        const auto sampleRates = listSampleRates(direction, channel);
        SoapySDR::RangeList ranges;
        for (const auto sampleRate: sampleRates) {
            ranges.push_back({sampleRate, sampleRate, 0});
        }
        return ranges;
    }

    void setSampleRate(
        const int direction,
        const size_t channel,
        const double rate
    )
    {
        (void)direction; (void)channel;

        std::scoped_lock lock(reg_mutex);

        if (rate != rate || rate <= 0)
            throw std::runtime_error("Sample rate must be positive");

        double divider = round(masterClock / rate);
        struct sampleRateRegs r;
        bool found = false;
        for (size_t i = 0; i < N_SAMPLE_RATES; i++) {
            r = sample_rates[i];
            if ((double)r.div == divider) {
                found = true;
                break;
            }
        }
        if (!found) {
            throw std::runtime_error("Unsupported sample rate");
        }
        // Disable RX and TX before changing sample rate.
        // Changing sample rate while RX/TX was running sometimes seemed
        // to break ordering of data over I2S, causing things like swapped
        // left and right channels or otherwise corrupted data.
        // This seems to fix the problem.
        set_register_bits(0x00, 1, 2, 0);
        write_registers_to_chip(0x00, 1);
        // Change the sample rate
        set_register_bits(0x12, 0, 4, r.clkout);
        set_register_bits(0x13, 7, 1, r.mant);
        set_register_bits(0x13, 6, 1, r.m);
        set_register_bits(0x13, 3, 3, r.n);
        write_registers_to_chip(0x12, 2);
        sampleRate = masterClock / divider;
        // Enable RX and TX again
        set_register_bits(0x00, 1, 2, 3);
        write_registers_to_chip(0x00, 1);
        update_rx_dc_alpha();
    }

    double getSampleRate(
        const int direction,
        const size_t channel
    ) const
    {
        (void)direction; (void)channel;
        std::scoped_lock lock(reg_mutex);
        return sampleRate;
    }

/***********************************************************************
 * Center frequency
 **********************************************************************/

    void setFrequency(
        const int direction,
        const size_t channel,
        const double frequency,
        const SoapySDR::Kwargs & args
    )
    {
        (void)channel; (void)args;

        std::scoped_lock lock(reg_mutex);
        write_frequency(direction, frequency);
        apply_calibration(direction);
    }

    // Write synthesizer frequency registers
    // without updating corrections.
    void write_frequency(const int direction, const double frequency)
    {
        std::scoped_lock lock(reg_mutex);

        const double step = masterClock * (1.0 / (double)(1L<<20));
        const uint32_t quantized = (uint32_t)scale_from_range(
            SoapySDR::Range(0, step * (double)((1L<<24)-1), step),
            frequency);
        if (direction == SOAPY_SDR_RX) {
            set_register_bits(0x01, 0, 8, quantized >> 16);
            set_register_bits(0x02, 0, 8, (quantized >> 8) & 0xFF);
            set_register_bits(0x03, 0, 8, quantized & 0xFF);
            write_registers_to_chip(0x01, 3);
        } else {
            set_register_bits(0x04, 0, 8, quantized >> 16);
            set_register_bits(0x05, 0, 8, (quantized >> 8) & 0xFF);
            set_register_bits(0x06, 0, 8, quantized & 0xFF);
            write_registers_to_chip(0x04, 3);
        }
    }

    double getFrequency(
        const int direction,
        const size_t channel
    ) const
    {
        (void)channel;

        std::scoped_lock lock(reg_mutex);

        const double step = masterClock * (1.0 / (double)(1L<<20));
        if (direction == SOAPY_SDR_RX)
            return step * (
                (((uint32_t)regs[1]) << 16) |
                (((uint32_t)regs[2]) << 8) |
                 ((uint32_t)regs[3]));
        else
            return step * (
                (((uint32_t)regs[4]) << 16) |
                (((uint32_t)regs[5]) << 8) |
                 ((uint32_t)regs[6]));
    }

/***********************************************************************
 * Gains
 **********************************************************************/

    std::vector<std::string> listGains(
        const int direction,
        const size_t channel
    ) const
    {
        (void)channel;
        if (direction == SOAPY_SDR_RX)
            return std::vector<std::string>{"LNA", "PGA"};
        else
            return std::vector<std::string>{"DAC", "MIXER"};
    }

    SoapySDR::Range getGainRange(
        const int direction,
        const size_t channel,
        const std::string & name
    ) const
    {
        (void)channel;
        if (direction == SOAPY_SDR_RX) {
            if (name == "LNA")   return {  0.0 , 48.0 , 6.0 };
            if (name == "PGA")   return {  0.0 , 30.0 , 2.0 };
        } else {
            if (name == "DAC")   return {  0.0 ,  9.0 , 3.0 };
            if (name == "MIXER") return {  0.0 , 30.0 , 2.0 };
        }
        return {0, 0, 0};
    }

    void setGain(
        const int direction,
        const size_t channel,
        const std::string & name,
        const double value
    )
    {
        std::scoped_lock lock(reg_mutex);

        int32_t quantized = scale_from_range(getGainRange(direction, channel, name), value);
        if (direction == SOAPY_SDR_RX) {
            if (name == "LNA") {
                // LNA gain does not have a constant step,
                // so some extra logic is needed.
                if (quantized <= 6) // -48 to -12 dB
                    set_register_bits(0x0C, 5, 3, 6-quantized/2);
                else if (quantized == 7) // -6 dB
                    set_register_bits(0x0C, 5, 3, 2);
                else // 0 dB
                    set_register_bits(0x0C, 5, 3, 1);
            } else if (name == "PGA") {
                set_register_bits(0x0C, 1, 4, quantized);
            }
            SoapySDR_logf(SOAPY_SDR_DEBUG, "RXFE1=0x%02x", regs[0x0C]);
            write_registers_to_chip(0x0C, 1);
        } else {
            if (name == "DAC") {
                set_register_bits(0x08, 4, 3, quantized);
            } else if (name == "MIXER") {
                set_register_bits(0x08, 0, 4, quantized);
            }
            SoapySDR_logf(SOAPY_SDR_DEBUG, "TXFE1=0x%02x", regs[0x08]);
            write_registers_to_chip(0x08, 1);
            // TX calibration depends on gains
            apply_calibration(SOAPY_SDR_TX);
        }
    }

    double getGain(
        const int direction,
        const size_t channel,
        const std::string & name
    ) const
    {
        std::scoped_lock lock(reg_mutex);

        int32_t quantized = 0;
        if (direction == SOAPY_SDR_RX) {
            if (name == "LNA") {
                const int32_t map[8] = {0, 8, 7, 6, 4, 2, 0, 0};
                quantized = map[get_cached_register_bits(0x0C, 5, 3)];
            } else if (name == "PGA") {
                quantized = get_cached_register_bits(0x0C, 1, 4);
            }
        } else {
            if (name == "DAC") {
                quantized = get_cached_register_bits(0x08, 4, 3);
            } else if (name == "MIXER") {
                quantized = get_cached_register_bits(0x08, 0, 4);
            }
        }
        return scale_to_range(getGainRange(direction, channel, name), quantized);
    }

    void setGain(
        const int direction,
        const size_t channel,
        const double value
    )
    {
        std::scoped_lock lock(reg_mutex);

        if (direction == SOAPY_SDR_RX) {
            // Keep PGA gain around pga_gain_target over most of the
            // gain range while adjusting LNA gain over a wide range.
            // PGA gain has a smaller step, so use it to fine tune gain.
            const double pga_gain_target = 12.0;
            setGain(direction, channel, "LNA", value - pga_gain_target);
            double lna_gain = getGain(direction, channel, "LNA");
            setGain(direction, channel, "PGA", value - lna_gain);
        } else {
            // Not sure about best TX gain distribution yet.
            // Use similar logic as RX gains for now.
            const double mixer_gain_target = 26.0;
            setGain(direction, channel, "DAC", value - mixer_gain_target);
            double dac_gain = getGain(direction, channel, "DAC");
            setGain(direction, channel, "MIXER", value - dac_gain);
        }
    }

/***********************************************************************
 * Antennas
 **********************************************************************/

    std::vector<std::string> listAntennas(const int direction, const size_t channel) const
    {
        (void)channel; // Only one channel
        std::vector<std::string> antennas;
        if (direction == SOAPY_SDR_RX) {
            antennas.push_back("RX");
            antennas.push_back("LB");
            // Digital loopback did not seem to work, so do not list it
            //antennas.push_back("DLB");
        } else {
            antennas.push_back("TX");
            antennas.push_back("NONE");
        }
        return antennas;
    }

    void setAntenna(const int direction, const size_t channel, const std::string &name)
    {
        (void)channel;
        std::scoped_lock lock(reg_mutex);

        if (direction == SOAPY_SDR_RX) {
            if (name == "RX") {
                // Disable loopback
                set_register_bits(0x10, 2, 2, 0);
            } else if (name == "LB") {
                // RF loopback
                set_register_bits(0x10, 2, 2, 1);
            } else if (name == "DLB") {
                // Digital loopback
                set_register_bits(0x10, 2, 2, 3);
            }
            write_registers_to_chip(0x10, 1);
        } else {
            if (name == "TX") {
                // Enable PA
                set_register_bits(0x00, 3, 1, 1);
            } else if (name == "NONE") {
                // Disable PA
                set_register_bits(0x00, 3, 1, 0);
            }
            write_registers_to_chip(0x00, 1);
        }
    }

    std::string getAntenna(const int direction, const size_t channel) const
    {
        (void)channel;
        std::scoped_lock lock(reg_mutex);

        if (direction == SOAPY_SDR_RX) {
            unsigned lb = get_cached_register_bits(0x10, 2, 2);
            if (lb & 2)
                return "DLB";
            if (lb & 1)
                return "LB";
            return "RX";
        } else {
            if (get_cached_register_bits(0x00, 3, 1)) {
                // PA is enabled
                return "TX";
            } else {
                // PA is disabled
                return "NONE";
            }
        }
    }

/***********************************************************************
 * DC offset and IQ balance corrections
 **********************************************************************/

    // Automatic DC offset removal is supported for RX.
    bool hasDCOffsetMode(const int direction, const size_t channel) const
    {
        (void)channel;
        return direction == SOAPY_SDR_RX;
    }

    void setDCOffsetMode(const int direction, const size_t channel, const bool automatic)
    {
        (void)channel;
        if (direction != SOAPY_SDR_RX)
            return;
        std::scoped_lock lock(corr_mutex);
        rx_corr.dc_removal = automatic;
    }

    bool getDCOffsetMode(const int direction, const size_t channel) const
    {
        (void)channel;
        if (direction != SOAPY_SDR_RX)
            return false;
        std::scoped_lock lock(corr_mutex);
        return rx_corr.dc_removal;
    }

    // Manual DC offset correction is supported for TX
    // to cancel LO leakage. It is added to the value from calibration table.
    bool hasDCOffset(const int direction, const size_t channel) const
    {
        (void)channel;
        return direction == SOAPY_SDR_TX;
    }

    void setDCOffset(const int direction, const size_t channel, const std::complex<double> &offset)
    {
        (void)channel;
        if (direction != SOAPY_SDR_TX)
            return;
        std::scoped_lock lock(corr_mutex);
        user_tx_dc = offset;
        update_corrections();
    }

    std::complex<double> getDCOffset(const int direction, const size_t channel) const
    {
        (void)channel;
        if (direction != SOAPY_SDR_TX)
            return 0.0;
        std::scoped_lock lock(corr_mutex);
        return user_tx_dc;
    }

    // IQ balance correction is applied as y = x + balance * conj(x)
    // to both received and transmitted samples.
    // Balance is added to the value from calibration table.
    bool hasIQBalance(const int direction, const size_t channel) const
    {
        (void)direction; (void)channel;
        return true;
    }

    void setIQBalance(const int direction, const size_t channel, const std::complex<double> &balance)
    {
        (void)channel;
        std::scoped_lock lock(corr_mutex);
        if (direction == SOAPY_SDR_RX)
            user_rx_iq = balance;
        else
            user_tx_iq = balance;
        update_corrections();
    }

    std::complex<double> getIQBalance(const int direction, const size_t channel) const
    {
        (void)channel;
        std::scoped_lock lock(corr_mutex);
        if (direction == SOAPY_SDR_RX)
            return user_rx_iq;
        else
            return user_tx_iq;
    }

/***********************************************************************
 * Built-in calibration
 **********************************************************************/
private:

    // State of a running calibration
    struct cal_state {
        // Samples read from RX and written to TX
        int64_t rx_count, tx_count;
        // RX sample index from which on current settings are in effect
        int64_t valid_from;
        // Normalized frequency difference of TX and RX LO
        double lo_offset;
        // Extra samples to skip after next change, e.g. for PLL lock
        int64_t extra_settle;
        // Corrections being tested
        struct rx_corrections rx;
        struct tx_corrections tx;
        // Raw sample buffers
        std::vector<int32_t> raw;
        std::vector<std::complex<float>> tone, converted;
    };

    static const size_t CAL_CHUNK = 1024;
    // How far ahead TX is written
    static const snd_pcm_sframes_t CAL_TX_AHEAD = 16384;
    // Samples to skip after a change, for filters to settle
    static const int64_t CAL_SETTLE = 2048;
    static const size_t CAL_LENGTH = 8192;
    static constexpr float CAL_AMPLITUDE = 0.25f;
    static constexpr double CAL_MIN_SNR = 40.0;

    bool calibration_needed(void)
    {
        std::scoped_lock lock(reg_mutex);
        auto tx_points = select_tx_points(calibration,
            getGain(SOAPY_SDR_TX, 0, "DAC"), getGain(SOAPY_SDR_TX, 0, "MIXER"), false);
        // Points with unknown TX gains do not count
        tx_points.erase(std::remove_if(tx_points.begin(), tx_points.end(),
            [](const struct calibration_point &p) { return std::isnan(p.tx_dac); }),
            tx_points.end());
        return nearest_point_distance(tx_points, getFrequency(SOAPY_SDR_TX, 0)) > calibration_tolerance
            || nearest_point_distance(calibration, getFrequency(SOAPY_SDR_RX, 0)) > calibration_tolerance;
    }

    // Write TX samples to keep TX CAL_TX_AHEAD samples ahead,
    // then read one chunk of RX samples into st.converted.
    void cal_pump(struct cal_state &st)
    {
        snd_pcm_sframes_t delay = 0;
        if (snd_pcm_delay(alsa_tx.pcm, &delay) < 0)
            delay = 0;
        if (delay < 0) {
            // TX has underrun. Skip forward to keep TX sample count
            // in sync with time, so that the tone stays continuous.
            snd_pcm_sframes_t forwarded = snd_pcm_forward(alsa_tx.pcm, -delay);
            if (forwarded > 0)
                st.tx_count += forwarded;
            delay = 0;
        }
        while (delay < CAL_TX_AHEAD) {
            for (size_t i = 0; i < CAL_CHUNK; i++) {
                double phase = 2.0 * M_PI * std::fmod(CAL_TONE * (double)(st.tx_count + (int64_t)i), 1.0);
                st.tone[i] = std::polar(CAL_AMPLITUDE, (float)phase);
            }
            convert_tx_buffer(st.tone.data(), 0, st.raw.data(), 0, CAL_CHUNK, 0.0f, st.tx);
            snd_pcm_sframes_t ret = snd_pcm_writei(alsa_tx.pcm, st.raw.data(), CAL_CHUNK);
            if (ret < 0)
                throw std::runtime_error("TX error during calibration: " + std::string(snd_strerror((int)ret)));
            st.tx_count += ret;
            delay += ret;
        }
        if (snd_pcm_state(alsa_rx.pcm) != SND_PCM_STATE_RUNNING)
            return;
        snd_pcm_sframes_t ret = snd_pcm_readi(alsa_rx.pcm, st.raw.data(), CAL_CHUNK);
        if (ret < 0)
            throw std::runtime_error("RX error during calibration: " + std::string(snd_strerror((int)ret)));
        std::complex<double> dc_state = 0.0;
        st.converted.resize(ret);
        convert_rx_buffer(st.raw.data(), 0, st.converted.data(), 0, ret, st.rx, dc_state);
        st.rx_count += ret;
    }

    // Mark samples received before current settings take effect as invalid.
    void cal_settings_changed(struct cal_state &st)
    {
        snd_pcm_sframes_t rx_avail = snd_pcm_avail(alsa_rx.pcm);
        snd_pcm_sframes_t tx_delay = 0;
        if (snd_pcm_delay(alsa_tx.pcm, &tx_delay) < 0)
            tx_delay = 0;
        // RX sample being received now, plus time TX samples
        // written from now on take to be transmitted.
        st.valid_from = st.rx_count + std::max(rx_avail, (snd_pcm_sframes_t)0)
            + std::max(tx_delay, (snd_pcm_sframes_t)0) + CAL_SETTLE + st.extra_settle;
        st.extra_settle = 0;
    }

    struct cal_measurement cal_measure(struct cal_state &st)
    {
        std::vector<std::complex<float>> x;
        x.reserve(CAL_LENGTH);
        int64_t n0 = -1;
        while (x.size() < CAL_LENGTH) {
            int64_t start = st.rx_count;
            cal_pump(st);
            for (size_t i = 0; i < st.converted.size() && x.size() < CAL_LENGTH; i++) {
                if (start + (int64_t)i < st.valid_from)
                    continue;
                if (n0 < 0)
                    n0 = start + (int64_t)i;
                x.push_back(st.converted[i]);
            }
        }
        return cal_analyze(x, n0, st.lo_offset);
    }

    // Find value of a correction nulling a complex measurement.
    // Jacobian with respect to real and imaginary part of the correction
    // is estimated from perturbations, so unknown phase shifts in the
    // loopback do not matter. Returns levels in dB before and after.
    struct cal_levels {
        double before, after;
    };

    struct cal_levels cal_null_search(
        struct cal_state &st,
        std::complex<float> &value,
        std::complex<double> (*objective)(const struct cal_measurement &))
    {
        auto evaluate = [&](std::complex<double> v) {
            value = std::complex<float>(v);
            cal_settings_changed(st);
            return objective(cal_measure(st));
        };
        const double target = std::pow(10.0, -70.0 / 20.0);
        double step = 0.01;
        std::complex<double> p = std::complex<double>(value);
        std::complex<double> f0 = evaluate(p);
        const double first = std::abs(f0);
        double best_level = first;
        std::complex<double> best = p;
        for (int iteration = 0; iteration < 4 && std::abs(f0) > target; iteration++) {
            std::complex<double> fx = evaluate(p + step);
            std::complex<double> fy = evaluate(p + std::complex<double>(0.0, step));
            // Solve J * dp = -f0, where columns of J are (fx-f0)/step and (fy-f0)/step
            double a = (fx - f0).real() / step, b = (fy - f0).real() / step;
            double c = (fx - f0).imag() / step, d = (fy - f0).imag() / step;
            double det = a * d - b * c;
            if (det == 0.0 || !std::isfinite(det))
                break;
            std::complex<double> dp(
                (-f0.real() * d + f0.imag() * b) / det,
                (-f0.imag() * a + f0.real() * c) / det);
            p += dp;
            f0 = evaluate(p);
            if (std::abs(f0) < best_level) {
                best_level = std::abs(f0);
                best = p;
            }
            step = std::min(step, std::max(std::abs(dp), step / 10.0));
        }
        value = std::complex<float>(best);
        return {to_db(first), to_db(best_level)};
    }

    // Choose RX PGA gain giving a reasonable level for the loopback signal.
    void cal_adjust_rx_gain(struct cal_state &st)
    {
        unsigned pga = 15;
        while (true) {
            set_register_bits(0x0C, 1, 4, pga);
            write_registers_to_chip(0x0C, 1);
            cal_settings_changed(st);
            auto m = cal_measure(st);
            if (m.peak < 0.3 || pga < 2)
                break;
            pga -= 2;
        }
    }

    // Calibrate with TX LO at tx_frequency and RX LO below it.
    // If rx_only is set, only RX IQ balance is calibrated.
    // Returns false if loopback signal is not received.
    bool cal_pass(struct cal_state &st, double tx_frequency, bool rx_only)
    {
        const double step = masterClock * (1.0 / (double)(1L<<20));
        const double offset = std::round(CAL_LO_OFFSET * sampleRate / step) * step;
        write_frequency(SOAPY_SDR_TX, tx_frequency);
        write_frequency(SOAPY_SDR_RX, tx_frequency - offset);
        st.lo_offset = (getFrequency(SOAPY_SDR_TX, 0) - getFrequency(SOAPY_SDR_RX, 0)) / sampleRate;
        // Wait 20 ms for PLLs to lock. Streams keep running meanwhile.
        st.extra_settle = (int64_t)(0.02 * sampleRate);

        cal_adjust_rx_gain(st);
        cal_settings_changed(st);
        auto m = cal_measure(st);
        const double snr = to_db(std::abs(m.tone)) - to_db(m.noise);
        SoapySDR_logf(SOAPY_SDR_INFO, "Calibrating at TX %.3f MHz: tone %.1f dBFS, SNR %.1f dB",
            tx_frequency * 1e-6, to_db(std::abs(m.tone)), snr);
        if (snr < CAL_MIN_SNR) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "Calibration failed: loopback signal not received. "
                "Check with tools/calibrate.py --diagnose.");
            return false;
        }

        auto rx = cal_null_search(st, st.rx.iq, [](const struct cal_measurement &m) {
            return m.rx_image / std::conj(m.tone);
        });
        SoapySDR_logf(SOAPY_SDR_INFO, "  RX image      %6.1f dBc -> %6.1f dBc", rx.before, rx.after);
        if (rx_only)
            return true;
        auto tx_image = cal_null_search(st, st.tx.iq, [](const struct cal_measurement &m) {
            return m.tx_image / m.tone;
        });
        SoapySDR_logf(SOAPY_SDR_INFO, "  TX image      %6.1f dBc -> %6.1f dBc", tx_image.before, tx_image.after);
        auto tx_lo = cal_null_search(st, st.tx.dc, [](const struct cal_measurement &m) {
            return m.tx_lo / m.tone;
        });
        SoapySDR_logf(SOAPY_SDR_INFO, "  TX LO leakage %6.1f dBc -> %6.1f dBc", tx_lo.before, tx_lo.after);
        return true;
    }

    // Add points to calibration table and append them to calibration file.
    void add_calibration_points(const std::vector<struct calibration_point> &points)
    {
        // Replace existing points for the same frequency and gains
        for (const auto &p : points) {
            calibration.erase(std::remove_if(calibration.begin(), calibration.end(),
                [&](const struct calibration_point &q) {
                    return q.frequency == p.frequency && q.rx_only == p.rx_only
                        && (p.rx_only || (q.tx_dac == p.tx_dac && q.tx_mixer == p.tx_mixer));
                }), calibration.end());
            calibration.push_back(p);
        }
        std::sort(calibration.begin(), calibration.end(),
            [](const struct calibration_point &a, const struct calibration_point &b) {
                return a.frequency < b.frequency;
            });
        if (calibration_path.empty()) {
            SoapySDR_logf(SOAPY_SDR_INFO, "Calibration file disabled, result not saved");
            return;
        }
        // Rewrite the whole file so that replaced points are removed
        create_parent_directories(calibration_path);
        std::ofstream out(calibration_path + ".tmp");
        out << "# SoapySX calibration table\n";
        out << "# Corrections are applied as y = x + iq * conj(x) + dc\n";
        out << "# frequency_hz tx_dc_re tx_dc_im tx_iq_re tx_iq_im rx_iq_re rx_iq_im tx_dac tx_mixer\n";
        for (const auto &p : calibration)
            write_calibration_point(out, p);
        out.close();
        if (!out || rename((calibration_path + ".tmp").c_str(), calibration_path.c_str()) != 0) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "Could not save calibration to %s", calibration_path.c_str());
            return;
        }
        SoapySDR_logf(SOAPY_SDR_INFO, "Calibration saved to %s", calibration_path.c_str());
    }

    // Calibrate TX DC offset, TX IQ balance and RX IQ balance at current
    // frequencies and TX gains using the RF loopback.
    // Streams must not be active. Caller must hold ALSA stream mutexes.
    void run_calibration(void)
    {
        std::scoped_lock lock(reg_mutex);

        const double tx_frequency = getFrequency(SOAPY_SDR_TX, 0);
        const double rx_frequency = getFrequency(SOAPY_SDR_RX, 0);
        const std::string saved_pa_mode = pa_mode;
        uint8_t saved_regs[N_INIT_REGISTERS];
        memcpy(saved_regs, regs, sizeof(saved_regs));
        // ALSA devices must be configured even if application
        // has not set up both streams.
        if (!alsa_rx.setup_done)
            alsa_rx.configure(0);
        if (!alsa_tx.setup_done)
            alsa_tx.configure(0);

        SoapySDR_logf(SOAPY_SDR_INFO, "Starting calibration. Test tone is transmitted at %.3f MHz.",
            tx_frequency * 1e-6);

        struct cal_state st;
        st.rx_count = 0;
        st.tx_count = 0;
        st.valid_from = 0;
        st.lo_offset = 0.0;
        st.extra_settle = 0;
        {
            std::scoped_lock corr_lock(corr_mutex);
            st.rx = rx_corr;
            st.tx = tx_corr;
        }
        st.rx.dc_removal = false;
        st.raw.resize(CAL_CHUNK * 2);
        st.tone.resize(CAL_CHUNK);

        bool success = false;
        // RX IQ balance at TX and RX frequency
        std::complex<float> tx_dc, tx_iq, rx_iq_tx, rx_iq;
        try {
            // RF loopback on, PA driver on. On some boards
            // the loopback only works with external PA enabled.
            set_register_bits(0x10, 2, 2, 1);
            write_registers_to_chip(0x10, 1);
            set_register_bits(0x00, 3, 1, 1);
            write_registers_to_chip(0x00, 1);
            writeSetting("PA", "AUTO");

            alsa_rx.reset();
            alsa_tx.reset();
            // Prefill TX and start streams
            cal_pump(st);
            if (linked) {
                snd_pcm_start(alsa_rx.pcm);
            } else {
                snd_pcm_start(alsa_tx.pcm);
                snd_pcm_start(alsa_rx.pcm);
            }

            success = cal_pass(st, tx_frequency, false);
            tx_dc = st.tx.dc;
            tx_iq = st.tx.iq;
            rx_iq_tx = rx_iq = st.rx.iq;
            // Calibrate RX IQ balance separately if RX frequency is
            // far from TX frequency
            if (success && std::abs(rx_frequency - tx_frequency) > calibration_tolerance) {
                const double step = masterClock * (1.0 / (double)(1L<<20));
                const double offset = std::round(CAL_LO_OFFSET * sampleRate / step) * step;
                success = cal_pass(st, rx_frequency + offset, true);
                rx_iq = st.rx.iq;
            }
        } catch (std::exception &e) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "Calibration failed: %s", e.what());
            success = false;
        }

        // Restore streams and registers
        alsa_rx.reset();
        alsa_tx.reset();
        // Restore registers changed by calibration:
        // enables, frequencies, gains and loopback.
        memcpy(regs, saved_regs, sizeof(saved_regs));
        write_registers_to_chip(0x00, 7);
        write_registers_to_chip(0x08, 9);
        writeSetting("PA", saved_pa_mode);

        if (success) {
            SoapySDR_logf(SOAPY_SDR_INFO, "Calibration done: tx_dc=%f%+fj tx_iq=%f%+fj rx_iq=%f%+fj",
                tx_dc.real(), tx_dc.imag(), tx_iq.real(), tx_iq.imag(), rx_iq.real(), rx_iq.imag());
            // Store total correction so that it does not depend
            // on values set by the application.
            std::vector<struct calibration_point> points;
            {
                std::scoped_lock corr_lock(corr_mutex);
                points.push_back({
                    tx_frequency,
                    std::complex<double>(tx_dc) - user_tx_dc,
                    std::complex<double>(tx_iq) - user_tx_iq,
                    std::complex<double>(rx_iq_tx) - user_rx_iq,
                    getGain(SOAPY_SDR_TX, 0, "DAC"), getGain(SOAPY_SDR_TX, 0, "MIXER"),
                    false,
                });
                if (std::abs(rx_frequency - tx_frequency) > calibration_tolerance) {
                    struct calibration_point rx_point = points[0];
                    rx_point.frequency = rx_frequency;
                    rx_point.rx_iq = std::complex<double>(rx_iq) - user_rx_iq;
                    rx_point.rx_only = true;
                    points.push_back(rx_point);
                }
            }
            add_calibration_points(points);
        }
        apply_calibration(SOAPY_SDR_RX);
        apply_calibration(SOAPY_SDR_TX);
    }

public:

/***********************************************************************
 * Other settings
 **********************************************************************/

    SoapySDR::ArgInfoList getSettingInfo(void) const
    {
        SoapySDR::ArgInfoList infos;
        {
            SoapySDR::ArgInfo info;
            info.key = "PA";
            info.name = "PA control";
            info.description = "ON: always on, OFF: always off, AUTO: controlled by TX stream";
            info.type = SoapySDR::ArgInfo::STRING;
            info.value = "AUTO";
            info.options = {"AUTO", "ON", "OFF"};
            infos.push_back(info);
        }
        {
            SoapySDR::ArgInfo info;
            info.key = "CALIBRATION_FILE";
            info.name = "Calibration file";
            info.description = "Path of DC offset and IQ balance calibration table. Empty or none disables it.";
            info.type = SoapySDR::ArgInfo::STRING;
            info.value = default_calibration_path();
            infos.push_back(info);
        }
        {
            SoapySDR::ArgInfo info;
            info.key = "RX_DC_CUTOFF";
            info.name = "RX DC removal cutoff";
            info.description = "Cutoff frequency of RX DC removal filter";
            info.units = "Hz";
            info.type = SoapySDR::ArgInfo::FLOAT;
            info.value = "10";
            info.range = SoapySDR::Range(0.1, 1000.0);
            infos.push_back(info);
        }
        return infos;
    }

    std::string readSetting(const std::string & key) const
    {
        if (key == "CALIBRATION_FILE") {
            std::scoped_lock lock(reg_mutex);
            return calibration_path;
        }
        if (key == "RX_DC_CUTOFF") {
            std::scoped_lock lock(corr_mutex);
            return std::to_string(rx_dc_cutoff);
        }
        return "";
    }

    void writeSetting(
        const std::string & key,
        const std::string & value
    )
    {
        // PA control modes
        if (key == "PA") {
            if (value == "ON" || value == "OFF" || value == "AUTO")
                pa_mode = value;
            if (value == "ON") {
                // PA always on
                gpio_tx.set_value(1);
                gpio_rx.set_value(0);
            } else if (value == "OFF") {
                // PA always off
                gpio_tx.set_value(0);
                gpio_rx.set_value(1);
            } else if (value == "AUTO") {
                // PA on/off controlled by TX stream (default)
                gpio_tx.set_value(1);
                gpio_rx.set_value(1);
            }
        } else if (key == "CALIBRATION_FILE") {
            load_calibration(value);
        } else if (key == "CALIBRATE") {
            std::scoped_lock lock(alsa_rx.mutex, alsa_tx.mutex);
            if (alsa_rx.activated || alsa_tx.activated)
                throw std::runtime_error("Calibration is only possible when streams are not active");
            run_calibration();
        } else if (key == "RX_DC_CUTOFF") {
            {
                std::scoped_lock lock(corr_mutex);
                rx_dc_cutoff = std::min(std::max(std::stod(value), 0.1), 1000.0);
            }
            update_rx_dc_alpha();
        }
    }

/***********************************************************************
 * Low level interfaces
 **********************************************************************/

    std::vector<unsigned> readRegisters(
        const std::string & name,
        const unsigned addr,
        const size_t length
    ) const
    {
        (void)name; // Ignore name since there's only one register bank
        std::scoped_lock lock(reg_mutex);

        size_t transfer_len = length + 1;
        // Buffer for SPI transfer
        std::vector<uint8_t> buf(transfer_len, 0);

        buf[0] = addr;
        for (size_t i = 1; i < transfer_len; i++)
            buf[i] = 0;

        spi.transfer(buf, buf);

        std::vector<unsigned> result(length, 0);
        for (size_t i = 0; i < length; i++)
            result[i] = buf[i+1];

        return result;
    }

    unsigned readRegister(
        const std::string & name,
        const unsigned addr
    ) const
    {
        auto r = readRegisters(name, addr, 1);
        return r.at(0);
    }

    void writeRegisters(
        const std::string & name,
        const unsigned addr,
        const std::vector<unsigned> & value
    )
    {
        (void)name; // Ignore name since there's only one register bank
        std::scoped_lock lock(reg_mutex);

        for (size_t i = 0; i < value.size(); i++)
            set_register_bits(addr + i, 0, 8, value[i]);

        write_registers_to_chip(addr, value.size());
    }

    void writeRegister(
        const std::string & name,
        const unsigned addr,
        const unsigned value
    )
    {
        (void)name;
        std::scoped_lock lock(reg_mutex);
        set_register_bits(addr, 0, 8, value);
        write_registers_to_chip(addr, 1);
    }

/***********************************************************************
 * Other hardware and driver information
 **********************************************************************/

    std::string getDriverKey(void) const
    {
        return "sx";
    }

    std::string getHardwareKey(void) const
    {
        return "sx";
    }

    SoapySDR::Kwargs getHardwareInfo(void) const
    {
        SoapySDR::Kwargs args;
        args["soapysx_tag"] = SoapySX_tag;
        args["soapysx_commit"] = SoapySX_commit;
        if (hat_info.read_success) {
            args["hardware_version"] = std::to_string(hat_info.product_ver >> 8)
                               + "." + std::to_string(hat_info.product_ver & 0xFF);
        } else {
            args["hardware_version"] = "unknown";
        }
        return args;
    }

    size_t getNumChannels(const int direction) const
    {
        (void)direction; // Same for both directions
        return 1;
    }

    std::string getNativeStreamFormat(
        const int direction,
        const size_t channel,
        double &fullScale
    ) const
    {
        (void)direction; (void)channel;
        fullScale = 1.0;
        // This is not really the "native" format
        // but the only one currently supported.
        return "CF32";
    }

    std::vector<std::string> getStreamFormats(const int direction, const size_t channel) const
    {
       (void)direction; (void)channel;
        std::vector<std::string> streamFormats;
        streamFormats.push_back("CF32");
        return streamFormats;
    }

    bool hasHardwareTime(const std::string &what) const
    {
        if (what == "")
            return true;
        return false;
    }
};

/***********************************************************************
 * Find available devices
 **********************************************************************/
static SoapySDR::KwargsList findDevice(const SoapySDR::Kwargs &args)
{
    (void)args;
    SoapySDR::KwargsList devices;

    // TODO: check whether a device is actually found

    SoapySDR::Kwargs device;
    device["label"] = "sx";
    device["driver"] = "sx";
    devices.push_back(device);

    return devices;
}

/***********************************************************************
 * Make device instance
 **********************************************************************/
static SoapySDR::Device *makeDevice(const SoapySDR::Kwargs &args)
{
    SoapySDR::logf(SOAPY_SDR_INFO, "SoapySX version %s %s", SoapySX_tag, SoapySX_commit);
    return new SoapySX(args, read_hat_info());
}

/***********************************************************************
 * Registration
 **********************************************************************/
static SoapySDR::Registry registerDevice("sx", &findDevice, &makeDevice, SOAPY_SDR_ABI_VERSION);
