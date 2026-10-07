## Setup on Raspberry Pi

Install dependencies:
```
sudo apt-get install --no-install-recommends git make g++ cmake libsoapysdr-dev libasound2-dev soapysdr-tools python3-soapysdr
```

Some prototype boards do not have the HAT identification EEPROM written.
If you have one of those, write it first by following
[EEPROM writing instructions](dts/README.md).

Compile and install SoapySDR module:
```
cd SoapySX
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig
```

Check that the module is found:
```
SoapySDRUtil --probe=driver=sx
```

## Features
SoapySX provides some support for timestamps which are used by some
applications to obtain a known timing relationship between transmitted and
received signals.
See the
[linear repeater example](example/linear_repeater.py)
for an example on using timestamps to obtain a constant, known latency
from received to transmitted signal.

## DC offset and IQ balance corrections
SX1255 has no registers for DC offset or IQ imbalance correction,
so SoapySX corrects the samples digitally:

* RX DC offset is removed by a high-pass filter.
  It is enabled by default and can be turned off with
  `setDCOffsetMode(SOAPY_SDR_RX, 0, False)`.
  Its cutoff frequency (default 10 Hz) can be changed with the setting
  `RX_DC_CUTOFF`.
* RX IQ balance, TX IQ balance and TX DC offset (cancelling LO leakage)
  can be set with `setIQBalance` and `setDCOffset`.
  Corrections are applied as `y = x + iq * conj(x) + dc`.
  Values set by an application are added to the values from the
  calibration table described below, so applications that set them
  to zero do not cancel the calibration.

The correction coefficients can be measured with the internal RF loopback
of SX1255 by running
```
python3 tools/calibrate.py -f 420:450:1
```
where `-f` gives the frequency range in MHz as start:stop:step.
The loopback only works with the external PA enabled on some boards,
so the test tone is also transmitted from the antenna connector.
Connect a dummy load or an attenuator while calibrating.
If calibration fails, `tools/calibrate.py --diagnose` shows what is
received with different settings.
TX DC offset depends on TX gains, so give the TX gains you use
with `--tx-dac` and `--tx-mixer`. See `tools/calibrate.py --help` for details.

The results are written to `~/.config/SoapySX/calibration.txt`.
SoapySX loads this file when the device is opened and sets the corrections
by interpolating the table every time the frequency is changed.
Another file can be given with the device argument `calibration=/path/to/file`
(for example `SoapySDR.Device.make([{'driver': 'sx', 'calibration': '/path/to/file'}])[0]`
in Python, since some versions of the bindings misinterpret a plain dict)
or the setting `CALIBRATION_FILE`, and `none` disables it.

When the device is opened, SoapySX logs either
`Loaded N calibration points from ...` or `No calibration table found in ...`.
Note that the default path depends on the user running the application,
so if it runs as root, the file is looked up in `/root/.config/SoapySX/`.
To see the corrections applied after each frequency change,
run the application with `SOAPY_SDR_LOG_LEVEL=DEBUG`.

### Built-in calibration
SoapySX can also run the same calibration by itself at the current
frequencies and TX gains, and add the result to the calibration table.

To enable automatic calibration, create `~/.config/SoapySX/soapysx.conf`
containing
```
auto_calibrate=1
```
or give the device argument `auto_calibrate=1`.
Then, when streams are activated, SoapySX checks whether the calibration
table has a point calibrated with the current TX gains (DAC and MIXER)
within 500 kHz of the current TX frequency, and a point within 500 kHz
of the current RX frequency. If not, it calibrates and appends the result
to the calibration file, so the next time the stored result is used.
The distance can be changed with `calibration_tolerance=<Hz>`
in the same file. Points written by older versions of `tools/calibrate.py`
without TX gains are used with any gains but do not prevent
automatic calibration.

Writing the setting `CALIBRATE` (any value) runs calibration immediately.
Streams must not be active.

Calibration takes a few seconds and the test tone is transmitted
through the external PA at the current TX frequency and gains,
so use it only with a dummy load or when transmitting the tone is
acceptable. Progress and results are logged. If calibration fails,
for example because the loopback signal is not received,
nothing is saved and the previous corrections are kept.

Note that `tools/calibrate.py` overwrites the whole calibration file.
