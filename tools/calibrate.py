#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Calibrate TX LO leakage, TX IQ balance and RX IQ balance of SX1255.

SX1255 has no registers for DC offset or IQ balance correction,
so SoapySX applies the corrections digitally to the samples.
This script finds the correction coefficients using the internal
RF loopback of SX1255 (RX antenna "LB") and writes them to a calibration
table which SoapySX loads when the device is opened.

Frequency plan (fs = sample rate):
  TX LO is at the calibrated frequency F and TX sends a tone at +3/32 fs.
  RX LO is tuned to F - 1/32 fs, made possible by separate RX and TX PLLs.
  This places every unwanted product at a different frequency in RX:
      +4/32 fs  wanted tone
      +1/32 fs  TX LO leakage           (nulled with TX DC offset)
      -2/32 fs  TX image                (nulled with TX IQ balance)
      -4/32 fs  RX image of the tone    (nulled with RX IQ balance)
       0        RX DC offset            (removed by a filter in SoapySX)

Each correction is found by a Newton search: the complex amplitude of the
unwanted product, relative to the wanted tone, is measured with the current
correction and with two small perturbations of it, giving a 2x2 Jacobian.
This does not need any knowledge of delays or phase shifts in the loopback.

On some boards the loopback only works with the external PA enabled
(--pa auto, the default), so the tone is also transmitted from the
antenna connector. Connect a dummy load or an attenuator while calibrating.

Corrections depend on TX gains, so calibrate with the TX gains you use.
"""

import argparse
import datetime
import os
import sys

import numpy as np

# Frequencies in units of sample rate
LO_OFFSET = 1.0 / 32.0
TONE = 3.0 / 32.0


def default_calibration_path():
    xdg = os.environ.get('XDG_CONFIG_HOME')
    if xdg:
        return os.path.join(xdg, 'SoapySX', 'calibration.txt')
    return os.path.join(os.path.expanduser('~'), '.config', 'SoapySX', 'calibration.txt')


def db(x):
    return 20.0 * np.log10(max(abs(x), 1e-15))


class Corrections:
    """Correction coefficients, using the same convention as SoapySX:
    y = x + iq * conj(x) + dc"""
    def __init__(self, rx_iq=0j, tx_iq=0j, tx_dc=0j):
        self.rx_iq = complex(rx_iq)
        self.tx_iq = complex(tx_iq)
        self.tx_dc = complex(tx_dc)

    def copy(self):
        return Corrections(self.rx_iq, self.tx_iq, self.tx_dc)


def apply_correction(x, iq, dc=0j):
    """Same correction as done in SoapySX."""
    return x + iq * np.conj(x) + dc


def correlate(x, n, freq):
    """Complex amplitude of a tone at normalized frequency freq in signal x,
    whose sample indices are n. Phase is referenced to n = 0."""
    window = np.hanning(len(x))
    phase = 2.0 * np.pi * np.mod(freq * n, 1.0)
    return np.sum(x * window * np.exp(-1j * phase)) / np.sum(window)


# Frequencies, in units of sample rate, where no signal is expected.
# Used to estimate the noise level of measurements.
NOISE_FREQUENCIES = (5.5/32, -5.5/32, 7.5/32, -7.5/32, 9.5/32, -9.5/32)


def analyze(x, n, lo_offset, tone):
    """Measure the wanted tone and unwanted products in received signal."""
    noise = [correlate(x, n, f) for f in NOISE_FREQUENCIES]
    return {
        'tone':     correlate(x, n, lo_offset + tone),
        'tx_lo':    correlate(x, n, lo_offset),
        'tx_image': correlate(x, n, lo_offset - tone),
        'rx_image': correlate(x, n, -(lo_offset + tone)),
        'noise':    float(np.sqrt(np.mean(np.abs(noise) ** 2))),
        'peak':     float(np.max(np.abs(x))),
    }


def spectrum_peaks(x, count=8):
    """Find strongest peaks in spectrum of x.
    Returns a list of (frequency in units of sample rate, level in dBFS)."""
    window = np.hanning(len(x))
    spectrum = np.abs(np.fft.fftshift(np.fft.fft(x * window))) / np.sum(window)
    freqs = np.fft.fftshift(np.fft.fftfreq(len(x)))
    peaks = []
    s = spectrum.copy()
    for _ in range(count):
        i = int(np.argmax(s))
        peaks.append((freqs[i], db(spectrum[i])))
        # Exclude neighbourhood of the peak
        s[max(i - 8, 0):i + 9] = 0.0
    return peaks


class Loopback:
    """Transmit a tone and receive it through the RF loopback of SX1255."""
    def __init__(self, args):
        import SoapySDR
        self.SoapySDR = SoapySDR
        self.args = args
        # Some versions of SoapySDR Python bindings convert a dict passed
        # to SoapySDR.Device into a list of its keys, which fails with
        # "no match". A list of dicts is not ambiguous, so open the device
        # with the parallel version of make.
        self.dev = SoapySDR.Device.make([{'driver': 'sx'}])[0]
        dev = self.dev
        # Do not apply any existing calibration while calibrating.
        # All corrections are set explicitly by _write_corrections.
        dev.writeSetting('CALIBRATION_FILE', 'none')

        rates = dev.listSampleRates(SoapySDR.SOAPY_SDR_RX, 0)
        self.fs = min(rates, key=lambda r: abs(r - args.sample_rate))
        dev.setSampleRate(SoapySDR.SOAPY_SDR_RX, 0, self.fs)
        dev.setSampleRate(SoapySDR.SOAPY_SDR_TX, 0, self.fs)

        dev.setAntenna(SoapySDR.SOAPY_SDR_RX, 0, 'LB')
        # PA driver of SX1255 must be enabled for the loopback to work.
        dev.setAntenna(SoapySDR.SOAPY_SDR_TX, 0, 'TX')
        dev.writeSetting('PA', args.pa.upper())
        self.amplitude = args.amplitude
        dev.setDCOffsetMode(SoapySDR.SOAPY_SDR_RX, 0, False)

        dev.setGain(SoapySDR.SOAPY_SDR_TX, 0, 'DAC', args.tx_dac)
        dev.setGain(SoapySDR.SOAPY_SDR_TX, 0, 'MIXER', args.tx_mixer)
        self.rx_pga = 30.0 if args.rx_pga is None else args.rx_pga
        dev.setGain(SoapySDR.SOAPY_SDR_RX, 0, 'PGA', self.rx_pga)

        self.chunk = args.chunk
        self.rx = dev.setupStream(SoapySDR.SOAPY_SDR_RX, SoapySDR.SOAPY_SDR_CF32, [0], {'period': str(self.chunk)})
        self.tx = dev.setupStream(SoapySDR.SOAPY_SDR_TX, SoapySDR.SOAPY_SDR_CF32, [0], {'period': str(self.chunk)})
        self.rx_buf = np.zeros(self.chunk, dtype=np.complex64)
        # Sample index used as the time reference for phase of TX tone
        self.n_ref = None
        # Sample index where the next TX chunk will be written
        self.next_tx = 0
        self.corr = Corrections()
        self._write_corrections()
        # RX sample index from which on the current settings are in effect
        self.valid_from = 0
        dev.activateStream(self.rx)
        dev.activateStream(self.tx)

    def close(self):
        SoapySDR = self.SoapySDR
        self.dev.deactivateStream(self.rx)
        self.dev.deactivateStream(self.tx)
        self.dev.closeStream(self.rx)
        self.dev.closeStream(self.tx)
        self.dev.setAntenna(SoapySDR.SOAPY_SDR_RX, 0, 'RX')
        self.dev.writeSetting('PA', 'AUTO')
        self.dev.close()

    def _write_corrections(self):
        SoapySDR = self.SoapySDR
        self.dev.setIQBalance(SoapySDR.SOAPY_SDR_RX, 0, self.corr.rx_iq)
        self.dev.setIQBalance(SoapySDR.SOAPY_SDR_TX, 0, self.corr.tx_iq)
        self.dev.setDCOffset(SoapySDR.SOAPY_SDR_TX, 0, self.corr.tx_dc)

    def _now(self):
        return self.SoapySDR.timeNsToTicks(self.dev.getHardwareTime(), self.fs)

    def set_frequency(self, frequency):
        SoapySDR = self.SoapySDR
        self.dev.setFrequency(SoapySDR.SOAPY_SDR_TX, 0, frequency)
        self.dev.setFrequency(SoapySDR.SOAPY_SDR_RX, 0, frequency - LO_OFFSET * self.fs)
        f_tx = self.dev.getFrequency(SoapySDR.SOAPY_SDR_TX, 0)
        f_rx = self.dev.getFrequency(SoapySDR.SOAPY_SDR_RX, 0)
        # Exact LO offset after quantization of synthesizer frequencies
        self.lo_offset = (f_tx - f_rx) / self.fs
        # Wait for PLLs to lock
        self.valid_from = self._now() + int(self.args.pll_settle * self.fs)

    def set_corrections(self, corr):
        self.corr = corr.copy()
        # The next TX chunk will be written with the new corrections,
        # so they will be in effect for samples after it.
        self._write_corrections()
        self.valid_from = max(self.valid_from, self.next_tx + self.args.settle,
            self._now() + self.args.latency + self.args.settle)

    def _pump(self):
        """Read one chunk of RX samples and write one chunk of TX samples."""
        SoapySDR = self.SoapySDR
        ret = self.dev.readStream(self.rx, [self.rx_buf], self.chunk, timeoutUs=1000000)
        if ret.ret <= 0:
            raise RuntimeError('RX error: %s' % ret)
        n_rx = SoapySDR.timeNsToTicks(ret.timeNs, self.fs)
        if self.n_ref is None:
            self.n_ref = n_rx
        # Keep TX stream continuous and a fixed latency ahead of hardware time.
        # RX samples may be old if they were not read for a while,
        # so TX timing is based on hardware time instead of RX timestamps.
        now = self._now()
        if self.next_tx < now + self.chunk:
            # TX has fallen behind (e.g. at start or after a pause),
            # restart it in the future.
            self.next_tx = now + self.args.latency
        if self.next_tx < now + 2 * self.args.latency:
            n_tx = self.next_tx
            self.next_tx = n_tx + self.chunk
            n = np.arange(n_tx, n_tx + self.chunk) - self.n_ref
            tx = (self.amplitude * np.exp(2j * np.pi * np.mod(TONE * n, 1.0))).astype(np.complex64)
            self.dev.writeStream(self.tx, [tx], len(tx), SoapySDR.SOAPY_SDR_HAS_TIME,
                SoapySDR.ticksToTimeNs(n_tx, self.fs))
        return n_rx, self.rx_buf[:ret.ret].copy()

    def capture(self):
        """Capture samples received with the current settings."""
        length = self.args.length
        x = np.zeros(length, dtype=np.complex64)
        filled = 0
        n_start = None
        while filled < length:
            n, buf = self._pump()
            valid_from = self.valid_from
            # Skip samples before settings are valid
            if n + len(buf) <= valid_from:
                continue
            if n < valid_from:
                buf = buf[valid_from - n:]
                n = valid_from
            # Restart if samples were lost
            if n_start is not None and n != n_start + filled:
                filled = 0
                n_start = None
            if n_start is None:
                n_start = n
            take = min(len(buf), length - filled)
            x[filled:filled+take] = buf[:take]
            filled += take
        return x, np.arange(n_start, n_start + length) - self.n_ref

    def measure(self):
        x, n = self.capture()
        return analyze(x, n, self.lo_offset, TONE)

    def configure(self, antenna=None, pa=None, amplitude=None):
        """Change settings for diagnostics."""
        SoapySDR = self.SoapySDR
        if antenna is not None:
            self.dev.setAntenna(SoapySDR.SOAPY_SDR_RX, 0, antenna)
        if pa is not None:
            self.dev.writeSetting('PA', pa)
        if amplitude is not None:
            self.amplitude = amplitude
        self.valid_from = max(self._now() + self.args.latency, self.next_tx) + self.args.settle

    def adjust_rx_gain(self):
        """Find a PGA gain giving a reasonable signal level."""
        if self.args.rx_pga is not None:
            return
        SoapySDR = self.SoapySDR
        while True:
            m = self.measure()
            if m['peak'] < 0.3 or self.rx_pga <= 0.0:
                break
            self.rx_pga = max(self.rx_pga - 4.0, 0.0)
            self.dev.setGain(SoapySDR.SOAPY_SDR_RX, 0, 'PGA', self.rx_pga)
            self.valid_from = self._now() + self.args.settle
        print('  RX PGA gain %.0f dB, peak %.3f' % (self.rx_pga, m['peak']))


class Simulation:
    """Simulated SX1255 loopback with random impairments,
    for testing the calibration algorithm without hardware."""
    def __init__(self, args):
        self.args = args
        self.fs = args.sample_rate
        self.rng = np.random.default_rng(args.seed)
        self.corr = Corrections()
        self.n = 0
        self.rx_pga = 0.0

    def close(self):
        pass

    def _rand_iq(self, gain_db, phase_deg):
        """Coefficients K1, K2 of a mixer with IQ imbalance:
        y = K1 x + K2 conj(x)"""
        g = 10.0 ** (self.rng.uniform(-gain_db, gain_db) / 20.0)
        p = np.radians(self.rng.uniform(-phase_deg, phase_deg))
        # I = Re(x), Q = g * (Im(x) cos p + Re(x) sin p)
        k1 = 0.5 * (1.0 + g * np.exp(-1j * p))
        k2 = 0.5 * (1.0 - g * np.exp(1j * p))
        return k1, k2

    def set_frequency(self, frequency):
        self.lo_offset = LO_OFFSET
        self.tx_k = self._rand_iq(1.0, 3.0)
        self.rx_k = self._rand_iq(1.0, 3.0)
        self.tx_leak = 0.2 * np.exp(2j * np.pi * self.rng.uniform())
        self.rx_dc = 0.05 * np.exp(2j * np.pi * self.rng.uniform())
        self.phase = np.exp(2j * np.pi * self.rng.uniform())
        self.delay = int(self.rng.integers(10, 1000))

    def set_corrections(self, corr):
        self.corr = corr.copy()

    def measure(self):
        length = self.args.length
        self.n += length + int(self.rng.integers(0, 10000))
        n = np.arange(self.n, self.n + length)
        # TX
        x = self.args.amplitude * np.exp(2j * np.pi * np.mod(TONE * (n - self.delay), 1.0))
        x = apply_correction(x, self.corr.tx_iq, self.corr.tx_dc)
        x = np.clip(x.real, -1, 1) + 1j * np.clip(x.imag, -1, 1)
        x = self.tx_k[0] * x + self.tx_k[1] * np.conj(x) + self.tx_leak
        # Loopback and LO offset
        x = x * self.phase * np.exp(2j * np.pi * np.mod(self.lo_offset * n, 1.0)) * 0.3
        # RX
        x = self.rx_k[0] * x + self.rx_k[1] * np.conj(x) + self.rx_dc
        x = x + 1e-4 * (self.rng.standard_normal(length) + 1j * self.rng.standard_normal(length))
        x = apply_correction(x, self.corr.rx_iq)
        return analyze(x, n, self.lo_offset, TONE)

    def adjust_rx_gain(self):
        pass


def null_search(lb, corr, field, measure_fn, step, iterations, target_db):
    """Find value of corrections field nulling complex measurement measure_fn.

    Jacobian of the measurement with respect to real and imaginary part
    of the correction is estimated from perturbations, so the search
    works regardless of unknown phase shifts."""
    def evaluate(value):
        c = corr.copy()
        setattr(c, field, value)
        lb.set_corrections(c)
        return measure_fn(lb.measure())

    p = getattr(corr, field)
    f0 = evaluate(p)
    first = f0
    best = (abs(f0), p)
    for _ in range(iterations):
        if db(f0) < target_db:
            break
        fx = evaluate(p + step)
        fy = evaluate(p + 1j * step)
        jac = np.array([
            [(fx - f0).real, (fy - f0).real],
            [(fx - f0).imag, (fy - f0).imag],
        ]) / step
        try:
            dp = np.linalg.solve(jac, [-f0.real, -f0.imag])
        except np.linalg.LinAlgError:
            break
        p = p + complex(dp[0], dp[1])
        f0 = evaluate(p)
        if abs(f0) < best[0]:
            best = (abs(f0), p)
        # Smaller perturbations near the optimum
        step = min(step, max(abs(complex(dp[0], dp[1])), step / 10.0))
    setattr(corr, field, best[1])
    lb.set_corrections(corr)
    return first, best[0]


def calibrate_frequency(lb, corr, args):
    lb.adjust_rx_gain()
    results = {}

    # Make sure the tone is actually received through the loopback.
    # Otherwise the search would only follow noise.
    m = lb.measure()
    snr = db(m['tone']) - db(m['noise'])
    print('  Tone %.1f dBFS, noise %.1f dBFS, SNR %.1f dB' % (db(m['tone']), db(m['noise']), snr))
    if snr < args.min_snr:
        raise RuntimeError('Tone is not received through the loopback (SNR %.1f dB < %.1f dB). '
            'Run with --diagnose to investigate.' % (snr, args.min_snr))

    # RX IQ balance: null RX image of the tone relative to the tone.
    # The ratio is normalized with the conjugate of the tone so that
    # it does not depend on phase of the received signal.
    results['rx_image'] = null_search(lb, corr, 'rx_iq',
        lambda m: m['rx_image'] / np.conj(m['tone']),
        args.step, args.iterations, args.target)

    # TX IQ balance: null TX image relative to the tone.
    results['tx_image'] = null_search(lb, corr, 'tx_iq',
        lambda m: m['tx_image'] / m['tone'],
        args.step, args.iterations, args.target)

    # TX DC offset: null TX LO leakage relative to the tone.
    results['tx_lo'] = null_search(lb, corr, 'tx_dc',
        lambda m: m['tx_lo'] / m['tone'],
        args.step, args.iterations, args.target)

    # Final check of all products with final corrections.
    lb.set_corrections(corr)
    m = lb.measure()
    final = {
        'rx_image': db(m['rx_image'] / m['tone']),
        'tx_image': db(m['tx_image'] / m['tone']),
        'tx_lo':    db(m['tx_lo'] / m['tone']),
    }
    return results, final


def diagnose(lb, frequency, args):
    """Show strongest spectral components received with different settings,
    to check whether the TX tone gets through the loopback."""
    lb.set_frequency(frequency)
    print('Diagnostics at %.3f MHz, RX PGA %.0f dB' % (frequency * 1e-6, lb.rx_pga))
    print('Frequencies are relative to RX LO, in units of fs/32. Expected:')
    print('  %+.2f tone, %+.2f TX LO leakage, %+.2f TX image, %+.2f RX image, 0 RX DC' % (
        32 * (lb.lo_offset + TONE), 32 * lb.lo_offset, 32 * (lb.lo_offset - TONE), -32 * (lb.lo_offset + TONE)))
    tests = (
        ('RF loopback, PA OFF, tone on',  'LB', 'OFF',  args.amplitude),
        ('RF loopback, PA OFF, tone off', 'LB', 'OFF',  0.0),
        ('RF loopback, PA AUTO, tone on', 'LB', 'AUTO', args.amplitude),
        ('RX antenna, PA OFF, tone on',   'RX', 'OFF',  args.amplitude),
    )
    for name, antenna, pa, amplitude in tests:
        lb.configure(antenna=antenna, pa=pa, amplitude=amplitude)
        x, n = lb.capture()
        m = analyze(x, n, lb.lo_offset, TONE)
        print('%s:' % name)
        print('  tone %.1f dBFS, noise %.1f dBFS, peak sample %.4f' % (db(m['tone']), db(m['noise']), m['peak']))
        print('  strongest components: ' + ', '.join(
            '%+.2f: %.1f dBFS' % (32 * f, level) for f, level in spectrum_peaks(x)))
    lb.configure(antenna='LB', pa=args.pa.upper(), amplitude=args.amplitude)


def parse_frequencies(text):
    """Parse 'start:stop:step' or comma separated list of frequencies in MHz."""
    if ':' in text:
        start, stop, step = (float(v) for v in text.split(':'))
        count = int(np.floor((stop - start) / step + 1e-9)) + 1
        return [(start + step * i) * 1e6 for i in range(count)]
    return [float(v) * 1e6 for v in text.split(',')]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('-f', '--frequencies', default='420:450:1',
        help='Frequencies in MHz as start:stop:step or a comma separated list (default: %(default)s)')
    parser.add_argument('-o', '--output', default=default_calibration_path(),
        help='Calibration table file (default: %(default)s)')
    parser.add_argument('--sample-rate', type=float, default=250e3,
        help='Sample rate, nearest supported is used (default: %(default)s)')
    parser.add_argument('--tx-dac', type=float, default=6.0, help='TX DAC gain (default: %(default)s)')
    parser.add_argument('--tx-mixer', type=float, default=26.0, help='TX mixer gain (default: %(default)s)')
    parser.add_argument('--rx-pga', type=float, default=None, help='RX PGA gain (default: automatic)')
    parser.add_argument('--amplitude', type=float, default=0.25, help='TX tone amplitude (default: %(default)s)')
    parser.add_argument('--length', type=int, default=8192, help='Samples per measurement (default: %(default)s)')
    parser.add_argument('--chunk', type=int, default=1024, help='Stream chunk size (default: %(default)s)')
    parser.add_argument('--latency', type=int, default=4096, help='RX to TX latency in samples (default: %(default)s)')
    parser.add_argument('--settle', type=int, default=2048,
        help='Samples to skip after changing corrections (default: %(default)s)')
    parser.add_argument('--pll-settle', type=float, default=0.01,
        help='Time to wait after retuning in seconds (default: %(default)s)')
    parser.add_argument('--step', type=float, default=0.01, help='Perturbation for Jacobian estimation (default: %(default)s)')
    parser.add_argument('--iterations', type=int, default=4, help='Maximum Newton iterations (default: %(default)s)')
    parser.add_argument('--target', type=float, default=-70.0, help='Stop when below this level in dBc (default: %(default)s)')
    parser.add_argument('--pa', choices=('off', 'auto'), default='auto',
        help='External PA control during calibration. On some boards the loopback '
        'does not work with PA off. (default: %(default)s)')
    parser.add_argument('--min-snr', type=float, default=40.0,
        help='Minimum SNR of the received tone in dB (default: %(default)s)')
    parser.add_argument('--diagnose', action='store_true',
        help='Only show received spectrum with different settings at the first frequency')
    parser.add_argument('--simulate', action='store_true', help='Use a simulated device to test the algorithm')
    parser.add_argument('--seed', type=int, default=1, help='Random seed for simulation')
    args = parser.parse_args()

    frequencies = parse_frequencies(args.frequencies)
    if not args.simulate and args.pa == 'auto':
        print('WARNING: the test tone is transmitted through the external PA.')
        print('Make sure a dummy load or an attenuator is connected to the antenna connector.')
    lb = Simulation(args) if args.simulate else Loopback(args)

    if args.diagnose:
        if args.simulate:
            print('--diagnose needs real hardware')
            return 1
        try:
            diagnose(lb, frequencies[0], args)
        finally:
            lb.close()
        return 0

    if os.path.dirname(args.output):
        os.makedirs(os.path.dirname(args.output), exist_ok=True)

    print('Sample rate %.0f Hz' % lb.fs)
    corr = Corrections()
    worst = {'rx_image': -999.0, 'tx_image': -999.0, 'tx_lo': -999.0}
    # Write to a temporary file so that an existing table
    # is replaced only if calibration succeeds.
    tmp_output = args.output + '.tmp'
    try:
        with open(tmp_output, 'w') as out:
            out.write('# SoapySX calibration table written by tools/calibrate.py on %s\n'
                % datetime.datetime.now().isoformat(timespec='seconds'))
            out.write('# TX gains: DAC %.0f dB, MIXER %.0f dB. Sample rate %.0f Hz.%s\n'
                % (args.tx_dac, args.tx_mixer, lb.fs, ' SIMULATED' if args.simulate else ''))
            out.write('# Corrections are applied as y = x + iq * conj(x) + dc\n')
            out.write('# frequency_hz tx_dc_re tx_dc_im tx_iq_re tx_iq_im rx_iq_re rx_iq_im tx_dac tx_mixer\n')
            for frequency in frequencies:
                print('%.3f MHz' % (frequency * 1e-6))
                lb.set_frequency(frequency)
                # Start from corrections of the previous frequency
                results, final = calibrate_frequency(lb, corr, args)
                for key, name in (('rx_image', 'RX image'), ('tx_image', 'TX image'), ('tx_lo', 'TX LO leakage')):
                    print('  %-14s %7.1f dBc -> %7.1f dBc' % (name, db(results[key][0]), final[key]))
                    worst[key] = max(worst[key], final[key])
                if abs(corr.tx_dc) > 0.2:
                    print('  Note: TX DC correction is %.0f %% of full scale, reducing headroom '
                        'for the signal. Higher MIXER gain may reduce LO leakage relative to the signal.'
                        % (100 * abs(corr.tx_dc)))
                print('  tx_dc=%s tx_iq=%s rx_iq=%s' % (
                    np.round(corr.tx_dc, 5), np.round(corr.tx_iq, 5), np.round(corr.rx_iq, 5)))
                out.write('%.0f %.7f %.7f %.7f %.7f %.7f %.7f %.1f %.1f\n' % (frequency,
                    corr.tx_dc.real, corr.tx_dc.imag,
                    corr.tx_iq.real, corr.tx_iq.imag,
                    corr.rx_iq.real, corr.rx_iq.imag,
                    args.tx_dac, args.tx_mixer))
                out.flush()
        os.replace(tmp_output, args.output)
    except Exception as e:
        print('Calibration failed: %s' % e)
        print('Existing calibration table was not changed.')
        if os.path.exists(tmp_output):
            os.remove(tmp_output)
        return 1
    finally:
        lb.close()

    print('Worst residuals: RX image %.1f dBc, TX image %.1f dBc, TX LO leakage %.1f dBc'
        % (worst['rx_image'], worst['tx_image'], worst['tx_lo']))
    print('Calibration written to %s' % args.output)
    return 0


if __name__ == '__main__':
    sys.exit(main())
