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

The correction coefficients can be measured with the internal RF loopback
of SX1255 by running
```
python3 tools/calibrate.py -f 420:450:1
```
where `-f` gives the frequency range in MHz as start:stop:step.
The loopback signal also leaks out of the antenna connector,
so connect a dummy load or an attenuator while calibrating.
TX DC offset depends on TX gains, so give the TX gains you use
with `--tx-dac` and `--tx-mixer`. See `tools/calibrate.py --help` for details.

The results are written to `~/.config/SoapySX/calibration.txt`.
SoapySX loads this file when the device is opened and sets the corrections
by interpolating the table every time the frequency is changed.
Another file can be given with the device argument `calibration=/path/to/file`
(for example `SoapySDR.Device('driver=sx,calibration=/path/to/file')` in Python)
or the setting `CALIBRATION_FILE`, and `none` disables it.
