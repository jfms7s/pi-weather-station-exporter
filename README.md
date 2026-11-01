# pi-weather-station-exporter

## Install

### GPIO library: rpi-lgpio

The anemometer is read through the `RPi.GPIO` API, provided by
[`rpi-lgpio`](https://rpi-lgpio.readthedocs.io/) (on top of `lgpio` and the
`/dev/gpiochip*` character device). The original `RPi.GPIO` package is not used: its
edge detection fails on Raspberry Pi OS Bookworm 6.6+ kernels and it does not support
the Pi 5.

- **Permissions:** the user running the exporter must be in the `gpio` group
  (`sudo usermod -aG gpio <user>`, then log in again) to open `/dev/gpiochip*`.
- **Writable working directory:** `lgpio` creates a `.lgd-nfy<N>` notification pipe in
  the process's current directory when it is imported. Start the exporter from a
  directory that user can write to (under systemd, set `WorkingDirectory=` to one, for
  example a `StateDirectory=`); the default `/` is not writable and the import fails.
- **Python 3.13 and later** (Raspberry Pi OS Trixie): `lgpio` 0.2.2.0 only ships wheels
  for CPython 3.9–3.12, so pip builds it from source. That needs a C compiler, the Python
  headers and SWIG **older than 4.3** (4.3 dropped the `PyInt_*`/`PyString_*` macros its
  bindings use, so the build fails with Trixie's `swig` package). Build it first, in the
  virtualenv, statically linked (`PYPI=1` is `lgpio`'s own switch; no system `liblgpio`
  needed):

  ```sh
  sudo apt install build-essential python3-dev
  pip install setuptools wheel 'swig<4.3'
  PYPI=1 CFLAGS="-std=gnu17 -Wno-error=incompatible-pointer-types" \
    pip install --no-build-isolation lgpio==0.2.2.0
  pip install -r requirements.txt
  ```

  The `CFLAGS` let GCC 14 and later compile its C sources (they are written for older C
  defaults). On Python 3.12 and older a prebuilt wheel is used and none of this is needed.

### Upgrading an existing install from RPi.GPIO

`RPi.GPIO` and `rpi-lgpio` both install the `RPi` package and conflict, and pip does not
remove the old one on its own. In the exporter's existing virtualenv:

```sh
pip uninstall -y RPi.GPIO
pip install -r requirements.txt
```
