# pi-weather-station-exporter

## Install

Python 3.11 or newer, on the Pi, as the unprivileged user that will run the exporter.

1. Enable I²C: `sudo raspi-config` → Interface Options → I2C (then reboot).
2. Install the build prerequisites (some dependencies, such as `sysv_ipc` and, on Python
   3.13+, `lgpio`, may be built from source):
   `sudo apt install python3-venv python3-dev build-essential`.
3. On Python 3.13 and later, build `lgpio` into the virtualenv first (see
   [GPIO library](#gpio-library-rpi-lgpio) below): `python3 -m venv .venv`, then run that
   recipe with `.venv/bin/pip` and `PATH=$PWD/.venv/bin:$PATH` (so the `swig` it installs
   is found).
4. `make install`. It creates `.venv/` in the checkout and installs the fully pinned,
   hash-checked set from `requirements.txt` into it (`pip install --require-hashes`).

> [!WARNING]
> Never `sudo pip install` and never `--break-system-packages`. Both run every package's
> install-time code as root on a networked device with GPIO and I²C access; the venv
> needs no root at all.

### Run and smoke check

```sh
.venv/bin/python Weather_Station.py
curl -s localhost:8705/metrics | grep -v '^#'
```

### Configuration

| Variable | Default | Meaning |
|---|---|---|
| `WEATHER_BIND` | `0.0.0.0` | Address the `/metrics` server listens on (for example `127.0.0.1` or the Pi's LAN address). |
| `WEATHER_PORT` | `8705` | Port for `/metrics`, 0–65535 (`0` picks a free port). An invalid value stops start-up. |

Unset or empty means the default, so an existing install keeps listening on
`0.0.0.0:8705`. Under systemd, set them with `Environment=`.

### Updating dependencies

The top-level pins live in `requirements.in`. `requirements.txt` is generated from it: a
universal lock (every platform and Python 3.11+) with a SHA-256 hash for every package,
including the transitive ones. To change a version, edit `requirements.in`, run
`make lock` (needs [`uv`](https://docs.astral.sh/uv/)), and commit both files. Do not
edit `requirements.txt` by hand.

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
  pip install --require-hashes -r requirements.txt
  ```

  The `CFLAGS` let GCC 14 and later compile its C sources (they are written for older C
  defaults). On Python 3.12 and older a prebuilt wheel is used and none of this is needed.

### Upgrading an existing install from RPi.GPIO

`RPi.GPIO` and `rpi-lgpio` both install the `RPi` package and conflict, and pip does not
remove the old one on its own. In the exporter's existing virtualenv:

```sh
pip uninstall -y RPi.GPIO
pip install --require-hashes -r requirements.txt
```

## DHT22

The DHT22 is sited apart from the BME280 (so it measures somewhere the BME280 cannot) and
is read through the kernel's `dht11` IIO driver, which also handles the DHT22. The kernel
does the bit timing; the exporter only reads two sysfs files, so there is no userspace
driver or helper process.

Enable the driver on the Pi for the sensor's data pin (BCM 19), then reboot:

```sh
echo 'dtoverlay=dht11,gpiopin=19' | sudo tee -a /boot/firmware/config.txt
sudo reboot
```

After the reboot, check the device is there and reads:

```sh
grep -H . /sys/bus/iio/devices/*/name                      # one of them is dht11 or dht11@13
cat /sys/bus/iio/devices/iio:device*/in_temp_input          # milli-degrees Celsius
```

The exporter finds the device by its driver name, so the `iio:deviceN` number does not
matter. A failed read (the driver returns `EIO` or `ETIMEDOUT`, which the DHT22 does now
and then) is logged and retried on the next poll.
