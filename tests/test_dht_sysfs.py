import pytest

import Weather_Station as ws


def make_iio_device(devices_dir, index, name, temperature=None, humidity=None):
    device = devices_dir / f"iio:device{index}"
    device.mkdir(parents=True)
    (device / "name").write_text(f"{name}\n")
    if temperature is not None:
        (device / "in_temp_input").write_text(f"{temperature}\n")
    if humidity is not None:
        (device / "in_humidityrelative_input").write_text(f"{humidity}\n")
    return device


def test_read_dht_sysfs_converts_milli_units(tmp_path):
    device = make_iio_device(tmp_path, 0, "dht11@13", temperature=21500, humidity=60300)

    assert ws.read_dht_sysfs(device) == (21.5, 60.3)


def test_read_dht_sysfs_handles_negative_temperature(tmp_path):
    device = make_iio_device(tmp_path, 0, "dht11@13", temperature=-3200, humidity=91000)

    assert ws.read_dht_sysfs(device) == (-3.2, 91.0)


def test_read_dht_sysfs_raises_oserror_when_the_driver_read_fails(tmp_path):
    # The driver answers a failed read with EIO/ETIMEDOUT; a missing file is the closest
    # off-hardware stand-in and must surface as OSError, not a value.
    device = make_iio_device(tmp_path, 0, "dht11@13", temperature=21500)

    with pytest.raises(OSError):
        ws.read_dht_sysfs(device)


def test_find_dht_iio_device_picks_the_dht11_driver(tmp_path):
    make_iio_device(tmp_path, 0, "mcp3008")
    dht = make_iio_device(tmp_path, 1, "dht11@13")

    assert ws.find_dht_iio_device(tmp_path) == dht


def test_find_dht_iio_device_raises_when_the_overlay_is_missing(tmp_path):
    make_iio_device(tmp_path, 0, "mcp3008")

    with pytest.raises(FileNotFoundError, match="dtoverlay=dht11"):
        ws.find_dht_iio_device(tmp_path)


def test_read_dht_finds_the_device_and_reads_it(tmp_path):
    make_iio_device(tmp_path, 3, "dht11", temperature=18250, humidity=47000)

    assert ws.read_dht(tmp_path) == (18.25, 47.0)


def test_userspace_dht_driver_is_gone():
    assert not hasattr(ws, "DHT_PIN_NAME")
    assert "adafruit_dht" not in ws.load_hardware.__code__.co_names
