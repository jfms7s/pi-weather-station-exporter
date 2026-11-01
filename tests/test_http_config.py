import pytest

import Weather_Station as ws


def test_http_config_defaults_to_all_interfaces_on_8705():
    assert ws.load_http_config({}) == ("0.0.0.0", 8705)


def test_http_config_empty_values_mean_defaults():
    assert ws.load_http_config({"WEATHER_BIND": "", "WEATHER_PORT": " "}) == ("0.0.0.0", 8705)


def test_http_config_reads_bind_and_port():
    environ = {"WEATHER_BIND": "192.168.1.50", "WEATHER_PORT": "9100"}
    assert ws.load_http_config(environ) == ("192.168.1.50", 9100)


def test_http_config_port_zero_is_ephemeral():
    assert ws.load_http_config({"WEATHER_PORT": "0"}) == ("0.0.0.0", 0)


@pytest.mark.parametrize("value", ["abc", "-1", "70000", "1.5"])
def test_http_config_rejects_invalid_port(value):
    with pytest.raises(SystemExit) as excinfo:
        ws.load_http_config({"WEATHER_PORT": value})
    assert "WEATHER_PORT" in str(excinfo.value)
