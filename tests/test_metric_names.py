"""Snapshot of every metric family the exporter exposes.

Change EXPECTED only together with a `feat!:` / `BREAKING CHANGE` commit: dashboards and
the planned consumers (Ansible role, scrape job, OpenDeck station source) depend on these
names."""

import hardware_stubs
from prometheus_client import CollectorRegistry

import Weather_Station as ws

EXPECTED = {
    "uv_sample_time",
    "uva_count",
    "uvb_count",
    "uv_index",
    "temperature_bme280",
    "pressure_bme280",
    "humidity_bme280",
    "temperature_3xx",
    "pressure_3xx",
    "rose_direction",
    "rose_power_percentage",
    "rose_power",
    "anemometer_rotations",
    "anemometer_activation",
    "temperature",
    "humidity",
    "weather_sensor_last_success_timestamp_seconds",
}


def test_exposed_metric_names_match_the_snapshot(monkeypatch):
    hardware_stubs.install(monkeypatch)
    hw = ws.load_hardware()
    registry = CollectorRegistry()

    ws.start_sensors(hw, hw.GPIO, registry=registry)
    ws.register_dht_metrics(ws.DhtState(), registry=registry)

    assert {metric.name for metric in registry.collect()} == EXPECTED
