"""Run Weather_Station.main() on fake hardware for the shutdown tests.

Prints "ready" once the exporter has started its wait, and "cleanup" (flushed) each time
GPIO.cleanup() is called, so the parent process can order them against its signal."""

import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import hardware_stubs

os.environ["WEATHER_PORT"] = "0"  # ephemeral, so parallel runs never collide
modules = hardware_stubs.install()
modules["RPi.GPIO"].cleanup = lambda: print("cleanup", flush=True)

import Weather_Station as ws  # noqa: E402 (after the fakes are installed)

start_dht_poller = ws.start_dht_poller


def start_and_report(*args, **kwargs):
    result = start_dht_poller(*args, **kwargs)
    print("ready", flush=True)
    return result


ws.start_dht_poller = start_and_report
ws.main()
