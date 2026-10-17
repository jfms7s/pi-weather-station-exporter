"""Fake hardware modules so Weather_Station can be exercised off a Raspberry Pi.

Every fake records what was done to it in ``EVENTS`` (a list of tuples) and can be
made to fail: set ``raise_on_init`` on a fake class to make construction raise, or
put an attribute name in an instance's ``raise_on_read`` to make reading it raise.
"""

import sys
import types
from typing import ClassVar

EVENTS = []


class SensorReadError(RuntimeError):
    """Raised by a fake whose read has been set to fail."""


class FakeDevice:
    raise_on_init = None
    readings: ClassVar[dict] = {}

    def __init__(self, *args, **kwargs):
        if type(self).raise_on_init is not None:
            raise type(self).raise_on_init
        EVENTS.append((type(self).__name__, args, kwargs))
        self.args = args
        self.kwargs = kwargs
        self.raise_on_read = set()

    def __getattr__(self, name):
        readings = type(self).readings
        if name not in readings:
            raise AttributeError(name)
        if name in self.__dict__.get("raise_on_read", ()):
            raise SensorReadError(f"{type(self).__name__}.{name} read failed")
        return readings[name]


class I2C(FakeDevice):
    pass


class VEML6075(FakeDevice):
    readings: ClassVar[dict] = {"integration_time": 200, "uva": 11.0, "uvb": 12.0, "uv_index": 1.5}


class DHT22(FakeDevice):
    readings: ClassVar[dict] = {"temperature": 21.5, "humidity": 40.0}


class BMP3XX_I2C(FakeDevice):
    readings: ClassVar[dict] = {"temperature": 19.0, "pressure": 1013.0}

    def __setattr__(self, name, value):
        if name in ("pressure_oversampling", "temperature_oversampling"):
            EVENTS.append(("BMP3XX_I2C.set", name, value))
        object.__setattr__(self, name, value)


class Adafruit_BME280_I2C(FakeDevice):
    readings: ClassVar[dict] = {"temperature": 20.0, "pressure": 1012.0, "humidity": 55.0}


class PCF8591(FakeDevice):
    pass


class AnalogIn(FakeDevice):
    readings: ClassVar[dict] = {"value": 32768, "reference_voltage": 3.3}


class FakeGPIO(types.ModuleType):
    BCM = "BCM"
    IN = "IN"
    PUD_UP = "PUD_UP"
    BOTH = "BOTH"

    def __init__(self):
        super().__init__("RPi.GPIO")
        self.callbacks = {}

    def setup(self, pin, mode, pull_up_down=None):
        EVENTS.append(("setup", pin, mode, pull_up_down))

    def add_event_detect(self, pin, edge, callback=None):
        EVENTS.append(("add_event_detect", pin, edge))
        self.callbacks[pin] = callback

    def cleanup(self):
        EVENTS.append(("cleanup",))

    def setmode(self, mode):
        EVENTS.append(("setmode", mode))


def _module(name, **attrs):
    module = types.ModuleType(name)
    for key, value in attrs.items():
        setattr(module, key, value)
    return module


def build_modules():
    """Return {module name: fake module} for every hardware import in Weather_Station."""
    gpio = FakeGPIO()
    pcf8591 = _module("adafruit_pcf8591.pcf8591", PCF8591=PCF8591, A0=0)
    analog_in = _module("adafruit_pcf8591.analog_in", AnalogIn=AnalogIn)
    bme280_basic = _module("adafruit_bme280.basic", Adafruit_BME280_I2C=Adafruit_BME280_I2C)
    rpi = _module("RPi", GPIO=gpio)
    return {
        "board": _module("board", SCL="SCL", SDA="SDA", D19="D19"),
        "busio": _module("busio", I2C=I2C),
        "adafruit_veml6075": _module("adafruit_veml6075", VEML6075=VEML6075),
        "adafruit_dht": _module("adafruit_dht", DHT22=DHT22),
        "adafruit_bmp3xx": _module("adafruit_bmp3xx", BMP3XX_I2C=BMP3XX_I2C),
        "adafruit_bme280": _module("adafruit_bme280", basic=bme280_basic),
        "adafruit_bme280.basic": bme280_basic,
        "RPi": rpi,
        "RPi.GPIO": gpio,
        "adafruit_pcf8591": _module("adafruit_pcf8591", pcf8591=pcf8591, analog_in=analog_in),
        "adafruit_pcf8591.pcf8591": pcf8591,
        "adafruit_pcf8591.analog_in": analog_in,
    }


def install(target=None):
    """Register the fake modules and clear EVENTS.

    ``target`` is a pytest ``monkeypatch`` (undone after the test) or None to write
    straight into ``sys.modules``. Returns the dict of fake modules.
    """
    EVENTS.clear()
    modules = build_modules()
    for name, module in modules.items():
        if target is None:
            sys.modules[name] = module
        else:
            target.setitem(sys.modules, name, module)
    return modules
