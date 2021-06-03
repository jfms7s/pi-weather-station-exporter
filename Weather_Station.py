#!/usr/bin/env python3

from prometheus_client import start_http_server,Gauge

import board
import busio
import adafruit_veml6075
import Adafruit_DHT
import adafruit_bmp3xx
import adafruit_bme280
import Adafruit_BMP.BMP085 as BMP085
import RPi.GPIO as GPIO
import itertools
import threading
import adafruit_pcf8591.pcf8591 as PCF
from adafruit_pcf8591.analog_in import AnalogIn


import time
import asyncio

class FastWriteCounter(object):
    def __init__(self):
        self._number_of_read = 0
        self._counter = itertools.count()
        self._read_lock = threading.Lock()
    def increment(self):
        next(self._counter)
    def value(self):
        with self._read_lock:
            value = next(self._counter) - self._number_of_read
            self._number_of_read += 1
        return value

if __name__ == "__main__":

    start_http_server(8705)

    loop = asyncio.get_event_loop()

    i2c = busio.I2C(board.SCL, board.SDA)
    veml = adafruit_veml6075.VEML6075(i2c, integration_time=200)
    pcf = PCF.PCF8591(i2c)

    GPIO.cleanup()
    GPIO.setmode(GPIO.BCM)

    ## UV LIGHT
    Gauge('uv_sample_time', 'time between samples').set_function(lambda: veml.integration_time)
    Gauge('uva_count', 'uva count between samples').set_function(lambda: veml.uva)
    Gauge('uvb_count', 'uvb count between samples').set_function(lambda: veml.uvb)
    Gauge('uv_index', 'Index UV').set_function(lambda: veml.uv_index)

    ## pressure 
    bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c)
    bmp = adafruit_bmp3xx.BMP3XX_I2C(i2c,0x76)

    bmp.pressure_oversampling = 8
    bmp.temperature_oversampling = 2

    Gauge('temperature_bme280', 'temperature in Centigrade').set_function(lambda:bme280.temperature)
    Gauge('pressure_bme280', 'pressure').set_function(lambda: bme280.pressure)
    Gauge('humidity_bme280', 'humidity').set_function(lambda:bme280.humidity)

    Gauge('temperature_3xx', 'temperature in Centigrade').set_function(lambda:bmp.temperature)
    Gauge('pressure_3xx', 'pressure').set_function(lambda: bmp.pressure)

    ## wind
    pcf_in_0 = AnalogIn(pcf, PCF.A0)

    counter = FastWriteCounter()
    pin=17
    num_triggers = 1 / 40

    GPIO.setup(pin, GPIO.IN, pull_up_down = GPIO.PUD_UP)
    GPIO.add_event_detect(pin, GPIO.BOTH, callback=lambda a: counter.increment())

    Gauge('rose_direction', 'current rose direction').set_function(lambda: (pcf_in_0.value / 65535) * 96 )
    Gauge('rose_power_percentage', 'current rose power_percentage').set_function(lambda: (pcf_in_0.value / 65535))
    Gauge('rose_power', 'current rose power').set_function(lambda: (pcf_in_0.value / 65535) * pcf_in_0.reference_voltage)
    Gauge('anemometer_rpm', 'number of complete rotations').set_function(lambda: counter.value() * num_triggers)
    Gauge('anemometer_activation', 'number of signals sent by anometer').set_function(lambda: counter.value())

    ## temperature and pressure
    def scrapeMetrics(DHT_PIN):
        DHT_SENSOR = Adafruit_DHT.DHT22
        temperature_metric = Gauge('temperature', 'temperature in Centigrade')
        humidity_metric = Gauge('humidity', 'humidity percentage')  
        while True:
            humidity, temperature = Adafruit_DHT.read_retry(DHT_SENSOR, DHT_PIN)
            if humidity is not None and temperature is not None:
                temperature_metric.set(temperature)
                humidity_metric.set(humidity)
                print("Temp={0:0.1f}*C  Humidity={1:0.1f}%".format(temperature, humidity))

    loop.run_in_executor(None,scrapeMetrics,19)

    loop.run_forever()

