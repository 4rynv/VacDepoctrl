import RPi.GPIO as GPIO
from adafruit_ads1x15.analog_in import AnalogIn


class PiraniController:
    # -----------------------------------
    # Thresholds
    # -----------------------------------
    ON_THRESHOLD  = 10500
    OFF_THRESHOLD = 9500

    def __init__(self, ads, opto_pin=17):
        self.opto_pin     = opto_pin
        self.opto_enabled = False

        # GPIO — main.py must call GPIO.setmode() before instantiating
        GPIO.setup(opto_pin, GPIO.OUT)
        GPIO.output(opto_pin, GPIO.LOW)

        # ADC channel A0
        self.channel = AnalogIn(ads, 0)

    def read(self):
        """Read sensor and update opto state. Returns a result dict."""
        adc     = self.channel.value
        voltage = self.channel.voltage

        if (not self.opto_enabled) and (adc <= self.ON_THRESHOLD):
            GPIO.output(self.opto_pin, GPIO.HIGH)
            self.opto_enabled = True

        elif self.opto_enabled and (adc >= self.OFF_THRESHOLD):
            GPIO.output(self.opto_pin, GPIO.LOW)
            self.opto_enabled = False

        return {
            "adc":          adc,
            "voltage":      voltage,
            "opto_enabled": self.opto_enabled,
        }

    def shutdown(self):
        """Drive opto low on exit."""
        GPIO.output(self.opto_pin, GPIO.LOW)
