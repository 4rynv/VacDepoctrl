import RPi.GPIO as GPIO
from adafruit_ads1x15.analog_in import AnalogIn
from config import (
    TURBOOPTO_ON_THRESHOLD,
    TURBOOPTO_OFF_THRESHOLD,
    ADC_CHANNEL_PIRANI,
    GPIO_PIRANI_PIN,
)


class PiraniController:

    def __init__(self, ads, opto_pin=None):
        # Use config pin if not overridden
        self.opto_pin     = opto_pin if opto_pin is not None else GPIO_PIRANI_PIN
        self.opto_enabled = False

        # GPIO — main.py must call GPIO.setmode() before instantiating
        GPIO.setup(self.opto_pin, GPIO.OUT)
        GPIO.output(self.opto_pin, GPIO.LOW)

        # ADC channel A0
        self.channel = AnalogIn(ads, ADC_CHANNEL_PIRANI)

    def read(self, auto_opto=True):
        """Read sensor. If auto_opto=False, opto state is not changed."""
        adc     = self.channel.value
        voltage = self.channel.voltage

        if auto_opto:
            if (not self.opto_enabled) and (adc <= TURBOOPTO_ON_THRESHOLD):
                GPIO.output(self.opto_pin, GPIO.HIGH)
                self.opto_enabled = True
            elif self.opto_enabled and (adc >= TURBOOPTO_OFF_THRESHOLD):
                GPIO.output(self.opto_pin, GPIO.LOW)
                self.opto_enabled = False

        return {
            "adc":          adc,
            "voltage":      voltage,
            "opto_enabled": self.opto_enabled,
        }

    def set_opto(self, enabled):
        """Manually force opto ON or OFF."""
        GPIO.output(self.opto_pin, GPIO.HIGH if enabled else GPIO.LOW)
        self.opto_enabled = enabled

    def shutdown(self):
        """Drive opto low on exit."""
        self.set_opto(False)
