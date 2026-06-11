import RPi.GPIO as GPIO
from adafruit_ads1x15.analog_in import AnalogIn


class PiraniController:
    # -----------------------------------
    # Thresholds
    # -----------------------------------
    ON_THRESHOLD  = 10500   # opto ON  when ADC drops ≤ this  (vacuum achieved)
    OFF_THRESHOLD = 15000   # opto OFF when ADC rises ≥ this  (vacuum lost — must be > ON)

    def __init__(self, ads, opto_pin=17):
        self.opto_pin     = opto_pin
        self.opto_enabled = False

        # GPIO — main.py must call GPIO.setmode() before instantiating
        GPIO.setup(opto_pin, GPIO.OUT)
        GPIO.output(opto_pin, GPIO.LOW)

        # ADC channel A0
        self.channel = AnalogIn(ads, 0)

    def read(self, auto_opto=True):
        """Read sensor. If auto_opto=False, opto state is not changed."""
        adc     = self.channel.value
        voltage = self.channel.voltage

        if auto_opto:
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

    def set_opto(self, enabled):
        """Manually force opto ON or OFF."""
        GPIO.output(self.opto_pin, GPIO.HIGH if enabled else GPIO.LOW)
        self.opto_enabled = enabled

    def shutdown(self):
        """Drive opto low on exit."""
        self.set_opto(False)
