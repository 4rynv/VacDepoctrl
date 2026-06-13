import RPi.GPIO as GPIO
from adafruit_ads1x15.analog_in import AnalogIn
from config import (
    ADC_VREF,
    MFC_FULL_SCALE,
    MFC_GAS_CORRECTION_FACTOR,
    ADC_CHANNEL_MFC,
    GPIO_MFC_VALVE_CLOSE_PIN,
    ARGON_DAC_I2C_ADDRESS,
)


class MFCController:

    def __init__(self, ads, i2c=None):
        self.valve_close_pin = GPIO_MFC_VALVE_CLOSE_PIN
        self.valve_closed = False
        self.flow_target = 0.0
        self.i2c = i2c
        self.dac_address = ARGON_DAC_I2C_ADDRESS

        # ADC channel A1
        self.channel = AnalogIn(ads, ADC_CHANNEL_MFC)

        # Initialize valve close pin (default: released)
        GPIO.setup(self.valve_close_pin, GPIO.IN)

        # Placeholder for DAC initialization on the shared I2C bus.
        # TODO: replace with actual DAC driver when hardware details are known.
        # Example: self.dac = SomeDAC(self.i2c, address=self.dac_address)

    def set_flow(self, sccm):
        """Set the argon MFC flow target via the DAC (placeholder)."""
        self.flow_target = float(sccm)
        # TODO: write the target to the DAC when hardware/address details are available.
        # if self.i2c is not None:
        #     self.dac.set_voltage_for_flow(self.flow_target)

    def read(self):
        """Read sensor and return calculated flow. Returns a result dict."""
        adc     = self.channel.value
        voltage = self.channel.voltage
        flow    = (voltage / ADC_VREF) * MFC_FULL_SCALE * MFC_GAS_CORRECTION_FACTOR

        return {
            "adc":         adc,
            "voltage":     voltage,
            "flow":        flow,
            "valve_closed": self.valve_closed,
        }

    def valve_close(self):
        """
        Force valve closed by pulling MFC valve close pin LOW.
        This is an emergency closure mechanism.
        """
        GPIO.setup(self.valve_close_pin, GPIO.OUT)
        GPIO.output(self.valve_close_pin, GPIO.LOW)
        self.valve_closed = True

    def valve_release(self):
        """
        Release valve-close override by setting pin to INPUT.
        MFC resumes normal setpoint control from upstream controller.
        """
        GPIO.setup(self.valve_close_pin, GPIO.IN)
        self.valve_closed = False
