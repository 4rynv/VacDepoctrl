from adafruit_ads1x15.analog_in import AnalogIn
from config import (
    ADC_CHANNEL_TURBO_RPM,
    TURBO_RPM_VOLTAGE_FULL_SCALE,
    TURBO_RPM_FULL_SCALE,
)


class TurboRPMController:
    """Reads the turbo pump's tach output (0-3.3V = 0-90000 RPM) on ADC A2.
    Display only for now; not yet wired into any control decision.
    """

    def __init__(self, ads):
        self.channel = AnalogIn(ads, ADC_CHANNEL_TURBO_RPM)

    def read(self):
        adc     = self.channel.value
        voltage = self.channel.voltage
        rpm     = (voltage / TURBO_RPM_VOLTAGE_FULL_SCALE) * TURBO_RPM_FULL_SCALE
        rpm     = max(0.0, min(rpm, TURBO_RPM_FULL_SCALE))

        return {
            "adc": adc,
            "voltage": voltage,
            "rpm": rpm,
        }
