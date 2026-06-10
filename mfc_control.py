from adafruit_ads1x15.analog_in import AnalogIn


class MFCController:
    # -----------------------------------
    # Constants
    # -----------------------------------
    ADC_VREF   = 3.33
    FULL_SCALE = 500.0   # sccm at full-scale output
    GCF        = 1.39    # gas correction factor for Ar

    def __init__(self, ads):
        # ADC channel A1
        self.channel = AnalogIn(ads, 1)

    def read(self):
        """Read sensor and return calculated flow. Returns a result dict."""
        adc     = self.channel.value
        voltage = self.channel.voltage
        flow    = (voltage / self.ADC_VREF) * self.FULL_SCALE * self.GCF

        return {
            "adc":     adc,
            "voltage": voltage,
            "flow":    flow,
        }
