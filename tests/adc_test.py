
import time

import adafruit_ads1x15.ads1115 as ADS

from adafruit_ads1x15.analog_in import AnalogIn
from adafruit_extended_bus import ExtendedI2C

# -----------------------------------
# Initialize I2C
# bus 1 = hardware I2C: SDA=GPIO2 (pin 3), SCL=GPIO3 (pin 5)
# -----------------------------------

i2c = ExtendedI2C(1)

# -----------------------------------
# Initialize ADS1115
# -----------------------------------

ads = ADS.ADS1115(i2c)

# -----------------------------------
# Set gain
# -----------------------------------

ads.gain = 1

# -----------------------------------
# Create channel A0
# -----------------------------------

channel = AnalogIn(
            ads,
                0
                )

# -----------------------------------
# Main Loop
# -----------------------------------

while True:
    print("--------------------------------")
    print(f"Raw ADC Value : {channel.value}")
    print(f"Voltage       : {channel.voltage:.6f} V")
    time.sleep(0.5)
