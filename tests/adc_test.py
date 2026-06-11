
import time
import board
import busio

import adafruit_ads1x15.ads1115 as ADS

from adafruit_ads1x15.analog_in import AnalogIn

# -----------------------------------
# Initialize I2C
# -----------------------------------

i2c = busio.I2C(
            board.SCL,
                board.SDA
                )

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
