import time
import RPi.GPIO as GPIO

# -----------------------------------
# GPIO Setup
# -----------------------------------

GPIO.setmode(GPIO.BCM)

GPIO.setup(17,GPIO.OUT)

# -----------------------------------
# Main Loop
# -----------------------------------

while True:

    print("OPTO ON")
    GPIO.output(17,GPIO.HIGH)

    time.sleep(5)

    print("OPTO OFF")
    GPIO.output(17,GPIO.LOW)

    time.sleep(5)
