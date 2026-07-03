import time
import RPi.GPIO as GPIO

# -----------------------------------
# Turbo inlet valve test (BCM GPIO 22 / physical pin 15)
# GPIO 22 -> 470R -> BC547 -> relay coil; valve on relay NC contact.
# GPIO HIGH = relay energized = NC broken   = VALVE OPEN
# GPIO LOW  = relay released  = NC shorted  = VALVE CLOSED
# Toggles every 5 s so actuation can be verified.
# -----------------------------------

GPIO.setmode(GPIO.BCM)

GPIO.setup(22, GPIO.OUT)

# -----------------------------------
# Main Loop
# -----------------------------------

try:
    while True:

        print("VALVE OPEN   (GPIO HIGH, relay energized, NC contact broken)")
        GPIO.output(22, GPIO.HIGH)

        time.sleep(5)

        print("VALVE CLOSED (GPIO LOW, relay released, NC contact shorted)")
        GPIO.output(22, GPIO.LOW)

        time.sleep(5)
except KeyboardInterrupt:
    GPIO.cleanup()
