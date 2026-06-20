import time

import RPi.GPIO as GPIO
from adafruit_ads1x15.analog_in import AnalogIn
from config import (
    ADC_VREF,
    MFC_FULL_SCALE,
    MFC_SETPOINT_VOLTAGE_FULL_SCALE,
    MFC_GAS_CORRECTION_FACTOR,
    ADC_CHANNEL_MFC,
    GPIO_MFC_VALVE_CLOSE_PIN,
    ARGON_DAC_I2C_ADDRESS,
    ARGON_DAC_VREF,
    ARGON_DAC_RESOLUTION,
    PRESSURE_CONTROL_KP,
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

        # Configure the DAC if we have an I2C bus.
        self.dac_ready = False

        self.dac_voltage = 0.0
        self.dac_code = 0
        if self.i2c is not None:
            i2c_locked = False
            try:
                i2c_locked = self.i2c.try_lock()
                if i2c_locked:
                    addrs = self.i2c.scan()
                    self.dac_ready = self.dac_address in addrs
            except Exception:
                self.dac_ready = False
            finally:
                if i2c_locked:
                    try:
                        self.i2c.unlock()
                    except Exception:
                        pass

    def set_flow(self, sccm):
        """Set the argon MFC flow target via the DAC.

        The MFC wants 0–5V for 0–MFC_FULL_SCALE sccm. The MCP4725 here is powered
        off the Pi's 3.3V rail (no level shifter), so it can only physically put out
        0–ARGON_DAC_VREF volts. We compute the voltage the MFC actually needs, then
        clamp to what the DAC can deliver — until a level-shift/scaling circuit is
        added, commanded flow above (ARGON_DAC_VREF / MFC_SETPOINT_VOLTAGE_FULL_SCALE)
        * MFC_FULL_SCALE will be capped at that ceiling.
        """
        self.flow_target = max(0.0, min(float(sccm), MFC_FULL_SCALE))

        if not self.dac_ready or self.i2c is None:
            return

        required_voltage = (self.flow_target / MFC_FULL_SCALE) * MFC_SETPOINT_VOLTAGE_FULL_SCALE
        dac_voltage = min(required_voltage, ARGON_DAC_VREF)
        code = int((dac_voltage / ARGON_DAC_VREF) * (ARGON_DAC_RESOLUTION - 1))
        code = max(0, min(code, ARGON_DAC_RESOLUTION - 1))
        self.dac_voltage = dac_voltage
        self.dac_code = code
        self._write_dac(code)

    def _write_dac(self, code):
        """Write a 12-bit code to the MCP4725 using a retrying lock mechanism."""
        if self.i2c is None:
            return

        # Prepare standard MCP4725 fast-write command bytes
        # [0b01000000 (write DAC register), high_byte, low_byte]
        high_byte = (code >> 4) & 0xFF
        low_byte = (code << 4) & 0xFF
        data = bytes([0x40, high_byte, low_byte])

        # Retry loop to guarantee delivery instead of dropping the write
        timeout = 0.1  # 100 milliseconds max wait time
        start = time.time()
        i2c_locked = False
        
        while time.time() - start < timeout:
            if self.i2c.try_lock():
                i2c_locked = True
                break
            time.sleep(0.002) # Yield briefly before re-trying

        if not i2c_locked:
            print("[WARN] DAC write dropped: I2C bus stayed busy too long.")
            return

        try:
            self.i2c.writeto(self.dac_address, data)
        except Exception as e:
            print(f"[ERROR] Physical DAC I2C transaction failed: {e}")
        finally:
            self.i2c.unlock()

    def read(self):
        """Read sensor and return calculated flow. Returns a result dict."""
        adc     = self.channel.value
        voltage = self.channel.voltage
        flow    = (voltage / ADC_VREF) * MFC_FULL_SCALE * MFC_GAS_CORRECTION_FACTOR

        return {
            "adc": adc,
            "voltage": voltage,
            "flow": flow,
            "valve_closed": self.valve_closed,
            "dac_ready": self.dac_ready,
            "flow_target": self.flow_target,
            "dac_voltage": self.dac_voltage,
            "dac_code": self.dac_code,
        }


    def valve_close(self):
            """
            Force valve closed by pulling MFC valve close pin LOW.
            This is an emergency closure mechanism.
            """
            GPIO.setup(self.valve_close_pin, GPIO.OUT)
            GPIO.output(self.valve_close_pin, GPIO.LOW)
            self.valve_closed = True
            self.set_flow(0.0)

    def valve_release(self):
        """
        Release valve-close override by setting pin to INPUT.
        MFC resumes normal setpoint control from upstream controller.
        """
        GPIO.setup(self.valve_close_pin, GPIO.IN)
        self.valve_closed = False

    def pressure_control_step(self, current_voltage, target_voltage):
        """Proportional control step: nudge MFC flow to drive Pirani toward target_voltage.

        Higher voltage = higher pressure. If current > target we are above the desired
        pressure, so reduce flow. If current < target we are below, so increase flow.
        Called once per poll tick from the polling thread.
        """
        error = current_voltage - target_voltage   # positive → too much pressure
        delta = PRESSURE_CONTROL_KP * error        # sccm to subtract
        new_flow = max(0.0, min(self.flow_target - delta, MFC_FULL_SCALE))
        self.set_flow(new_flow)
