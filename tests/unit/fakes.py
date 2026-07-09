"""
Fake hardware modules so the controller code imports and runs on any machine
(no RPi.GPIO, no Adafruit libraries, no I2C bus required).

Call install() BEFORE importing pirani / mfc_control / state_machine.
"""

import sys
import types


# ────────────────────────────────────────────────────────
#  Fake RPi.GPIO
# ────────────────────────────────────────────────────────
class FakeGPIOModule:
    BCM  = 11
    BOARD = 10
    OUT  = 0
    IN   = 1
    HIGH = 1
    LOW  = 0

    def __init__(self):
        self.reset()

    def reset(self):
        self.mode = None
        self.pin_modes = {}    # pin -> OUT/IN
        self.pin_levels = {}   # pin -> 0/1
        self.cleaned_up = False

    def setmode(self, mode):
        self.mode = mode

    def setwarnings(self, flag):
        pass

    def setup(self, pin, mode):
        self.pin_modes[pin] = mode
        self.pin_levels.setdefault(pin, 0)

    def output(self, pin, value):
        self.pin_levels[pin] = 1 if value else 0

    def input(self, pin):
        return self.pin_levels.get(pin, 0)

    def cleanup(self):
        self.cleaned_up = True
        self.pin_modes.clear()


# ────────────────────────────────────────────────────────
#  Fake ADS1115 + AnalogIn
# ────────────────────────────────────────────────────────
class FakeADS1115:
    """Set .channel_voltages / .channel_values from tests to feed sensor data."""

    def __init__(self, i2c=None, address=0x48):
        self.i2c = i2c
        self.address = address
        self.gain = 1
        self.channel_voltages = {0: 0.0, 1: 0.0, 2: 0.0, 3: 0.0}
        self.channel_values   = {0: 0,   1: 0,   2: 0,   3: 0}

    def set_channel(self, channel, voltage=None, value=None):
        if voltage is not None:
            self.channel_voltages[channel] = voltage
            if value is None:
                # roughly consistent counts for gain=1 (±4.096V over 16-bit signed)
                value = int(voltage / 4.096 * 32767)
        if value is not None:
            self.channel_values[channel] = value


class FakeAnalogIn:
    def __init__(self, ads, channel):
        self._ads = ads
        self._channel = channel

    @property
    def value(self):
        return self._ads.channel_values.get(self._channel, 0)

    @property
    def voltage(self):
        return self._ads.channel_voltages.get(self._channel, 0.0)


# ────────────────────────────────────────────────────────
#  Fake I2C bus (ExtendedI2C stand-in)
# ────────────────────────────────────────────────────────
class FakeI2C:
    def __init__(self, devices=(0x48, 0x60)):
        self.devices = list(devices)
        self.locked = False
        self.writes = []            # list of (address, bytes)

    def try_lock(self):
        if self.locked:
            return False
        self.locked = True
        return True

    def unlock(self):
        self.locked = False

    def scan(self):
        return list(self.devices)

    def writeto(self, address, data):
        if address not in self.devices:
            raise OSError(f"No ACK from 0x{address:02X}")
        self.writes.append((address, bytes(data)))

    def readfrom_into(self, address, buf):
        if address not in self.devices:
            raise OSError(f"No ACK from 0x{address:02X}")
        for i in range(len(buf)):
            buf[i] = 0

    # convenience for assertions
    def last_dac_code(self, address=0x60):
        """Decode the most recent MCP4725 fast-write (0x40, hi, lo) to a 12-bit code."""
        for addr, data in reversed(self.writes):
            if addr == address and len(data) == 3 and data[0] == 0x40:
                return (data[1] << 4) | (data[2] >> 4)
        return None


# ────────────────────────────────────────────────────────
#  Fake pyserial (PZEMController's transport)
# ────────────────────────────────────────────────────────
class FakeSerial:
    """Stand-in for serial.Serial. Records every write() frame in `.writes`;
    tests queue canned response bytes in `.responses` (one entry consumed
    per read() call; an empty queue returns b"", simulating a timeout).
    Constructing with a port name listed in FAIL_PORTS raises OSError, so
    tests can simulate "meter not plugged in" without touching real hardware.
    """

    FAIL_PORTS = set()

    def __init__(self, port, baudrate=9600, bytesize=8, parity="N",
                 stopbits=1, timeout=None):
        if port in FakeSerial.FAIL_PORTS:
            raise OSError(f"[Errno 2] could not open port {port}")
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.writes = []
        self.responses = []
        self.closed = False

    def reset_input_buffer(self):
        pass

    def write(self, data):
        self.writes.append(bytes(data))

    def read(self, n):
        if not self.responses:
            return b""
        return self.responses.pop(0)

    def close(self):
        self.closed = True


# ────────────────────────────────────────────────────────
#  Module installation
# ────────────────────────────────────────────────────────
gpio = FakeGPIOModule()   # single shared instance, reset() between tests


def install():
    """Insert fake hardware modules into sys.modules (idempotent)."""
    if isinstance(sys.modules.get("RPi.GPIO"), FakeGPIOModule):
        return

    rpi = types.ModuleType("RPi")
    rpi.GPIO = gpio
    sys.modules["RPi"] = rpi
    sys.modules["RPi.GPIO"] = gpio

    pkg = types.ModuleType("adafruit_ads1x15")
    ads1115_mod = types.ModuleType("adafruit_ads1x15.ads1115")
    ads1115_mod.ADS1115 = FakeADS1115
    analog_in_mod = types.ModuleType("adafruit_ads1x15.analog_in")
    analog_in_mod.AnalogIn = FakeAnalogIn
    pkg.ads1115 = ads1115_mod
    pkg.analog_in = analog_in_mod
    sys.modules["adafruit_ads1x15"] = pkg
    sys.modules["adafruit_ads1x15.ads1115"] = ads1115_mod
    sys.modules["adafruit_ads1x15.analog_in"] = analog_in_mod

    ext_bus = types.ModuleType("adafruit_extended_bus")
    ext_bus.ExtendedI2C = lambda bus_number: FakeI2C()
    sys.modules["adafruit_extended_bus"] = ext_bus

    board = types.ModuleType("board")
    board.SCL, board.SDA = object(), object()
    sys.modules["board"] = board
    busio = types.ModuleType("busio")
    busio.I2C = lambda scl, sda: FakeI2C()
    sys.modules["busio"] = busio

    fake_serial = types.ModuleType("serial")
    fake_serial.Serial = FakeSerial
    fake_serial.EIGHTBITS = 8
    fake_serial.PARITY_NONE = "N"
    fake_serial.STOPBITS_ONE = 1
    sys.modules["serial"] = fake_serial


# ────────────────────────────────────────────────────────
#  Calibration loader — pulls the table + both interpolation
#  functions out of main.py WITHOUT importing it (main.py
#  starts hardware + the GUI at import time).
# ────────────────────────────────────────────────────────
def load_main_slice(repo_root, overrides=None):
    """Like load_calibration, but lets a test override specific config
    constants before exec -- e.g. to feed config_sanity_failures() a
    hypothetical bad config. Each call re-execs into a fresh namespace, so
    overrides never leak between tests (unlike mutating the shared
    load_calibration() result: a function's free-variable lookups go
    through its *original* __globals__, not a copy of that dict, so
    patching a dict copy silently has no effect on the function at all).
    """
    import math, os
    import config
    src = open(os.path.join(repo_root, "main.py")).read()
    start = src.index("_PIRANI_CAL = [")
    end = src.index("from pirani")
    # turbo_valve_step / cross-sensor / global-gate functions in this slice
    # reference config globals at call time -- inject every constant
    # (config.py holds nothing but ALL_CAPS constants) so new ones don't
    # need to be listed here by hand.
    ns = {"math": math}
    ns.update({k: v for k, v in vars(config).items() if k.isupper()})
    if overrides:
        ns.update(overrides)
    exec(src[start:end], ns)
    return ns


def load_calibration(repo_root):
    return load_main_slice(repo_root)
