# sim_hardware.py — Fully-simulated hardware for running main.py's GUI and
# control loop without a Raspberry Pi or any real sensors attached.
#
# Enable with `SPUTTER_SIM=1 python main.py` or `python main.py --sim`.
#
# Installs fake RPi.GPIO / adafruit_ads1x15 / adafruit_extended_bus / serial
# modules into sys.modules -- the same sys.modules-injection technique
# tests/unit/fakes.py already uses for the hardware-free test suite -- but
# backed by a live ChamberSim model that evolves believable pressure/flow/
# RPM/current values based on what the real control loop actually commands
# (GPIO/DAC/serial writes), instead of the test suite's static, manually-set
# values. main.py's own hardware-init and control code runs completely
# unmodified against these; nothing in main.py needs to know it's simulated.
#
# SAFETY: this never touches real hardware. Nothing here can open a real
# serial port, toggle a real GPIO pin, or write to a real I2C device --
# every class below is a pure-Python stand-in.

import sys
import types

from config import (
    ADC_VREF,
    ADC_CHANNEL_PIRANI,
    ADC_CHANNEL_MFC,
    ADC_CHANNEL_TURBO_RPM,
    GPIO_PIRANI_PIN,
    GPIO_TURBO_VALVE_PIN,
    MFC_FULL_SCALE,
    MFC_GAS_CORRECTION_FACTOR,
    MFC_SETPOINT_VOLTAGE_FULL_SCALE,
    TURBO_RPM_FULL_SCALE,
    TURBO_RPM_VOLTAGE_FULL_SCALE,
    ADS1115_I2C_ADDRESS,
    ARGON_DAC_I2C_ADDRESS,
    ARGON_DAC_VREF,
    ARGON_DAC_RESOLUTION,
    PZEM_SLAVE_ADDRESS,
)


class ChamberSim:
    """Simple first-order dynamics model of the vacuum chamber, turbo pump,
    and MFC -- driven entirely by what the real control loop writes
    (GPIO pin states, DAC codes) and read back as ADC voltages. Not a
    metrology-grade model -- just believable enough to walk the real state
    machine through a full IDLE -> ... -> SPUTTERING -> VENTING -> IDLE
    cycle at a demoable pace, exercising the real production code paths
    (cross-sensor checks, plasma detection, the vent-complete gate) against
    plausible, evolving signals instead of static values.

    Pressure is modeled directly in Pirani ADC-volt space (0V ~ high vacuum,
    ~3.3V ~ atmosphere) rather than converting through the mbar calibration
    table -- avoids importing main.py's calibration function (which would
    require main.py to import this module and vice versa) and doesn't need
    physical-unit precision for a dev/demo aid.
    """

    def __init__(self):
        self.pirani_v = 3.3       # start at atmosphere, like a real cold start
        self.turbo_rpm = 0.0
        self.mfc_flow = 0.0       # measured flow, lags commanded
        self.pzem_current = 0.02  # ~idle/leakage reading, not exactly zero

        self._turbo_enabled = False   # GPIO_PIRANI_PIN (turbo enable opto)
        self._turbo_valve_open = False  # GPIO_TURBO_VALVE_PIN
        self._commanded_flow = 0.0
        self._plasma_struck = False   # operator-controlled, see set_plasma_struck()

    # ── Inputs from the (simulated) hardware-facing code ──────────
    def set_turbo_enabled(self, on):
        self._turbo_enabled = bool(on)

    def set_turbo_valve(self, open_):
        self._turbo_valve_open = bool(open_)

    def set_mfc_flow(self, sccm):
        self._commanded_flow = max(0.0, min(MFC_FULL_SCALE, sccm))

    def set_plasma_struck(self, struck):
        """Manual operator control (SIM_MODE GUI's 'Plasma Strike (SIM)'
        toggle) -- deliberately not automatic. A flow+low-pressure heuristic
        was tried first, but it fired during ARGON_FLUSH itself (PID
        legitimately drives flow/pressure into the same ballpark while just
        converging on the flush target, well before any real ignition
        attempt), energizing the simulated variac before the operator had
        done anything resembling a strike. `_ignite_plasma()` is still a
        stub on real hardware too -- nothing decides this automatically
        there either, so the simulation shouldn't pretend otherwise.
        """
        self._plasma_struck = bool(struck)

    # ── Derived ADC voltages (what AnalogIn.voltage returns) ───────
    @property
    def mfc_v(self):
        full_scale_flow = MFC_FULL_SCALE * MFC_GAS_CORRECTION_FACTOR
        return max(0.0, min(ADC_VREF, self.mfc_flow / full_scale_flow * ADC_VREF))

    @property
    def turbo_v(self):
        return max(0.0, min(TURBO_RPM_VOLTAGE_FULL_SCALE,
                            self.turbo_rpm / TURBO_RPM_FULL_SCALE * TURBO_RPM_VOLTAGE_FULL_SCALE))

    # ── Physics tick, called once per poll tick from main.py's _poll() ──
    def step(self, dt):
        # Turbo RPM ramps toward its target based on the enable signal --
        # ~30s 0-to-full-scale, comfortably under TURBO_SPINUP_GRACE_SECONDS
        # (60s) so the real spin-up cross-sensor check doesn't spuriously trip.
        target_rpm = TURBO_RPM_FULL_SCALE if self._turbo_enabled else 0.0
        rate = 3000.0  # RPM/s
        if self.turbo_rpm < target_rpm:
            self.turbo_rpm = min(target_rpm, self.turbo_rpm + rate * dt)
        else:
            self.turbo_rpm = max(target_rpm, self.turbo_rpm - rate * dt)

        # MFC measured flow chases commanded flow with a short lag (~0.5s
        # time constant) -- well within MFC_FLOW_GRACE_SECONDS (5s), so the
        # real MFC-response cross-sensor check doesn't spuriously trip.
        self.mfc_flow += (self._commanded_flow - self.mfc_flow) * min(1.0, dt / 0.5)

        # Pressure: modeled as relaxing toward a moving equilibrium, not as
        # a raw per-tick addition -- a first attempt added the gas-load
        # term directly (scaled only by dt), which could swing pressure by
        # ~0.5V in a single 0.2s tick whenever commanded flow jumped. The
        # real PID's derivative term is tuned for the physical rig's actual
        # response time, so a swing that sharp fed it a huge instantaneous
        # error-derivative and sent flow_target oscillating 0<->700 sccm
        # (bang-bang), pressure with it -- never converging. Relaxing
        # toward an equilibrium with its own smooth time constant instead
        # decouples "how big the tick-to-tick step is" from "how large the
        # underlying flow swing was", the same qualitative fix a real
        # under-damped control loop gets from adding lag/filtering.
        pump_rate = 0.15 + 0.5 * (self.turbo_rpm / TURBO_RPM_FULL_SCALE)
        if self._turbo_valve_open:
            equilibrium_v = 3.3
            tau = 1.5
        else:
            # Equilibrium of dp/dt = -p*pump_rate + flow*k, solved for
            # dp/dt = 0. Coefficient (k=0.004) chosen so a flow in
            # ARGON_FLUSH_FLOW_SETPOINT's ballpark (150 sccm) at full turbo
            # speed lands equilibrium pressure near ARGON_FLUSH_TARGET_VOLTAGE
            # (1.287V) -- close enough to this rig's real flush setpoint/
            # target relationship for the PID loop to actually have
            # somewhere reachable to converge to.
            equilibrium_v = self.mfc_flow * 0.004 / max(pump_rate, 0.05)
            tau = 2.0
        self.pirani_v += (equilibrium_v - self.pirani_v) * min(1.0, dt / tau)
        self.pirani_v = max(0.0, min(3.3, self.pirani_v))

        # Plasma current reflects only the operator's manual toggle -- see
        # set_plasma_struck().
        self.pzem_current = 1.4 if self._plasma_struck else 0.02


class _SimGPIO:
    BCM = 11
    BOARD = 10
    OUT = 0
    IN = 1
    HIGH = 1
    LOW = 0

    def __init__(self, chamber):
        self.chamber = chamber
        self.pin_modes = {}
        self.pin_levels = {}

    def setmode(self, mode):
        pass

    def setwarnings(self, flag):
        pass

    def setup(self, pin, mode):
        self.pin_modes[pin] = mode
        self.pin_levels.setdefault(pin, 0)

    def output(self, pin, value):
        level = 1 if value else 0
        self.pin_levels[pin] = level
        if pin == GPIO_PIRANI_PIN:
            self.chamber.set_turbo_enabled(bool(level))
        elif pin == GPIO_TURBO_VALVE_PIN:
            self.chamber.set_turbo_valve(bool(level))

    def input(self, pin):
        return self.pin_levels.get(pin, 0)

    def cleanup(self):
        pass


class _SimAnalogIn:
    def __init__(self, chamber, channel):
        self.chamber = chamber
        self.channel = channel

    @property
    def voltage(self):
        if self.channel == ADC_CHANNEL_PIRANI:
            return self.chamber.pirani_v
        if self.channel == ADC_CHANNEL_MFC:
            return self.chamber.mfc_v
        if self.channel == ADC_CHANNEL_TURBO_RPM:
            return self.chamber.turbo_v
        return 0.0

    @property
    def value(self):
        return int(self.voltage / 4.096 * 32767)


class _SimI2C:
    def __init__(self, chamber):
        self.chamber = chamber
        self.locked = False

    def try_lock(self):
        if self.locked:
            return False
        self.locked = True
        return True

    def unlock(self):
        self.locked = False

    def scan(self):
        return [ADS1115_I2C_ADDRESS, ARGON_DAC_I2C_ADDRESS]

    def writeto(self, address, data):
        if address != ARGON_DAC_I2C_ADDRESS:
            return
        data = bytes(data)
        if len(data) != 3 or data[0] != 0x40:
            return
        code = (data[1] << 4) | (data[2] >> 4)
        dac_voltage = code / (ARGON_DAC_RESOLUTION - 1) * ARGON_DAC_VREF
        flow_target = dac_voltage / MFC_SETPOINT_VOLTAGE_FULL_SCALE * MFC_FULL_SCALE
        self.chamber.set_mfc_flow(flow_target)

    def readfrom_into(self, address, buf):
        for i in range(len(buf)):
            buf[i] = 0


class _SimADS1115:
    def __init__(self, chamber):
        self.chamber = chamber
        self.gain = 1


class _SimSerial:
    """Stand-in for pyserial's Serial, used by pzem_meter.py. Ignores the
    request bytes entirely and always answers with a freshly-built, valid
    Modbus response reflecting the chamber's current simulated PZEM
    reading -- reuses pzem_meter's own CRC implementation rather than
    duplicating it.
    """

    def __init__(self, chamber, port=None, baudrate=9600, bytesize=8,
                 parity="N", stopbits=1, timeout=None):
        self.chamber = chamber

    def reset_input_buffer(self):
        pass

    def write(self, data):
        pass

    def read(self, n):
        from pzem_meter import _crc16_modbus

        voltage_reg = 2300          # 230.0 V nominal simulated mains
        current = int(round(self.chamber.pzem_current * 1000))
        power = int(round(self.chamber.pzem_current * 230.0 * 10))
        energy = 0
        freq_reg = 500              # 50.0 Hz
        pf_reg = 95

        regs = [
            voltage_reg,
            current & 0xFFFF, (current >> 16) & 0xFFFF,
            power & 0xFFFF, (power >> 16) & 0xFFFF,
            energy & 0xFFFF, (energy >> 16) & 0xFFFF,
            freq_reg, pf_reg, 0,
        ]
        payload = bytes([PZEM_SLAVE_ADDRESS, 0x04, len(regs) * 2])
        for r in regs:
            payload += bytes([(r >> 8) & 0xFF, r & 0xFF])
        crc = _crc16_modbus(payload)
        return payload + bytes([crc & 0xFF, (crc >> 8) & 0xFF])

    def close(self):
        pass


def install(chamber=None):
    """Installs simulated hardware modules into sys.modules. Must be called
    BEFORE `import RPi.GPIO`, `import adafruit_ads1x15...`, or `import
    serial` run anywhere (main.py checks the sim flag and calls this at the
    very top, before those imports). Returns the shared ChamberSim instance
    so the caller can drive its step() loop once per poll tick.
    """
    if chamber is None:
        chamber = ChamberSim()

    rpi = types.ModuleType("RPi")
    gpio = _SimGPIO(chamber)
    rpi.GPIO = gpio
    sys.modules["RPi"] = rpi
    sys.modules["RPi.GPIO"] = gpio

    i2c = _SimI2C(chamber)

    pkg = types.ModuleType("adafruit_ads1x15")
    ads1115_mod = types.ModuleType("adafruit_ads1x15.ads1115")
    ads1115_mod.ADS1115 = lambda i2c_arg, address=None: _SimADS1115(chamber)
    analog_in_mod = types.ModuleType("adafruit_ads1x15.analog_in")
    analog_in_mod.AnalogIn = lambda ads, channel: _SimAnalogIn(chamber, channel)
    pkg.ads1115 = ads1115_mod
    pkg.analog_in = analog_in_mod
    sys.modules["adafruit_ads1x15"] = pkg
    sys.modules["adafruit_ads1x15.ads1115"] = ads1115_mod
    sys.modules["adafruit_ads1x15.analog_in"] = analog_in_mod

    ext_bus = types.ModuleType("adafruit_extended_bus")
    ext_bus.ExtendedI2C = lambda bus_number: i2c
    sys.modules["adafruit_extended_bus"] = ext_bus

    fake_serial = types.ModuleType("serial")
    fake_serial.Serial = lambda *a, **kw: _SimSerial(chamber, *a, **kw)
    fake_serial.EIGHTBITS = 8
    fake_serial.PARITY_NONE = "N"
    fake_serial.STOPBITS_ONE = 1
    sys.modules["serial"] = fake_serial

    return chamber
