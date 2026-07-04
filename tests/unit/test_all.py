"""
Hardware-free test suite for the Sputter Vacuum Controller.

Run from the repo root on any machine (no Pi, no sensors, no I2C needed):

    python3 -m unittest discover tests/unit -v

Fake hardware modules are installed before the controller code is imported,
so pirani.py / mfc_control.py / state_machine.py run exactly as written.
main.py is NOT imported (it starts the GUI); its calibration table and
interpolation functions are extracted from source and tested directly.
"""

import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE)
sys.path.insert(0, REPO_ROOT)

import fakes
fakes.install()

import config
from state_machine import SputterStateMachine, VALID_TRANSITIONS, STATE_COLORS
import mfc_control
import pirani as pirani_mod
from mfc_control import MFCController
from pirani import PiraniController
from fakes import FakeADS1115, FakeI2C, gpio

CAL = fakes.load_calibration(REPO_ROOT)
mbar_to_v = CAL["mbar_to_adc_voltage"]
v_to_mbar = CAL["adc_voltage_to_mbar"]
CAL_TABLE = CAL["_PIRANI_CAL_ADC"]

ALL_STATES = list(VALID_TRANSITIONS.keys())


# ════════════════════════════════════════════════════════
#  Config sanity
# ════════════════════════════════════════════════════════
class TestConfigSanity(unittest.TestCase):
    def test_pressure_thresholds_ordered(self):
        self.assertLess(config.PUMP_DOWN_COMPLETE_VOLTAGE,
                        config.ARGON_FLUSH_TARGET_VOLTAGE)
        self.assertLess(config.ARGON_FLUSH_TARGET_VOLTAGE,
                        config.VENTING_COMPLETE_VOLTAGE)
        self.assertLess(config.VENTING_COMPLETE_VOLTAGE,
                        config.IDLE_PRESSURE_MAX_VOLTAGE)

    def test_gpio_pins_distinct(self):
        pins = [config.GPIO_PIRANI_PIN,
                config.GPIO_MFC_VALVE_CLOSE_PIN,
                config.GPIO_TURBO_VALVE_PIN]
        self.assertEqual(len(pins), len(set(pins)), "GPIO pin collision")

    def test_pins_avoid_dead_pads(self):
        # GPIO 2, 3, 4 were affected during early hardware issue; GPIO 23/24 are the I2C bus
        forbidden = {2, 3, 4, 23, 24}
        for pin in (config.GPIO_PIRANI_PIN,
                    config.GPIO_MFC_VALVE_CLOSE_PIN,
                    config.GPIO_TURBO_VALVE_PIN):
            self.assertNotIn(pin, forbidden, f"GPIO{pin} is dead or reserved")

    def test_pid_gains_sane(self):
        self.assertGreaterEqual(config.PRESSURE_CONTROL_KI, 0.0)
        self.assertGreaterEqual(config.PRESSURE_CONTROL_KD, 0.0)
        self.assertGreater(config.PRESSURE_CONTROL_KP, 0.0)
        self.assertGreater(config.PRESSURE_CONTROL_ICLAMP, 0.0)

    def test_timing_sane(self):
        self.assertGreater(config.POLLING_INTERVAL, 0.0)
        self.assertGreater(config.GUI_REFRESH_INTERVAL, 0)
        self.assertGreater(config.PLASMA_IGNITION_TIMEOUT, 0.0)
        self.assertGreater(config.ERROR_DISPLAY_SECONDS, 0)

    def test_dac_config(self):
        self.assertEqual(config.ARGON_DAC_RESOLUTION, 4096)
        self.assertGreater(config.ARGON_DAC_VREF, 0.0)
        self.assertGreater(config.MFC_FULL_SCALE, 0.0)

    def test_i2c_bus_is_software_bus(self):
        # hardware bus 1 pads (GPIO2/3) are dead — bus must be 3
        self.assertEqual(config.I2C_BUS_NUMBER, 3)

    def test_turbo_valve_threshold(self):
        self.assertGreater(config.TURBO_VALVE_OPEN_MBAR, 0.0)
        self.assertLess(config.TURBO_VALVE_OPEN_MBAR, 1.0)


# ════════════════════════════════════════════════════════
#  Pirani calibration interpolation (from main.py source)
# ════════════════════════════════════════════════════════
class TestCalibration(unittest.TestCase):
    def test_known_points_forward(self):
        self.assertAlmostEqual(mbar_to_v(10),    2.706, places=3)
        self.assertAlmostEqual(mbar_to_v(0.09),  1.287, places=3)
        self.assertAlmostEqual(mbar_to_v(0.007), 0.363, places=3)

    def test_known_points_inverse(self):
        self.assertAlmostEqual(v_to_mbar(2.706), 10,    places=2)
        self.assertAlmostEqual(v_to_mbar(1.287), 0.09,  places=4)
        self.assertAlmostEqual(v_to_mbar(0.363), 0.007, places=5)

    def test_clamps(self):
        self.assertEqual(v_to_mbar(5.0), 999)     # above table -> atmosphere
        self.assertAlmostEqual(v_to_mbar(3.30), 999, places=6)
        self.assertEqual(v_to_mbar(0.0), 0.0)     # below table -> over-range
        self.assertEqual(mbar_to_v(2000), CAL_TABLE[0][1])
        self.assertEqual(mbar_to_v(-1), CAL_TABLE[-1][1])

    def test_round_trip_all_table_points(self):
        for p, v in CAL_TABLE[:-1]:
            if p <= 0:
                continue
            self.assertAlmostEqual(v_to_mbar(v), p, delta=p * 1e-6,
                                   msg=f"round-trip failed at {p} mbar")

    def test_round_trip_midpoints(self):
        import random
        rng = random.Random(42)
        for _ in range(200):
            v = rng.uniform(0.01, 3.29)
            self.assertAlmostEqual(mbar_to_v(v_to_mbar(v)), v, places=9)

    def test_monotonic(self):
        """More voltage must always mean more pressure."""
        prev = None
        for i in range(4, 330):
            m = v_to_mbar(i / 100.0)
            if prev is not None:
                self.assertGreaterEqual(m, prev)
            prev = m

    def test_turbo_valve_threshold_voltage(self):
        # 0.01 mbar should sit near 0.512 V ADC per the datasheet table
        self.assertAlmostEqual(mbar_to_v(config.TURBO_VALVE_OPEN_MBAR),
                               0.5115, places=3)


# ════════════════════════════════════════════════════════
#  State machine
# ════════════════════════════════════════════════════════
class TestStateMachine(unittest.TestCase):
    def setUp(self):
        self.sm = SputterStateMachine()

    def _force(self, state):
        self.sm._state = state

    def test_initial_state(self):
        self.assertEqual(self.sm.state, "IDLE")

    def test_idle_to_pumpdown_requires_low_pressure(self):
        self.assertFalse(self.sm.transition("PUMP_DOWN"))                       # no voltage
        self.assertFalse(self.sm.transition("PUMP_DOWN", pirani_voltage=3.0))   # too high
        self.assertTrue(self.sm.transition("PUMP_DOWN", pirani_voltage=2.0))
        self.assertEqual(self.sm.state, "PUMP_DOWN")

    def test_invalid_transitions_rejected(self):
        self.assertFalse(self.sm.transition("SPUTTERING"))
        self.assertFalse(self.sm.transition("VENTING"))     # not allowed from IDLE
        self.assertEqual(self.sm.state, "IDLE")

    def test_pumpdown_to_ready_requires_opto(self):
        self._force("PUMP_DOWN")
        self.sm.update(0.15, opto_enabled=False)
        self.assertEqual(self.sm.state, "PUMP_DOWN", "must not reach READY without opto")
        self.sm.update(0.15, opto_enabled=True)
        self.assertEqual(self.sm.state, "READY")

    def test_pumpdown_aborts_on_pressure_rise(self):
        self._force("PUMP_DOWN")
        self.sm.update(2.8)
        self.assertEqual(self.sm.state, "IDLE")

    def test_ready_degrades_to_pumpdown(self):
        self._force("READY")
        self.sm.update(2.8)
        self.assertEqual(self.sm.state, "PUMP_DOWN")

    def test_argon_flush_has_no_auto_ignition(self):
        """ARGON_FLUSH -> PLASMA_IGNITING is owned by _poll() ONLY (arms the
        timeout + fires _ignite_plasma). sm.update() must never duplicate it."""
        self._force("ARGON_FLUSH")
        self.sm.update(2.0)   # well above ignition threshold
        self.assertEqual(self.sm.state, "ARGON_FLUSH")

    def test_plasma_confirm_is_operator_only(self):
        """No sensor reading may auto-confirm plasma."""
        self._force("PLASMA_IGNITING")
        for v in (0.0, 0.3, 0.363, 0.6, 1.0):
            self.sm.update(v)
            self.assertEqual(self.sm.state, "PLASMA_IGNITING",
                             f"auto-confirmed plasma at {v}V!")
        self.assertTrue(self.sm.transition("SPUTTER_READY"))

    def test_venting_completes_at_atmosphere(self):
        self._force("VENTING")
        self.sm.update(2.4)
        self.assertEqual(self.sm.state, "VENTING")
        self.sm.update(2.6)
        self.assertEqual(self.sm.state, "IDLE")

    def test_estop_from_every_state(self):
        for state in ALL_STATES:
            sm = SputterStateMachine()
            sm._state = state
            sm.emergency_stop()
            self.assertEqual(sm.state, "IDLE", f"E-STOP failed from {state}")

    def test_full_manual_chain(self):
        self._force("READY")
        for target in ("ARGON_FLUSH", "PLASMA_IGNITING", "SPUTTER_READY",
                       "SPUTTERING", "VENTING", "IDLE"):
            self.assertTrue(self.sm.transition(target), f"chain broke at {target}")
        self.assertEqual(self.sm.state, "IDLE")

    def test_callback_fires_on_manual_and_auto(self):
        calls = []
        self.sm.on_transition_callback = lambda old, new: calls.append((old, new))
        self.sm.transition("PUMP_DOWN", pirani_voltage=2.0)
        self.sm.update(0.15, opto_enabled=True)
        self.sm.emergency_stop()
        self.assertEqual(calls, [("IDLE", "PUMP_DOWN"),
                                 ("PUMP_DOWN", "READY"),
                                 ("READY", "IDLE")])

    def test_callback_exception_does_not_break_machine(self):
        def bad_callback(old, new):
            raise RuntimeError("boom")
        self.sm.on_transition_callback = bad_callback
        self.assertTrue(self.sm.transition("PUMP_DOWN", pirani_voltage=2.0))
        self.assertEqual(self.sm.state, "PUMP_DOWN")
        self.sm.update(2.9)   # auto transition with broken callback
        self.assertEqual(self.sm.state, "IDLE")

    def test_every_state_has_a_color(self):
        for state in ALL_STATES:
            self.assertIn(state, STATE_COLORS)


# ════════════════════════════════════════════════════════
#  MFC controller (DAC math, valve, PID)
# ════════════════════════════════════════════════════════
class TestMFCController(unittest.TestCase):
    def setUp(self):
        gpio.reset()
        self.ads = FakeADS1115()
        self.i2c = FakeI2C(devices=[0x60])
        self.mfc = MFCController(self.ads, i2c=self.i2c)
        # deterministic clock for PID tests
        self._t = [1000.0]
        self._orig_time = mfc_control.time.time
        mfc_control.time.time = lambda: self._t[0]

    def tearDown(self):
        mfc_control.time.time = self._orig_time

    def _tick(self, dt=0.2):
        self._t[0] += dt

    def test_dac_detected(self):
        self.assertTrue(self.mfc.dac_ready)

    def test_dac_not_detected_without_device(self):
        mfc2 = MFCController(FakeADS1115(), i2c=FakeI2C(devices=[]))
        self.assertFalse(mfc2.dac_ready)

    def test_set_flow_dac_code(self):
        self.mfc.set_flow(350.0)   # half scale -> 2.5V of 5V vref -> code 2047
        self.assertEqual(self.i2c.last_dac_code(), 2047)
        self.assertAlmostEqual(self.mfc.dac_voltage, 2.5, places=3)

    def test_set_flow_zero(self):
        self.mfc.set_flow(0.0)
        self.assertEqual(self.i2c.last_dac_code(), 0)

    def test_set_flow_clamps(self):
        self.mfc.set_flow(-50)
        self.assertEqual(self.mfc.flow_target, 0.0)
        self.mfc.set_flow(9999)
        self.assertEqual(self.mfc.flow_target, config.MFC_FULL_SCALE)

    def test_dac_voltage_never_exceeds_vref(self):
        self.mfc.set_flow(config.MFC_FULL_SCALE)
        self.assertLessEqual(self.mfc.dac_voltage, config.ARGON_DAC_VREF + 1e-9)
        self.assertLessEqual(self.mfc.dac_code, config.ARGON_DAC_RESOLUTION - 1)

    def test_valve_close_resets_pid_and_flow(self):
        self.mfc.set_flow(100)
        self.mfc._integral = 5.0
        self.mfc._prev_error = 1.0
        self.mfc.valve_close()
        self.assertTrue(self.mfc.valve_closed)
        self.assertEqual(self.mfc.flow_target, 0.0)
        self.assertEqual(self.mfc._integral, 0.0)
        self.assertIsNone(self.mfc._prev_error)
        self.assertEqual(gpio.pin_levels[config.GPIO_MFC_VALVE_CLOSE_PIN], 0)
        self.assertEqual(gpio.pin_modes[config.GPIO_MFC_VALVE_CLOSE_PIN], gpio.OUT)

    def test_valve_release(self):
        self.mfc.valve_close()
        self.mfc.valve_release()
        self.assertFalse(self.mfc.valve_closed)
        self.assertEqual(gpio.pin_modes[config.GPIO_MFC_VALVE_CLOSE_PIN], gpio.IN)

    def test_pid_increases_flow_below_target(self):
        """Pressure below target (voltage low) -> flow must rise."""
        self.mfc.set_flow(100.0)
        self.mfc.pressure_control_step(0.8, 1.287)   # prime prev_error
        self._tick()
        self.mfc.pressure_control_step(0.8, 1.287)
        self.assertGreater(self.mfc.flow_target, 100.0)

    def test_pid_decreases_flow_above_target(self):
        self.mfc.set_flow(100.0)
        self.mfc.pressure_control_step(1.8, 1.287)
        self._tick()
        self.mfc.pressure_control_step(1.8, 1.287)
        self.assertLess(self.mfc.flow_target, 100.0)

    def test_pid_flow_never_negative_or_above_full_scale(self):
        self.mfc.set_flow(0.0)
        for _ in range(50):
            self.mfc.pressure_control_step(3.0, 0.363)   # far above target
            self._tick()
        self.assertGreaterEqual(self.mfc.flow_target, 0.0)
        self.mfc.set_flow(config.MFC_FULL_SCALE)
        for _ in range(50):
            self.mfc.pressure_control_step(0.0, 1.287)   # far below target
            self._tick()
        self.assertLessEqual(self.mfc.flow_target, config.MFC_FULL_SCALE)

    def test_ki_zero_does_not_crash(self):
        """Regression: ICLAMP/KI used to divide by zero when KI=0 (tuning)."""
        orig = mfc_control.PRESSURE_CONTROL_KI
        try:
            mfc_control.PRESSURE_CONTROL_KI = 0.0
            self.mfc.pressure_control_step(1.0, 1.287)
            self._tick()
            self.mfc.pressure_control_step(1.0, 1.287)   # would raise pre-fix
        finally:
            mfc_control.PRESSURE_CONTROL_KI = orig

    def test_integral_clamped(self):
        """Anti-windup: sustained error must not grow the I term without bound."""
        self.mfc.pressure_control_step(3.0, 0.363)
        for _ in range(10000):
            self._tick(1.0)
            self.mfc.pressure_control_step(3.0, 0.363)
        limit = mfc_control.PRESSURE_CONTROL_ICLAMP / mfc_control.PRESSURE_CONTROL_KI
        self.assertLessEqual(abs(self.mfc._integral), limit + 1e-6)

    def test_read_returns_expected_keys(self):
        self.ads.set_channel(1, voltage=1.0)
        m = self.mfc.read()
        for key in ("adc", "voltage", "flow", "valve_closed",
                    "dac_ready", "flow_target", "dac_voltage", "dac_code"):
            self.assertIn(key, m)

    def test_dac_write_survives_busy_bus(self):
        """A locked I2C bus must not crash set_flow (write is dropped with a warning)."""
        mfc_control.time.time = self._orig_time   # real clock: retry loop needs it to time out
        self.i2c.locked = True
        self.mfc.set_flow(100.0)   # must not raise (drops write after ~0.1s)


# ════════════════════════════════════════════════════════
#  Pirani controller (opto logic)
# ════════════════════════════════════════════════════════
class TestPiraniController(unittest.TestCase):
    def setUp(self):
        gpio.reset()
        self.ads = FakeADS1115()
        self.p = PiraniController(self.ads)

    def _set(self, value, voltage=None):
        self.ads.set_channel(config.ADC_CHANNEL_PIRANI, voltage=voltage, value=value)

    def test_opto_starts_off(self):
        self.assertFalse(self.p.opto_enabled)
        self.assertEqual(gpio.pin_levels[config.GPIO_PIRANI_PIN], 0)

    def test_auto_opto_turns_on_below_threshold(self):
        self._set(config.TURBOOPTO_ON_THRESHOLD - 100, voltage=1.2)
        r = self.p.read(auto_opto=True)
        self.assertTrue(r["opto_enabled"])
        self.assertEqual(gpio.pin_levels[config.GPIO_PIRANI_PIN], 1)

    def test_auto_opto_hysteresis_holds_between_thresholds(self):
        self._set(config.TURBOOPTO_ON_THRESHOLD - 100, voltage=1.2)
        self.p.read(auto_opto=True)
        self._set(12000, voltage=1.5)   # between ON (10500) and OFF (15000)
        r = self.p.read(auto_opto=True)
        self.assertTrue(r["opto_enabled"], "opto chattered inside hysteresis band")

    def test_auto_opto_turns_off_above_threshold(self):
        self._set(config.TURBOOPTO_ON_THRESHOLD - 100, voltage=1.2)
        self.p.read(auto_opto=True)
        self._set(config.TURBOOPTO_OFF_THRESHOLD + 100, voltage=2.0)
        r = self.p.read(auto_opto=True)
        self.assertFalse(r["opto_enabled"])
        self.assertEqual(gpio.pin_levels[config.GPIO_PIRANI_PIN], 0)

    def test_read_without_auto_opto_never_touches_pin(self):
        self._set(100, voltage=0.1)   # far below ON threshold
        r = self.p.read(auto_opto=False)
        self.assertFalse(r["opto_enabled"])
        self.assertEqual(gpio.pin_levels[config.GPIO_PIRANI_PIN], 0)

    def test_manual_set_opto(self):
        self.p.set_opto(True)
        self.assertEqual(gpio.pin_levels[config.GPIO_PIRANI_PIN], 1)
        self.p.set_opto(False)
        self.assertEqual(gpio.pin_levels[config.GPIO_PIRANI_PIN], 0)

    def test_shutdown_drives_opto_low(self):
        self.p.set_opto(True)
        self.p.shutdown()
        self.assertEqual(gpio.pin_levels[config.GPIO_PIRANI_PIN], 0)
        self.assertFalse(self.p.opto_enabled)


# ════════════════════════════════════════════════════════
#  Integration: full virtual process cycle
# ════════════════════════════════════════════════════════
class TestFullProcessCycle(unittest.TestCase):
    """Drives the state machine + MFC PID through a complete run against a
    toy chamber model: pump-down, argon flush to ignition pressure, plasma
    confirm, sputter, vent — asserting every expected state along the way."""

    def setUp(self):
        # deterministic clock: PID dt must equal the simulated tick, not
        # wall-clock microseconds (which would blow up the derivative term)
        self._t = [1000.0]
        self._orig_time = mfc_control.time.time
        mfc_control.time.time = lambda: self._t[0]

    def tearDown(self):
        mfc_control.time.time = self._orig_time

    def test_virtual_run(self):
        gpio.reset()
        sm = SputterStateMachine()
        ads = FakeADS1115()
        i2c = FakeI2C(devices=[0x60])
        mfc = MFCController(ads, i2c=i2c)
        cutoffs = []
        sm.on_transition_callback = (
            lambda old, new: cutoffs.append(new) if new in ("IDLE", "VENTING") else None
        )

        pressure = 999.0   # mbar, atmosphere
        visited = ["IDLE"]

        def v():
            return mbar_to_v(pressure)

        def note():
            if sm.state != visited[-1]:
                visited.append(sm.state)

        # ── pump down ─────────────────────────────
        for _ in range(200):
            pressure = max(pressure * 0.85, 0.002)   # pump pulls down
            if sm.state == "IDLE" and v() <= config.IDLE_PRESSURE_MAX_VOLTAGE:
                sm.transition("PUMP_DOWN", pirani_voltage=v())
            opto_on = v() <= 1.3   # transition-based opto fires below 1.3V
            sm.update(v(), opto_enabled=opto_on)
            note()
            if sm.state == "READY":
                break
        self.assertEqual(sm.state, "READY", f"never reached READY (p={pressure:.4g})")

        # ── argon flush: PID raises pressure to 0.09 mbar ──
        self.assertTrue(sm.transition("ARGON_FLUSH"))
        note()
        mfc.set_flow(config.ARGON_FLUSH_FLOW_SETPOINT)
        for _ in range(500):
            self._t[0] += config.POLLING_INTERVAL   # advance the fake clock one tick
            # toy chamber: flow raises pressure, pump fights it
            pressure += mfc.flow_target * 1e-5
            pressure = max(pressure * 0.97, 0.002)
            mfc.pressure_control_step(v(), config.ARGON_FLUSH_TARGET_VOLTAGE)
            sm.update(v(), opto_enabled=True)
            if v() >= config.ARGON_FLUSH_TARGET_VOLTAGE:
                sm.transition("PLASMA_IGNITING")   # _poll owns this in real code
                break
        note()
        self.assertEqual(sm.state, "PLASMA_IGNITING",
                         f"flush never hit ignition pressure (p={pressure:.4g}, "
                         f"flow={mfc.flow_target:.1f})")

        # ── operator confirms plasma, starts sputter ──
        self.assertTrue(sm.transition("SPUTTER_READY")); note()
        self.assertTrue(sm.transition("SPUTTERING")); note()

        # ── stop sputter -> vent -> atmosphere ──
        self.assertTrue(sm.transition("VENTING")); note()
        self.assertIn("VENTING", cutoffs, "interlock callback missed VENTING")
        for _ in range(200):
            pressure = min(pressure * 2.0, 999.0)
            sm.update(v())
            if sm.state == "IDLE":
                break
        note()
        self.assertEqual(sm.state, "IDLE")
        self.assertIn("IDLE", cutoffs, "interlock callback missed IDLE")

        self.assertEqual(visited, ["IDLE", "PUMP_DOWN", "READY", "ARGON_FLUSH",
                                   "PLASMA_IGNITING", "SPUTTER_READY",
                                   "SPUTTERING", "VENTING", "IDLE"])

    def test_turbo_valve_latch_logic(self):
        """Replicates the GPIO22 valve rule from _poll(): closed everywhere,
        latched open above threshold during VENTING only."""
        def valve_should_open(state, mbar, currently_open):
            if state == "VENTING":
                if not currently_open and mbar > config.TURBO_VALVE_OPEN_MBAR:
                    return True
                return currently_open
            return False

        # closed in all non-venting states regardless of pressure
        for state in ALL_STATES:
            if state == "VENTING":
                continue
            self.assertFalse(valve_should_open(state, 999, False))
        # venting below threshold: still closed
        self.assertFalse(valve_should_open("VENTING", 0.005, False))
        # venting crosses threshold: opens, and latches
        opened = valve_should_open("VENTING", 0.02, False)
        self.assertTrue(opened)
        self.assertTrue(valve_should_open("VENTING", 0.005, opened),
                        "latch must hold through noise dips")
        # leaving venting: closes
        self.assertFalse(valve_should_open("IDLE", 999, True))


if __name__ == "__main__":
    unittest.main(verbosity=2)
