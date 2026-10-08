"""
Hardware-free test suite for the Sputter Vacuum Controller.

Run from the repo root on any machine (no Pi, no sensors, no I2C needed):

    python3 -m unittest discover tests/unit -v

Fake hardware modules are installed before the controller code is imported,
so drivers/pirani.py / drivers/mfc_control.py / state_machine.py run exactly as written.
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
from drivers import mfc_control
from drivers import pirani as pirani_mod
from drivers.mfc_control import MFCController
from drivers.pirani import PiraniController
from drivers.turbo_rpm import TurboRPMController
from drivers.pzem_meter import PZEMController
from sim.sim_hardware import ChamberSim
from ui.web_ui import WebUI
from fakes import FakeADS1115, FakeI2C, FakeSerial, gpio

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
        # Increasing voltage = increasing pressure: deep vacuum (PUMP_DOWN
        # complete) < flush target < safe pump-down-start ceiling < atmosphere
        # (VENTING complete, must be the highest — it's a real pressure, not
        # a vacuum gate)
        self.assertLess(config.PUMP_DOWN_COMPLETE_VOLTAGE,
                        config.ARGON_FLUSH_TARGET_VOLTAGE)
        self.assertLess(config.ARGON_FLUSH_TARGET_VOLTAGE,
                        config.IDLE_PRESSURE_MAX_VOLTAGE)
        self.assertLess(config.IDLE_PRESSURE_MAX_VOLTAGE,
                        config.VENTING_COMPLETE_VOLTAGE)

    def test_venting_complete_near_atmosphere(self):
        # Must represent real atmosphere, not a partial vacuum (the bug this
        # guards against: VENTING_COMPLETE_VOLTAGE was 2.5V, only ~5 mbar)
        self.assertGreaterEqual(v_to_mbar(config.VENTING_COMPLETE_VOLTAGE), 100)

    def test_gpio_pins_distinct(self):
        pins = [config.GPIO_PIRANI_PIN,
                config.GPIO_MFC_VALVE_CLOSE_PIN,
                config.GPIO_TURBO_VALVE_PIN]
        self.assertEqual(len(pins), len(set(pins)), "GPIO pin collision")

    def test_pins_avoid_i2c_bus(self):
        # Hardware I2C1 reserves GPIO2/3.
        forbidden = {2, 3}
        for pin in (config.GPIO_PIRANI_PIN,
                    config.GPIO_MFC_VALVE_CLOSE_PIN,
                    config.GPIO_TURBO_VALVE_PIN):
            self.assertNotIn(pin, forbidden, f"GPIO{pin} is reserved for I2C")

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

    def test_i2c_bus_is_hardware_bus(self):
        self.assertEqual(config.I2C_BUS_NUMBER, 1)

    def test_turbo_valve_threshold(self):
        self.assertGreater(config.TURBO_VALVE_OPEN_RPM_MAX, 0.0)
        self.assertLess(config.TURBO_VALVE_OPEN_RPM_MAX, config.TURBO_RPM_FULL_SCALE)

    def test_turbo_valve_confirm_samples(self):
        # >= 2 or a single corrupted I2C read can open the valve
        self.assertGreaterEqual(config.TURBO_VALVE_CONFIRM_SAMPLES, 2)
        # keep the open delay under 2 s at the configured polling rate
        self.assertLessEqual(
            config.TURBO_VALVE_CONFIRM_SAMPLES * config.POLLING_INTERVAL, 2.0)

    def test_turbo_rpm_stall_threshold_in_range(self):
        self.assertGreater(config.TURBO_RPM_STALL_THRESHOLD, 0.0)
        self.assertLess(config.TURBO_RPM_STALL_THRESHOLD, config.TURBO_RPM_FULL_SCALE)

    def test_venting_pump_off_prompt_below_stall_threshold(self):
        # The venting prompt is an operator cue for "nearly stopped", not a
        # fault threshold -- it should fire at a lower RPM than the fault-
        # detection bar, not above/equal to it.
        self.assertGreater(config.VENTING_PUMP_OFF_PROMPT_RPM, 0.0)
        self.assertLess(config.VENTING_PUMP_OFF_PROMPT_RPM,
                        config.TURBO_RPM_STALL_THRESHOLD)

    def test_dac_saturation_thresholds_sane(self):
        self.assertGreater(config.DAC_SATURATION_MARGIN_V, 0.0)
        self.assertLess(config.DAC_SATURATION_MARGIN_V, config.ARGON_DAC_VREF)
        self.assertGreater(config.DAC_SATURATION_FLOW_FRACTION, 0.0)
        self.assertLessEqual(config.DAC_SATURATION_FLOW_FRACTION, 1.0)

    def test_pzem_plasma_hysteresis_ordered(self):
        self.assertLess(config.PZEM_PLASMA_CURRENT_OFF_A, config.PZEM_PLASMA_CURRENT_ON_A)

    def test_pzem_confirm_samples_sane(self):
        # >= 2 or a single Modbus CRC glitch could flip plasma_detected
        self.assertGreaterEqual(config.PZEM_PLASMA_CONFIRM_SAMPLES, 2)

    def test_pzem_frequency_band_sane(self):
        self.assertLess(config.PZEM_FREQUENCY_MIN, config.PZEM_FREQUENCY_MAX)


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


# ════════════════════════════════════════════════════════
#  Turbo valve debounced latch (from main.py source)
# ════════════════════════════════════════════════════════
class TestTurboValveStep(unittest.TestCase):
    """The valve must never open on readings that don't stay at/below
    TURBO_VALVE_OPEN_RPM_MAX for TURBO_VALVE_CONFIRM_SAMPLES consecutive polls.
    (rpm_safe = rotor slow enough to admit gas; rpm_unsafe = still spinning
    too fast, valve must stay closed)."""

    def setUp(self):
        self.step = CAL["turbo_valve_step"]
        self.N = config.TURBO_VALVE_CONFIRM_SAMPLES
        self.rpm_safe   = config.TURBO_VALVE_OPEN_RPM_MAX / 2
        self.rpm_unsafe = config.TURBO_VALVE_OPEN_RPM_MAX * 2

    def _run(self, venting_rpm_seq, valve_open=False, ticks=0):
        for venting, rpm in venting_rpm_seq:
            valve_open, ticks = self.step(venting, rpm, valve_open, ticks)
        return valve_open, ticks

    def test_single_glitch_does_not_open(self):
        # one corrupted zero/garbage-low sample amid a still-spinning rotor: stays closed
        seq = ([(True, self.rpm_unsafe)] * 5 + [(True, 0)]
               + [(True, self.rpm_unsafe)] * 5)
        valve_open, ticks = self._run(seq)
        self.assertFalse(valve_open)
        self.assertEqual(ticks, 0)

    def test_repeated_isolated_glitches_do_not_open(self):
        seq = [(True, self.rpm_unsafe), (True, 0)] * (self.N * 3)
        valve_open, _ = self._run(seq)
        self.assertFalse(valve_open)

    def test_opens_after_confirm_samples(self):
        seq = [(True, self.rpm_safe)] * self.N
        valve_open, _ = self._run(seq)
        self.assertTrue(valve_open)

    def test_does_not_open_one_sample_early(self):
        seq = [(True, self.rpm_safe)] * (self.N - 1)
        valve_open, _ = self._run(seq)
        self.assertFalse(valve_open)

    def test_above_threshold_resets_count(self):
        seq = ([(True, self.rpm_safe)] * (self.N - 1)
               + [(True, self.rpm_unsafe)]
               + [(True, self.rpm_safe)] * (self.N - 1))
        valve_open, _ = self._run(seq)
        self.assertFalse(valve_open)

    def test_never_opens_outside_venting(self):
        seq = [(False, 0)] * (self.N * 2)
        valve_open, ticks = self._run(seq)
        self.assertFalse(valve_open)
        self.assertEqual(ticks, 0)

    def test_latches_open_despite_spikes(self):
        # once open, a momentary RPM spike must not chatter the relay
        valve_open, ticks = self._run([(True, self.rpm_unsafe)], valve_open=True)
        self.assertTrue(valve_open)

    def test_leaving_venting_closes_and_resets(self):
        valve_open, ticks = self._run([(False, self.rpm_safe)],
                                      valve_open=True, ticks=self.N)
        self.assertFalse(valve_open)
        self.assertEqual(ticks, 0)

    def test_estop_mid_count_does_not_carry_into_next_vent(self):
        # N-1 safe reads, E-STOP to IDLE, vent again: one more safe read
        # must not be enough to open
        seq = ([(True, self.rpm_safe)] * (self.N - 1)
               + [(False, self.rpm_unsafe)]
               + [(True, self.rpm_safe)])
        valve_open, _ = self._run(seq)
        self.assertFalse(valve_open)

    def test_exactly_at_threshold_counts_as_safe(self):
        seq = [(True, config.TURBO_VALVE_OPEN_RPM_MAX)] * self.N
        valve_open, _ = self._run(seq)
        self.assertTrue(valve_open)


# ════════════════════════════════════════════════════════
#  Cross-sensor consistency checks (from main.py source)
# ════════════════════════════════════════════════════════
T0 = 1_700_000_000.0  # arbitrary fixed epoch so tests don't depend on wall clock


class TestTurboSpinupCheck(unittest.TestCase):
    def setUp(self):
        self.check = CAL["turbo_spinup_check"]
        self.grace = config.TURBO_SPINUP_GRACE_SECONDS
        self.stall_rpm = config.TURBO_RPM_STALL_THRESHOLD / 2
        self.spinning_rpm = config.TURBO_RPM_STALL_THRESHOLD * 10

    def test_opto_off_never_flags(self):
        flagged, since = self.check(False, 0, None, T0)
        self.assertFalse(flagged)
        self.assertIsNone(since)

    def test_spinning_normally_never_flags(self):
        flagged, since = self.check(True, self.spinning_rpm, T0, T0 + self.grace * 10)
        self.assertFalse(flagged)

    def test_does_not_flag_before_grace_elapses(self):
        flagged, since = self.check(True, self.stall_rpm, T0, T0 + self.grace - 1)
        self.assertFalse(flagged)

    def test_flags_after_grace_elapses_while_stalled(self):
        flagged, since = self.check(True, self.stall_rpm, T0, T0 + self.grace)
        self.assertTrue(flagged)

    def test_on_since_latched_from_first_tick_opto_turned_on(self):
        _, since = self.check(True, self.stall_rpm, None, T0)
        self.assertEqual(since, T0)
        # a later tick must not reset the clock while opto stays on
        _, since2 = self.check(True, self.stall_rpm, since, T0 + 1)
        self.assertEqual(since2, T0)

    def test_opto_turning_off_resets_clock(self):
        flagged, since = self.check(False, self.stall_rpm, T0, T0 + self.grace)
        self.assertFalse(flagged)
        self.assertIsNone(since)


class TestTurboRpmDropCheck(unittest.TestCase):
    """Distinct from turbo_spinup_check: catches a healthy turbo losing
    speed later, not just a turbo that never spun up."""

    def setUp(self):
        self.check = CAL["turbo_rpm_drop_check"]
        self.grace = config.TURBO_RPM_DROP_GRACE_SECONDS
        self.healthy_rpm = config.TURBO_RPM_STALL_THRESHOLD * 2
        self.dropped_rpm = config.TURBO_RPM_STALL_THRESHOLD / 2

    def test_opto_off_never_flags_and_resets(self):
        flagged, healthy, since = self.check(False, 0, True, T0, T0 + self.grace)
        self.assertFalse(flagged)
        self.assertFalse(healthy)
        self.assertIsNone(since)

    def test_never_confirmed_healthy_never_flags(self):
        # low RPM before ever crossing the stall threshold is turbo_spinup_check's
        # job, not this one's -- must not flag here even past the grace period
        flagged, healthy, since = self.check(True, self.dropped_rpm, False, T0, T0 + self.grace * 10)
        self.assertFalse(flagged)
        self.assertFalse(healthy)

    def test_crossing_threshold_confirms_healthy_and_never_flags(self):
        flagged, healthy, since = self.check(True, self.healthy_rpm, False, None, T0)
        self.assertFalse(flagged)
        self.assertTrue(healthy)
        self.assertIsNone(since)

    def test_does_not_flag_before_grace_elapses(self):
        flagged, healthy, since = self.check(
            True, self.dropped_rpm, True, T0, T0 + self.grace - 1)
        self.assertFalse(flagged)

    def test_flags_after_grace_elapses_while_dropped(self):
        flagged, healthy, since = self.check(
            True, self.dropped_rpm, True, T0, T0 + self.grace)
        self.assertTrue(flagged)

    def test_recovering_above_threshold_resets_clock(self):
        flagged, healthy, since = self.check(
            True, self.healthy_rpm, True, T0, T0 + self.grace)
        self.assertFalse(flagged)
        self.assertTrue(healthy)
        self.assertIsNone(since)

    def test_dropped_since_latched_from_first_drop_tick(self):
        _, _, since = self.check(True, self.dropped_rpm, True, None, T0)
        self.assertEqual(since, T0)
        _, _, since2 = self.check(True, self.dropped_rpm, True, since, T0 + 1)
        self.assertEqual(since2, T0)


class TestMfcFlowCheck(unittest.TestCase):
    def setUp(self):
        self.check = CAL["mfc_flow_check"]
        self.grace = config.MFC_FLOW_GRACE_SECONDS
        self.commanded = config.MFC_FLOW_MIN_COMMANDED_SCCM * 5

    def test_small_commanded_flow_never_flags(self):
        # below MFC_FLOW_MIN_COMMANDED_SCCM: PID settling noise near zero, not a fault
        tiny = config.MFC_FLOW_MIN_COMMANDED_SCCM / 2
        flagged, since = self.check(tiny, 0.0, T0, T0 + self.grace * 10)
        self.assertFalse(flagged)
        self.assertIsNone(since)

    def test_responding_normally_never_flags(self):
        flagged, since = self.check(self.commanded, self.commanded * 0.9, None, T0)
        self.assertFalse(flagged)

    def test_does_not_flag_before_grace_elapses(self):
        flagged, since = self.check(self.commanded, 0.0, T0, T0 + self.grace - 1)
        self.assertFalse(flagged)

    def test_flags_after_grace_elapses_while_stalled(self):
        flagged, since = self.check(self.commanded, 0.0, T0, T0 + self.grace)
        self.assertTrue(flagged)

    def test_recovering_resets_clock(self):
        flagged, since = self.check(self.commanded, self.commanded * 0.9, T0, T0 + self.grace)
        self.assertFalse(flagged)
        self.assertIsNone(since)


class TestDacSaturationFlowCheck(unittest.TestCase):
    """DAC pegged at its ceiling while flow doesn't respond -- a stronger
    fault signal than mfc_flow_check's target/measured mismatch."""

    def setUp(self):
        self.check = CAL["dac_saturation_flow_check"]
        self.grace = config.DAC_SATURATION_GRACE_SECONDS
        self.ceiling = config.ARGON_DAC_VREF
        self.responding_flow = config.MFC_FULL_SCALE * config.DAC_SATURATION_FLOW_FRACTION * 1.5
        self.stalled_flow = config.MFC_FULL_SCALE * config.DAC_SATURATION_FLOW_FRACTION * 0.1

    def test_not_saturated_never_flags(self):
        mid_voltage = self.ceiling / 2
        flagged, since = self.check(mid_voltage, 0.0, T0, T0 + self.grace * 10)
        self.assertFalse(flagged)
        self.assertIsNone(since)

    def test_saturated_and_responding_never_flags(self):
        flagged, since = self.check(self.ceiling, self.responding_flow, None, T0)
        self.assertFalse(flagged)

    def test_within_margin_of_ceiling_counts_as_saturated(self):
        near_ceiling = self.ceiling - config.DAC_SATURATION_MARGIN_V / 2
        flagged, since = self.check(near_ceiling, self.stalled_flow, T0, T0 + self.grace)
        self.assertTrue(flagged)

    def test_does_not_flag_before_grace_elapses(self):
        flagged, since = self.check(self.ceiling, self.stalled_flow, T0, T0 + self.grace - 1)
        self.assertFalse(flagged)

    def test_flags_after_grace_elapses_while_stalled(self):
        flagged, since = self.check(self.ceiling, self.stalled_flow, T0, T0 + self.grace)
        self.assertTrue(flagged)

    def test_recovering_resets_clock(self):
        flagged, since = self.check(self.ceiling, self.responding_flow, T0, T0 + self.grace)
        self.assertFalse(flagged)
        self.assertIsNone(since)


class TestPressureConvergenceCheck(unittest.TestCase):
    def setUp(self):
        self.check = CAL["pressure_convergence_check"]
        self.grace = config.PRESSURE_CONVERGENCE_GRACE_SECONDS
        self.tol = config.PRESSURE_CONVERGENCE_TOLERANCE_V

    def test_within_tolerance_never_flags(self):
        flagged, since = self.check(1.287, 1.287 + self.tol / 2, T0, T0 + self.grace * 10)
        self.assertFalse(flagged)
        self.assertIsNone(since)

    def test_does_not_flag_before_grace_elapses(self):
        flagged, since = self.check(1.287, 1.287 + self.tol * 2, T0, T0 + self.grace - 1)
        self.assertFalse(flagged)

    def test_flags_after_grace_elapses_while_diverging(self):
        flagged, since = self.check(1.287, 1.287 + self.tol * 2, T0, T0 + self.grace)
        self.assertTrue(flagged)

    def test_converging_resets_clock(self):
        flagged, since = self.check(1.287, 1.287 + self.tol / 2, T0, T0 + self.grace)
        self.assertFalse(flagged)
        self.assertIsNone(since)

    def test_symmetric_over_under_shoot(self):
        # error is |current - target|; overshoot must flag the same as undershoot
        flagged_over, _ = self.check(1.287 + self.tol * 2, 1.287, T0, T0 + self.grace)
        flagged_under, _ = self.check(1.287 - self.tol * 2, 1.287, T0, T0 + self.grace)
        self.assertTrue(flagged_over)
        self.assertTrue(flagged_under)


class TestSensorRangeCheck(unittest.TestCase):
    """Global gate: physically impossible ADC-derived voltages (disconnected/
    shorted/miswired sensor), debounced like every other hardware check here."""

    def setUp(self):
        self.check = CAL["sensor_range_check"]
        self.N = config.SENSOR_RANGE_CONFIRM_SAMPLES
        self.sane = (1.2, 0.5, 2.0)
        self.insane = (config.SANE_VOLTAGE_MAX * 3, 0.5, 2.0)

    def test_sane_readings_never_flag(self):
        bad_ticks = 0
        for _ in range(self.N * 3):
            flagged, bad_ticks = self.check(self.sane, bad_ticks)
        self.assertFalse(flagged)

    def test_below_min_flags_after_confirm_samples(self):
        below_min = (config.SANE_VOLTAGE_MIN - 1.0, 0.5, 2.0)
        bad_ticks = 0
        for _ in range(self.N):
            flagged, bad_ticks = self.check(below_min, bad_ticks)
        self.assertTrue(flagged)

    def test_above_max_flags_after_confirm_samples(self):
        bad_ticks = 0
        for _ in range(self.N):
            flagged, bad_ticks = self.check(self.insane, bad_ticks)
        self.assertTrue(flagged)

    def test_does_not_flag_one_sample_early(self):
        bad_ticks = 0
        for _ in range(self.N - 1):
            flagged, bad_ticks = self.check(self.insane, bad_ticks)
        self.assertFalse(flagged)

    def test_single_glitch_does_not_flag(self):
        # one corrupted reading amid sane ones must not trip the gate
        flagged, bad_ticks = False, 0
        seq = [self.sane] * 5 + [self.insane] + [self.sane] * 5
        for voltages in seq:
            flagged, bad_ticks = self.check(voltages, bad_ticks)
        self.assertFalse(flagged)

    def test_recovering_resets_count(self):
        bad_ticks = 0
        for _ in range(self.N - 1):
            _, bad_ticks = self.check(self.insane, bad_ticks)
        flagged, bad_ticks = self.check(self.sane, bad_ticks)
        self.assertFalse(flagged)
        self.assertEqual(bad_ticks, 0)


class TestPlasmaDetectStep(unittest.TestCase):
    """Hysteresis + debounce latch for PZEM current-draw plasma detection
    (from main.py source) -- same shape as the turbo opto ON/OFF hysteresis
    and the turbo valve confirm-samples debounce, combined."""

    def setUp(self):
        self.step = CAL["plasma_detect_step"]
        self.on_a = config.PZEM_PLASMA_CURRENT_ON_A
        self.off_a = config.PZEM_PLASMA_CURRENT_OFF_A
        self.N = config.PZEM_PLASMA_CONFIRM_SAMPLES

    def _run(self, currents, detected=False, ticks=0):
        for a in currents:
            detected, ticks = self.step(a, detected, ticks)
        return detected, ticks

    def test_below_off_never_detects(self):
        detected, _ = self._run([self.off_a - 0.01] * (self.N * 3))
        self.assertFalse(detected)

    def test_detects_after_confirm_samples_at_on(self):
        detected, _ = self._run([self.on_a] * self.N)
        self.assertTrue(detected)

    def test_does_not_detect_one_sample_early(self):
        detected, _ = self._run([self.on_a] * (self.N - 1))
        self.assertFalse(detected)

    def test_single_glitch_does_not_detect(self):
        seq = [self.off_a] * 5 + [self.on_a] + [self.off_a] * 5
        detected, _ = self._run(seq)
        self.assertFalse(detected)

    def test_dip_into_dead_band_resets_confirm_count(self):
        mid = (self.on_a + self.off_a) / 2
        seq = [self.on_a] * (self.N - 1) + [mid] + [self.on_a] * (self.N - 1)
        detected, _ = self._run(seq)
        self.assertFalse(detected)

    def test_falls_immediately_below_off_threshold(self):
        detected, ticks = self._run([self.on_a] * self.N)
        self.assertTrue(detected)
        detected, ticks = self.step(self.off_a, detected, ticks)
        self.assertFalse(detected)

    def test_latches_through_dead_band_once_detected(self):
        detected, ticks = self._run([self.on_a] * self.N)
        mid = (self.on_a + self.off_a) / 2
        detected, ticks = self.step(mid, detected, ticks)
        self.assertTrue(detected, "dead band must hold the latched state")

    def test_exactly_at_on_threshold_counts(self):
        detected, _ = self._run([self.on_a] * self.N)
        self.assertTrue(detected)

    def test_exactly_at_off_threshold_clears(self):
        detected, ticks = self._run([self.on_a] * self.N)
        detected, _ = self.step(self.off_a, detected, ticks)
        self.assertFalse(detected)


class TestPzemPlasmaAbsentCheck(unittest.TestCase):
    """SPUTTER_READY/SPUTTERING assume plasma is already lit; PLASMA_IGNITING
    is deliberately excluded (owned by PLASMA_IGNITION_TIMEOUT instead)."""

    def setUp(self):
        self.check = CAL["pzem_plasma_absent_check"]
        self.grace = config.PZEM_PLASMA_ABSENT_GRACE_SECONDS

    def test_plasma_igniting_never_flags(self):
        flagged, since = self.check("PLASMA_IGNITING", False, T0, T0 + self.grace * 10)
        self.assertFalse(flagged)
        self.assertIsNone(since)

    def test_non_plasma_states_never_flag(self):
        for state in ("IDLE", "PUMP_DOWN", "READY", "ARGON_FLUSH", "VENTING"):
            flagged, since = self.check(state, False, T0, T0 + self.grace * 10)
            self.assertFalse(flagged, f"{state} must not flag")

    def test_detected_never_flags(self):
        flagged, since = self.check("SPUTTERING", True, None, T0)
        self.assertFalse(flagged)
        self.assertIsNone(since)

    def test_does_not_flag_before_grace_elapses(self):
        flagged, since = self.check("SPUTTER_READY", False, T0, T0 + self.grace - 1)
        self.assertFalse(flagged)

    def test_flags_after_grace_elapses(self):
        flagged, since = self.check("SPUTTER_READY", False, T0, T0 + self.grace)
        self.assertTrue(flagged)

    def test_both_expected_states_covered(self):
        for state in ("SPUTTER_READY", "SPUTTERING"):
            flagged, since = self.check(state, False, T0, T0 + self.grace)
            self.assertTrue(flagged, f"{state} must flag when plasma absent")

    def test_recovering_resets_clock(self):
        flagged, since = self.check("SPUTTERING", True, T0, T0 + self.grace)
        self.assertFalse(flagged)
        self.assertIsNone(since)


class TestPzemPowerUnexpectedCheck(unittest.TestCase):
    """Plasma-level current draw while nowhere near attempting ignition —
    stuck relay, live supply, or wiring fault."""

    def setUp(self):
        self.check = CAL["pzem_power_unexpected_check"]
        self.grace = config.PZEM_POWER_UNEXPECTED_GRACE_SECONDS

    def test_expected_states_never_flag(self):
        for state in ("PLASMA_IGNITING", "SPUTTER_READY", "SPUTTERING"):
            flagged, since = self.check(state, True, T0, T0 + self.grace * 10)
            self.assertFalse(flagged, f"{state} must not flag")

    def test_not_detected_never_flags(self):
        flagged, since = self.check("IDLE", False, None, T0)
        self.assertFalse(flagged)
        self.assertIsNone(since)

    def test_does_not_flag_before_grace_elapses(self):
        flagged, since = self.check("READY", True, T0, T0 + self.grace - 1)
        self.assertFalse(flagged)

    def test_flags_after_grace_elapses(self):
        flagged, since = self.check("READY", True, T0, T0 + self.grace)
        self.assertTrue(flagged)

    def test_all_unexpected_states_covered(self):
        for state in ("IDLE", "PUMP_DOWN", "READY", "ARGON_FLUSH", "VENTING"):
            flagged, since = self.check(state, True, T0, T0 + self.grace)
            self.assertTrue(flagged, f"{state} must flag on unexpected power")

    def test_recovering_resets_clock(self):
        flagged, since = self.check("IDLE", False, T0, T0 + self.grace)
        self.assertFalse(flagged)
        self.assertIsNone(since)


# ════════════════════════════════════════════════════════
#  Vent-complete confirmation gate (from main.py source)
# ════════════════════════════════════════════════════════
class TestVentCompleteReady(unittest.TestCase):
    """VENTING -> IDLE requires BOTH a latched atmospheric-pressure
    indication AND explicit operator confirmation the primary/roughing
    pump is off -- neither alone is ever enough (pending-alone let the
    turbo inlet valve reclose before the operator could react; the raw-
    voltage version of this check that predated the `pending` latch made
    the Confirm Pump Off button flicker on any reading that dipped back
    below the threshold)."""

    def setUp(self):
        self.ready = CAL["vent_complete_ready"]

    def test_not_pending_never_ready_even_if_confirmed(self):
        self.assertFalse(self.ready(False, True))

    def test_pending_not_ready_without_confirmation(self):
        self.assertFalse(self.ready(True, False))

    def test_pending_and_confirmed_is_ready(self):
        self.assertTrue(self.ready(True, True))

    def test_neither_not_ready(self):
        self.assertFalse(self.ready(False, False))


# ════════════════════════════════════════════════════════
#  PZEM energy meter driver (Modbus-RTU over a fake serial port)
# ════════════════════════════════════════════════════════
def _crc16(data):
    """Independent Modbus CRC16 implementation used only to build test
    fixtures -- deliberately not shared with pzem_meter's own _crc16_modbus
    so a bug in that implementation wouldn't silently pass here too."""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def _pzem_response_frame(slave_addr, regs):
    payload = bytes([slave_addr, 0x04, len(regs) * 2])
    for r in regs:
        payload += bytes([(r >> 8) & 0xFF, r & 0xFF])
    crc = _crc16(payload)
    return payload + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


# voltage=230.1V, current=1.234A, power=326.5W, energy=12345Wh, freq=50.0Hz, pf=0.98, no alarm
_SAMPLE_REGS = [2301, 1234, 0, 3265, 0, 12345, 0, 500, 98, 0]


class TestPZEMController(unittest.TestCase):
    def setUp(self):
        FakeSerial.FAIL_PORTS = set()

    def tearDown(self):
        FakeSerial.FAIL_PORTS = set()

    def test_port_unavailable_is_not_ready_and_does_not_raise(self):
        FakeSerial.FAIL_PORTS = {"/dev/does-not-exist"}
        ctrl = PZEMController(port="/dev/does-not-exist")
        self.assertFalse(ctrl.ready)
        reading = ctrl.read()   # must not raise
        self.assertFalse(reading["ready"])
        self.assertEqual(reading["current"], 0.0)

    def test_valid_response_decodes_all_fields(self):
        ctrl = PZEMController(port="/dev/fake0")
        ctrl._ser.responses.append(_pzem_response_frame(ctrl.slave_addr, _SAMPLE_REGS))
        r = ctrl.read()
        self.assertTrue(r["ready"])
        self.assertAlmostEqual(r["voltage"], 230.1, places=3)
        self.assertAlmostEqual(r["current"], 1.234, places=3)
        self.assertAlmostEqual(r["power"], 326.5, places=3)
        self.assertEqual(r["energy"], 12345.0)
        self.assertAlmostEqual(r["frequency"], 50.0, places=3)
        self.assertAlmostEqual(r["power_factor"], 0.98, places=3)
        self.assertFalse(r["alarm"])
        self.assertTrue(ctrl.ready)

    def test_request_frame_is_well_formed(self):
        ctrl = PZEMController(port="/dev/fake0")
        ctrl._ser.responses.append(_pzem_response_frame(ctrl.slave_addr, _SAMPLE_REGS))
        ctrl.read()
        frame = ctrl._ser.writes[-1]
        self.assertEqual(frame[0], ctrl.slave_addr)
        self.assertEqual(frame[1], 0x04)
        self.assertEqual(frame[2:6], bytes([0x00, 0x00, 0x00, 0x0A]))
        self.assertEqual(_crc16(frame[:-2]), frame[-2] | (frame[-1] << 8))

    def test_timeout_is_not_ready_and_does_not_raise(self):
        ctrl = PZEMController(port="/dev/fake0")
        # no response queued -> FakeSerial.read() returns b"", simulating a timeout
        r = ctrl.read()
        self.assertFalse(r["ready"])
        self.assertEqual(r["voltage"], 0.0)

    def test_crc_mismatch_is_not_ready(self):
        ctrl = PZEMController(port="/dev/fake0")
        corrupt = bytearray(_pzem_response_frame(ctrl.slave_addr, _SAMPLE_REGS))
        corrupt[-1] ^= 0xFF
        ctrl._ser.responses.append(bytes(corrupt))
        r = ctrl.read()
        self.assertFalse(r["ready"])

    def test_bad_response_drops_connection_for_next_read(self):
        ctrl = PZEMController(port="/dev/fake0")
        ctrl._ser.responses.append(b"\x00\x01")   # too short to be a valid frame
        r = ctrl.read()
        self.assertFalse(r["ready"])
        self.assertIsNone(ctrl._ser, "must drop the connection so the next read() reopens it")

    def test_recovers_once_the_meter_is_plugged_in(self):
        FakeSerial.FAIL_PORTS = {"/dev/fake0"}
        ctrl = PZEMController(port="/dev/fake0")
        self.assertIsNone(ctrl._ser)
        self.assertFalse(ctrl.read()["ready"])

        FakeSerial.FAIL_PORTS = set()   # "device plugged in"
        ctrl._open()
        ctrl._ser.responses.append(_pzem_response_frame(ctrl.slave_addr, _SAMPLE_REGS))
        r = ctrl.read()
        self.assertTrue(r["ready"])


class TestConfigSanityFailures(unittest.TestCase):
    """Pure runtime self-test logic (config_sanity_failures) -- must agree
    with the real config.py (no failures) and must actually catch the
    exact bug class it exists to prevent (thresholds out of order)."""

    def setUp(self):
        self.check = CAL["config_sanity_failures"]

    def test_real_config_has_no_failures(self):
        self.assertEqual(self.check(), [])

    def test_catches_venting_threshold_regression(self):
        # Replicates the actual historical bug: VENTING_COMPLETE_VOLTAGE
        # below IDLE_PRESSURE_MAX_VOLTAGE instead of above it.
        bad_ns = fakes.load_main_slice(REPO_ROOT, overrides={
            "VENTING_COMPLETE_VOLTAGE": config.IDLE_PRESSURE_MAX_VOLTAGE - 0.1,
        })
        failures = bad_ns["config_sanity_failures"]()
        self.assertTrue(any("order" in f for f in failures))

    def test_catches_gpio_pin_collision(self):
        bad_ns = fakes.load_main_slice(REPO_ROOT, overrides={
            "GPIO_MFC_VALVE_CLOSE_PIN": config.GPIO_TURBO_VALVE_PIN,
        })
        failures = bad_ns["config_sanity_failures"]()
        self.assertTrue(any("collision" in f for f in failures))

    def test_catches_dead_pad_reuse(self):
        bad_ns = fakes.load_main_slice(REPO_ROOT, overrides={
            "GPIO_TURBO_VALVE_PIN": 4,  # dead pad from the 26V incident
        })
        failures = bad_ns["config_sanity_failures"]()
        self.assertTrue(any("dead/reserved" in f for f in failures))

    def test_catches_rpm_threshold_out_of_range(self):
        bad_ns = fakes.load_main_slice(REPO_ROOT, overrides={
            "TURBO_VALVE_OPEN_RPM_MAX": config.TURBO_RPM_FULL_SCALE * 2,
        })
        failures = bad_ns["config_sanity_failures"]()
        self.assertTrue(any("TURBO_VALVE_OPEN_RPM_MAX" in f for f in failures))

    def test_catches_pzem_hysteresis_inversion(self):
        bad_ns = fakes.load_main_slice(REPO_ROOT, overrides={
            "PZEM_PLASMA_CURRENT_OFF_A": config.PZEM_PLASMA_CURRENT_ON_A + 1.0,
        })
        failures = bad_ns["config_sanity_failures"]()
        self.assertTrue(any("PZEM_PLASMA_CURRENT_OFF_A" in f for f in failures))


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

    def test_venting_has_no_auto_complete(self):
        """VENTING -> IDLE is owned exclusively by _poll(), gated on operator
        confirmation that the primary/roughing pump is off (see
        vent_complete_ready() in main.py) -- reaching atmospheric pressure
        alone must never auto-complete it via sm.update(), the same
        reasoning as test_argon_flush_has_no_auto_ignition and
        test_plasma_confirm_is_operator_only."""
        self._force("VENTING")
        for v in (config.VENTING_COMPLETE_VOLTAGE - 0.1,
                  config.VENTING_COMPLETE_VOLTAGE,
                  config.VENTING_COMPLETE_VOLTAGE + 1.0):
            self.sm.update(v)
            self.assertEqual(self.sm.state, "VENTING",
                             f"auto-completed vent at {v}V!")
        self.assertTrue(self.sm.transition("IDLE"))

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
#  Turbo pump RPM (tach on A2, display only)
# ════════════════════════════════════════════════════════
class TestTurboRPMController(unittest.TestCase):
    def setUp(self):
        gpio.reset()
        self.ads = FakeADS1115()
        self.r = TurboRPMController(self.ads)

    def _set(self, voltage):
        self.ads.set_channel(config.ADC_CHANNEL_TURBO_RPM, voltage=voltage)

    def test_zero_volts_is_zero_rpm(self):
        self._set(0.0)
        self.assertEqual(self.r.read()["rpm"], 0.0)

    def test_full_scale_voltage_is_full_scale_rpm(self):
        self._set(config.TURBO_RPM_VOLTAGE_FULL_SCALE)
        self.assertAlmostEqual(self.r.read()["rpm"],
                               config.TURBO_RPM_FULL_SCALE, places=1)

    def test_half_scale_voltage_is_half_scale_rpm(self):
        self._set(config.TURBO_RPM_VOLTAGE_FULL_SCALE / 2)
        self.assertAlmostEqual(self.r.read()["rpm"],
                               config.TURBO_RPM_FULL_SCALE / 2, places=1)

    def test_overrange_voltage_clamps_at_full_scale(self):
        self._set(config.TURBO_RPM_VOLTAGE_FULL_SCALE + 1.0)
        self.assertEqual(self.r.read()["rpm"], config.TURBO_RPM_FULL_SCALE)

    def test_negative_voltage_clamps_at_zero(self):
        self._set(-0.5)
        self.assertEqual(self.r.read()["rpm"], 0.0)


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
            if v() >= config.VENTING_COMPLETE_VOLTAGE:
                sm.transition("IDLE")   # _poll() + operator confirmation own this in real code
                break
        note()
        self.assertEqual(sm.state, "IDLE")
        self.assertIn("IDLE", cutoffs, "interlock callback missed IDLE")

        self.assertEqual(visited, ["IDLE", "PUMP_DOWN", "READY", "ARGON_FLUSH",
                                   "PLASMA_IGNITING", "SPUTTER_READY",
                                   "SPUTTERING", "VENTING", "IDLE"])

    def test_turbo_valve_latch_logic(self):
        """Drives the real GPIO22 valve rule (turbo_valve_step): closed
        everywhere, latched open during VENTING only after the turbo rotor
        has sustained a drop to/below TURBO_VALVE_OPEN_RPM_MAX."""
        step = CAL["turbo_valve_step"]
        n = config.TURBO_VALVE_CONFIRM_SAMPLES
        rpm_unsafe = config.TURBO_VALVE_OPEN_RPM_MAX * 2  # still spinning fast
        rpm_safe   = config.TURBO_VALVE_OPEN_RPM_MAX / 2  # slow enough to open

        # closed in all non-venting states regardless of rotor speed
        for state in ALL_STATES:
            if state == "VENTING":
                continue
            self.assertEqual(step(state == "VENTING", rpm_safe, False, 0),
                             (False, 0))
        # venting, rotor still fast: still closed
        self.assertEqual(step(True, rpm_unsafe, False, 0), (False, 0))
        # venting, rotor sustains a drop to/below threshold: opens, and latches
        opened, ticks = False, 0
        for _ in range(n):
            opened, ticks = step(True, rpm_safe, opened, ticks)
        self.assertTrue(opened)
        opened, _ = step(True, rpm_unsafe, opened, 0)
        self.assertTrue(opened, "latch must hold through noise spikes")
        # leaving venting: closes
        self.assertEqual(step(False, rpm_unsafe, True, 0), (False, 0))


# ════════════════════════════════════════════════════════
#  Simulation mode (sim_hardware.ChamberSim) — the physics model behind
#  SPUTTER_SIM=1, tested in isolation from install()'s sys.modules
#  injection (which would clobber the fakes.install() state every other
#  test in this file depends on).
# ════════════════════════════════════════════════════════
class TestChamberSim(unittest.TestCase):
    def setUp(self):
        self.chamber = ChamberSim()

    def _run(self, seconds, dt=0.2):
        n = int(seconds / dt)
        for _ in range(n):
            self.chamber.step(dt)

    def test_starts_at_atmosphere_no_spin(self):
        self.assertAlmostEqual(self.chamber.pirani_v, 3.3, places=3)
        self.assertEqual(self.chamber.turbo_rpm, 0.0)

    def test_turbo_ramps_up_when_enabled(self):
        self.chamber.set_turbo_enabled(True)
        self._run(5.0)
        self.assertGreater(self.chamber.turbo_rpm, 0.0)

    def test_turbo_ramps_down_when_disabled(self):
        self.chamber.set_turbo_enabled(True)
        self._run(10.0)
        spun_up = self.chamber.turbo_rpm
        self.assertGreater(spun_up, 0.0)
        self.chamber.set_turbo_enabled(False)
        self._run(10.0)
        self.assertLess(self.chamber.turbo_rpm, spun_up)

    def test_turbo_never_exceeds_full_scale(self):
        self.chamber.set_turbo_enabled(True)
        self._run(120.0)
        self.assertLessEqual(self.chamber.turbo_rpm, config.TURBO_RPM_FULL_SCALE)

    def test_pressure_falls_while_pumping_with_valve_closed(self):
        self.chamber.set_turbo_enabled(True)
        self._run(20.0)
        self.assertLess(self.chamber.pirani_v, 3.3)

    def test_pressure_rises_toward_atmosphere_when_valve_open(self):
        self.chamber.set_turbo_enabled(True)
        self._run(20.0)
        low_point = self.chamber.pirani_v
        self.chamber.set_turbo_valve(True)
        self._run(10.0)
        self.assertGreater(self.chamber.pirani_v, low_point)

    def test_pressure_never_leaves_sane_bounds(self):
        self.chamber.set_turbo_enabled(True)
        self.chamber.set_mfc_flow(config.MFC_FULL_SCALE)
        self._run(120.0)
        self.assertGreaterEqual(self.chamber.pirani_v, 0.0)
        self.assertLessEqual(self.chamber.pirani_v, 3.3)

    def test_mfc_flow_chases_commanded(self):
        self.chamber.set_mfc_flow(200.0)
        self._run(3.0)
        self.assertGreater(self.chamber.mfc_flow, 100.0)
        self.assertLessEqual(self.chamber.mfc_flow, 200.0 + 1e-6)

    def test_mfc_v_reflects_measured_flow(self):
        self.chamber.set_mfc_flow(config.MFC_FULL_SCALE)
        self._run(5.0)
        self.assertGreater(self.chamber.mfc_v, 0.0)
        self.assertLessEqual(self.chamber.mfc_v, config.ADC_VREF + 1e-6)

    def test_turbo_v_reflects_rpm(self):
        self.chamber.set_turbo_enabled(True)
        self._run(60.0)
        self.assertAlmostEqual(
            self.chamber.turbo_v,
            self.chamber.turbo_rpm / config.TURBO_RPM_FULL_SCALE * config.TURBO_RPM_VOLTAGE_FULL_SCALE,
            places=6)

    def test_plasma_current_idle_by_default(self):
        self._run(5.0)
        self.assertLess(self.chamber.pzem_current, config.PZEM_PLASMA_CURRENT_ON_A)

    def test_plasma_strike_is_manual_not_automatic(self):
        # A flow+low-pressure heuristic was tried first, but it fired
        # during ARGON_FLUSH itself since the PID legitimately drives flow
        # and pressure into the same ballpark while just converging on the
        # flush target -- well before any real ignition attempt. Even with
        # flow commanded at a deposition-relevant low pressure, sustained
        # for a while, the simulated variac must stay off until the
        # operator explicitly says otherwise.
        self.chamber.set_turbo_enabled(True)
        self.chamber.turbo_rpm = config.TURBO_RPM_FULL_SCALE
        self.chamber.pirani_v = 1.0
        self.chamber.set_mfc_flow(150.0)
        self._run(30.0)
        self.assertLess(self.chamber.pzem_current, config.PZEM_PLASMA_CURRENT_ON_A)

    def test_set_plasma_struck_true_raises_current(self):
        self.chamber.set_plasma_struck(True)
        self._run(1.0)
        self.assertGreaterEqual(self.chamber.pzem_current, config.PZEM_PLASMA_CURRENT_ON_A)

    def test_set_plasma_struck_false_clears_current(self):
        self.chamber.set_plasma_struck(True)
        self._run(1.0)
        self.assertGreaterEqual(self.chamber.pzem_current, config.PZEM_PLASMA_CURRENT_ON_A)
        self.chamber.set_plasma_struck(False)
        self._run(1.0)
        self.assertLess(self.chamber.pzem_current, config.PZEM_PLASMA_CURRENT_ON_A)

    def test_plasma_struck_persists_regardless_of_flow_or_pressure(self):
        # Manual control means manual -- nothing about flow/pressure
        # dynamics should clear it once the operator has set it.
        self.chamber.set_plasma_struck(True)
        self.chamber.set_mfc_flow(0.0)
        self.chamber.pirani_v = 3.3
        self._run(5.0)
        self.assertGreaterEqual(self.chamber.pzem_current, config.PZEM_PLASMA_CURRENT_ON_A)


# ════════════════════════════════════════════════════════
#  Web dashboard server (web_ui.WebUI) -- stdlib HTTP + SSE, tested
#  against a real socket on an ephemeral port with fake state/commands.
# ════════════════════════════════════════════════════════
import json as _json
import urllib.request
import urllib.error


class TestWebUI(unittest.TestCase):
    def setUp(self):
        self.snapshot = {"sm_state": "IDLE", "pirani_voltage": 1.5, "flag": True}
        self.calls = []

        def ok_cmd(value):
            self.calls.append(("ok_cmd", value))
            return True, ""

        self.ui = WebUI(
            get_snapshot=lambda: dict(self.snapshot),
            commands={
                "ok_cmd":   ok_cmd,
                "fail_cmd": lambda v: (False, "guard rejected it"),
                "boom":     lambda v: 1 / 0,
            },
            page_path=os.path.join(REPO_ROOT, "ui", "web", "index.html"),
            port=0,                  # ephemeral -- avoids clashes between test runs
            update_interval=0.02,
        )
        self.assertTrue(self.ui.start())
        self.base = f"http://127.0.0.1:{self.ui.port}"

    def tearDown(self):
        self.ui.stop()

    def _post(self, body_bytes):
        req = urllib.request.Request(self.base + "/command", data=body_bytes,
                                     headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=2) as resp:
                return resp.status, _json.loads(resp.read())
        except urllib.error.HTTPError as e:
            return e.code, _json.loads(e.read())

    def test_serves_dashboard_page(self):
        with urllib.request.urlopen(self.base + "/", timeout=2) as resp:
            self.assertEqual(resp.status, 200)
            body = resp.read().decode()
        self.assertIn("Sputter Vacuum Controller", body)
        self.assertIn("EventSource", body, "page must connect to the SSE stream")

    def test_sse_stream_yields_parseable_snapshots(self):
        with urllib.request.urlopen(self.base + "/events", timeout=2) as resp:
            self.assertEqual(resp.headers.get("Content-Type"), "text/event-stream")
            line = resp.readline()
            while not line.startswith(b"data:"):
                line = resp.readline()
        snap = _json.loads(line[len(b"data:"):].strip())
        self.assertEqual(snap["sm_state"], "IDLE")
        self.assertEqual(snap["pirani_voltage"], 1.5)

    def test_command_dispatch_and_value_passthrough(self):
        status, out = self._post(_json.dumps({"action": "ok_cmd", "value": "0.02"}).encode())
        self.assertEqual(status, 200)
        self.assertTrue(out["ok"])
        self.assertEqual(self.calls, [("ok_cmd", "0.02")])

    def test_command_failure_reported_not_raised(self):
        status, out = self._post(_json.dumps({"action": "fail_cmd"}).encode())
        self.assertEqual(status, 200)
        self.assertFalse(out["ok"])
        self.assertIn("guard rejected it", out["message"])

    def test_unknown_action_rejected(self):
        status, out = self._post(_json.dumps({"action": "no_such_thing"}).encode())
        self.assertEqual(status, 400)
        self.assertFalse(out["ok"])

    def test_handler_exception_answers_500_and_server_survives(self):
        status, out = self._post(_json.dumps({"action": "boom"}).encode())
        self.assertEqual(status, 500)
        self.assertFalse(out["ok"])
        # the server must still be alive afterward
        status, out = self._post(_json.dumps({"action": "ok_cmd"}).encode())
        self.assertEqual(status, 200)
        self.assertTrue(out["ok"])

    def test_malformed_json_rejected(self):
        status, out = self._post(b"this is not json{{")
        self.assertEqual(status, 400)
        self.assertFalse(out["ok"])

    def test_unknown_path_404(self):
        with self.assertRaises(urllib.error.HTTPError) as ctx:
            urllib.request.urlopen(self.base + "/nope", timeout=2)
        self.assertEqual(ctx.exception.code, 404)

    def test_port_conflict_degrades_gracefully(self):
        second = WebUI(get_snapshot=lambda: {}, commands={},
                       page_path=self.ui.page_path,
                       port=self.ui.port,  # deliberately taken
                       update_interval=0.02)
        self.assertFalse(second.start(), "must return False, not raise")


if __name__ == "__main__":
    unittest.main(verbosity=2)
