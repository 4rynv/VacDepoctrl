#!/usr/bin/env python3
"""
main.py — Sputter Vacuum Controller
State machine + live scrolling graphs for Pirani voltage and MFC flow.
"""

import os
import sys

# Simulation mode: SPUTTER_SIM=1 python main.py, or python main.py --sim.
# Installs fake hardware modules into sys.modules BEFORE the real ones are
# imported below -- see sim_hardware.py. Everything else in this file runs
# completely unmodified against them; nothing downstream needs to know.
SIM_MODE = os.environ.get("SPUTTER_SIM") == "1" or "--sim" in sys.argv
_sim_chamber = None
if SIM_MODE:
    import sim_hardware
    _sim_chamber = sim_hardware.install()

import logging
import math
import socket
import threading
import time
import tkinter as tk
from tkinter import messagebox

import RPi.GPIO as GPIO
import adafruit_ads1x15.ads1115 as ADS
from adafruit_extended_bus import ExtendedI2C

from config import (
    ADC_GAIN,
    POLLING_INTERVAL,
    GUI_REFRESH_INTERVAL,
    ERROR_DISPLAY_SECONDS,
    GRAPH_MAX_SAMPLES,
    GRAPH_PIRANI_MIN,
    GRAPH_PIRANI_MAX,
    GRAPH_PIRANI_COLOR,
    GRAPH_MFC_MIN,
    GRAPH_MFC_MAX,
    GRAPH_MFC_COLOR,
    GRAPH_TURBO_RPM_MIN,
    GRAPH_TURBO_RPM_MAX,
    GRAPH_TURBO_RPM_COLOR,
    IDLE_PRESSURE_MAX_VOLTAGE,
    PUMP_DOWN_COMPLETE_VOLTAGE,
    VENTING_COMPLETE_VOLTAGE,
    ARGON_FLUSH_TARGET_VOLTAGE,
    ARGON_FLUSH_FLOW_SETPOINT,
    SPUTTER_READY_TARGET_VOLTAGE,
    SPUTTER_TARGET_MAX_MBAR,
    ARGON_PRESSURE_MIN_PSI,
    PLASMA_IGNITION_TIMEOUT,
    PRESSURE_CONTROL_KP,
    ADS1115_I2C_ADDRESS,
    ARGON_DAC_I2C_ADDRESS,
    ARGON_DAC_VREF,
    GPIO_PIRANI_PIN,
    GPIO_MFC_VALVE_CLOSE_PIN,
    GPIO_TURBO_VALVE_PIN,
    TURBO_VALVE_OPEN_RPM_MAX,
    TURBO_VALVE_CONFIRM_SAMPLES,
    VENTING_PUMP_OFF_PROMPT_RPM,
    TURBO_RPM_FULL_SCALE,
    TURBO_SPINUP_GRACE_SECONDS,
    TURBO_RPM_STALL_THRESHOLD,
    TURBO_RPM_DROP_GRACE_SECONDS,
    MFC_FULL_SCALE,
    MFC_FLOW_MIN_COMMANDED_SCCM,
    MFC_FLOW_RESPONSE_FRACTION,
    MFC_FLOW_GRACE_SECONDS,
    DAC_SATURATION_MARGIN_V,
    DAC_SATURATION_FLOW_FRACTION,
    DAC_SATURATION_GRACE_SECONDS,
    PRESSURE_CONVERGENCE_TOLERANCE_V,
    PRESSURE_CONVERGENCE_GRACE_SECONDS,
    STARTUP_GRACE_SECONDS,
    HEARTBEAT_TIMEOUT_SECONDS,
    CONSECUTIVE_EXCEPTION_LIMIT,
    SANE_VOLTAGE_MIN,
    SANE_VOLTAGE_MAX,
    SENSOR_RANGE_CONFIRM_SAMPLES,
    EVENT_LOG_FILENAME,
    I2C_BUS_NUMBER,
    PZEM_PLASMA_CURRENT_ON_A,
    PZEM_PLASMA_CURRENT_OFF_A,
    PZEM_PLASMA_CONFIRM_SAMPLES,
    PZEM_PLASMA_ABSENT_GRACE_SECONDS,
    PZEM_POWER_UNEXPECTED_GRACE_SECONDS,
    GRAPH_PZEM_VOLTAGE_MAX,
    GRAPH_PZEM_CURRENT_MAX,
    GRAPH_PZEM_POWER_MAX,
    GRAPH_PZEM_FREQ_MIN,
    GRAPH_PZEM_FREQ_MAX,
)
# Pirani calibration table: (mbar, gauge_voltage) from datasheet
# ADC voltage = gauge_voltage * 0.33 (resistor divider)
_PIRANI_CAL = [
    (999, 10.00), (850, 9.58), (800, 9.52), (700, 9.45), (600, 9.38),
    (500, 9.28),  (400, 9.20), (300, 9.15), (200, 9.08), (100, 9.04),
    (50,  8.85),  (25,  8.70), (20,  8.60), (15,  8.40), (10,  8.20),
    (9,   8.10),  (8,   8.00), (7,   7.85), (6,   7.75), (5,   7.55),
    (4,   7.28),  (3,   6.92), (2,   6.27), (1,   5.70), (0.9, 5.60),
    (0.8, 5.51),  (0.7, 5.45), (0.6, 5.31), (0.5, 5.15), (0.4, 4.90),
    (0.3, 4.52),  (0.2, 4.28), (0.1, 3.98), (0.09, 3.90),(0.08, 3.75),
    (0.07, 3.64), (0.06, 3.55),(0.05, 3.44),(0.04, 3.23),(0.03, 2.93),
    (0.02, 2.50), (0.01, 1.55),(0.009, 1.45),(0.008, 1.30),(0.007, 1.10),
    (0.006, 0.95),(0.005, 0.80),(0.004, 0.65),(0.003, 0.50),(0.002, 0.35),
    (0.001, 0.12),(0.0, 0.01),
]
# Pre-scale by divider ratio so table is in ADC volts
_PIRANI_CAL_ADC = [(p, v * 0.33) for p, v in _PIRANI_CAL]

def mbar_to_adc_voltage(mbar):
    """Interpolate Pirani calibration table using log-linear interpolation.
    Interpolates linearly in log-pressure space to match the gauge physical response.
    """
    cal = _PIRANI_CAL_ADC
    if mbar >= cal[0][0]:  return cal[0][1]
    if mbar <= cal[-1][0]: return cal[-1][1]
    for i in range(len(cal) - 1):
        p_hi, v_hi = cal[i]
        p_lo, v_lo = cal[i+1]
        if p_lo <= mbar <= p_hi:
            if p_lo <= 0: p_lo = 1e-6
            t = (math.log(mbar) - math.log(p_lo)) / (math.log(p_hi) - math.log(p_lo))
            return v_lo + t * (v_hi - v_lo)
    return cal[-1][1]

def adc_voltage_to_mbar(voltage):
    """Inverse of mbar_to_adc_voltage: ADC voltage -> pressure in mbar.
    Interpolates linearly in voltage, logarithmically in pressure, matching
    the Pirani gauge's log response (same DHPG-015 calibration table).
    """
    cal = _PIRANI_CAL_ADC
    if voltage >= cal[0][1]:  return cal[0][0]   # >= 3.30 V: atmosphere (999)
    if voltage <= cal[-1][1]: return cal[-1][0]  # <= 0.0033 V: over-range (0)
    for i in range(len(cal) - 1):
        p_hi, v_hi = cal[i]
        p_lo, v_lo = cal[i+1]
        if v_lo <= voltage <= v_hi:
            if p_lo <= 0: p_lo = 1e-6
            t = (voltage - v_lo) / (v_hi - v_lo)
            return math.exp(math.log(p_lo) + t * (math.log(p_hi) - math.log(p_lo)))
    return cal[-1][0]

def turbo_valve_step(venting, rpm, valve_open, below_ticks):
    """Debounced turbo inlet valve latch. Pure decision logic — no I/O.

    Driven by turbo pump RPM (tach on ADC A2), not chamber pressure: the
    rotor's own speed is the direct signal for whether it's safe to admit
    gas, rather than inferring rotor state from pressure. The valve may
    only open during VENTING, and only after TURBO_VALVE_CONFIRM_SAMPLES
    consecutive readings at or below TURBO_VALVE_OPEN_RPM_MAX: a single
    corrupted read on the software I2C bus (EMI bit-flips return garbage
    without raising) must not latch the valve open. An above-threshold
    reading resets the count. Once open, the valve stays latched open (no
    relay chatter) until VENTING is left; leaving VENTING also clears the
    count so a partial count can't carry into the next vent. Returns
    (valve_open, below_ticks).
    """
    if not venting:
        return False, 0
    if valve_open:
        return True, 0
    if rpm <= TURBO_VALVE_OPEN_RPM_MAX:
        below_ticks += 1
        return below_ticks >= TURBO_VALVE_CONFIRM_SAMPLES, below_ticks
    return False, 0

def turbo_spinup_check(opto_enabled, rpm, on_since, now):
    """Cross-sensor check: opto_enabled (GPIO 17) is us commanding the turbo
    to spin up; rpm (ADC A2) is the pump reporting back. Individually both
    can look "valid" while still disagreeing -- flags if opto has been on
    for TURBO_SPINUP_GRACE_SECONDS but the rotor still reads below
    TURBO_RPM_STALL_THRESHOLD (turbo fault, RPM sensor fault, or wiring
    fault). `on_since` is the timestamp opto first turned on, threaded back
    in by the caller each tick so this function stays pure. Returns
    (flagged, on_since).
    """
    if not opto_enabled:
        return False, None
    if on_since is None:
        on_since = now
    flagged = (rpm < TURBO_RPM_STALL_THRESHOLD
               and (now - on_since) >= TURBO_SPINUP_GRACE_SECONDS)
    return flagged, on_since

def turbo_rpm_drop_check(opto_enabled, rpm, confirmed_healthy, dropped_since, now):
    """Cross-sensor check: distinct fault from turbo_spinup_check, which
    only catches "never spun up". Once RPM has confirmed healthy spin
    (exceeded TURBO_RPM_STALL_THRESHOLD at least once since opto turned
    on), a sustained drop back below that threshold while opto is still
    enabled means the turbo was running and is now failing/degrading, not
    just slow to start. `confirmed_healthy` and `dropped_since` are
    threaded back in by the caller each tick so this stays pure. Returns
    (flagged, confirmed_healthy, dropped_since).
    """
    if not opto_enabled:
        return False, False, None
    if rpm >= TURBO_RPM_STALL_THRESHOLD:
        return False, True, None
    if not confirmed_healthy:
        return False, False, None
    if dropped_since is None:
        dropped_since = now
    flagged = (now - dropped_since) >= TURBO_RPM_DROP_GRACE_SECONDS
    return flagged, confirmed_healthy, dropped_since

def mfc_flow_check(flow_target, measured_flow, stalled_since, now):
    """Cross-sensor check: flow_target (what we commanded the MFC via the
    DAC) vs measured_flow (the MFC's own analog feedback on A1). Flags if a
    meaningful commanded flow has stayed well above measured flow for
    MFC_FLOW_GRACE_SECONDS -- the actuator isn't responding (dead MFC,
    disconnected wiring, empty gas supply), rather than the PID loop just
    still settling. Returns (flagged, stalled_since).
    """
    commanding = flow_target >= MFC_FLOW_MIN_COMMANDED_SCCM
    responding = measured_flow >= flow_target * MFC_FLOW_RESPONSE_FRACTION
    if not commanding or responding:
        return False, None
    if stalled_since is None:
        stalled_since = now
    flagged = (now - stalled_since) >= MFC_FLOW_GRACE_SECONDS
    return flagged, stalled_since

def dac_saturation_flow_check(dac_voltage, measured_flow, stalled_since, now):
    """Cross-sensor check: is the DAC pegged at (or very near) its output
    ceiling while measured flow stays far below what full-scale should
    deliver? DAC saturation means the PID has given up on proportional
    correction and is commanding maximum output -- a stronger fault signal
    than mfc_flow_check's target/measured mismatch: valve stuck, line
    blocked, or gas supply empty. Returns (flagged, stalled_since).
    """
    saturated = dac_voltage >= (ARGON_DAC_VREF - DAC_SATURATION_MARGIN_V)
    responding = measured_flow >= MFC_FULL_SCALE * DAC_SATURATION_FLOW_FRACTION
    if not saturated or responding:
        return False, None
    if stalled_since is None:
        stalled_since = now
    flagged = (now - stalled_since) >= DAC_SATURATION_GRACE_SECONDS
    return flagged, stalled_since

def pressure_convergence_check(current_voltage, target_voltage, diverging_since, now):
    """Cross-sensor check: is the pressure PID loop actually converging on
    its own target? Flags if |current - target| has stayed above
    PRESSURE_CONVERGENCE_TOLERANCE_V for PRESSURE_CONVERGENCE_GRACE_SECONDS
    despite the control loop running every tick -- suggests a leak, stuck
    valve, or MFC fault rather than normal settling time. Returns
    (flagged, diverging_since).
    """
    error = abs(current_voltage - target_voltage)
    if error <= PRESSURE_CONVERGENCE_TOLERANCE_V:
        return False, None
    if diverging_since is None:
        diverging_since = now
    flagged = (now - diverging_since) >= PRESSURE_CONVERGENCE_GRACE_SECONDS
    return flagged, diverging_since

def vent_complete_ready(pirani_voltage, confirmed):
    """Pure decision: has VENTING earned its transition to IDLE? Requires
    BOTH atmospheric pressure (VENTING_COMPLETE_VOLTAGE) AND explicit
    operator confirmation that the primary/roughing pump has been turned
    off. Pressure alone used to auto-complete this via sm.update(), but
    that let the turbo inlet valve reclose (turbo_valve_step() closes it
    the instant `venting` goes False) the moment atmosphere was reached —
    before the operator had any real chance to react to the "turn off the
    primary pump" cue, let alone actually flip the switch. Now owned
    exclusively by _poll(), the same way ARGON_FLUSH -> PLASMA_IGNITING is
    (see state_machine.py's update() docstring).
    """
    return pirani_voltage >= VENTING_COMPLETE_VOLTAGE and confirmed


_PZEM_PLASMA_EXPECTED_STATES = ("PLASMA_IGNITING", "SPUTTER_READY", "SPUTTERING")


def plasma_detect_step(current_a, detected, confirm_ticks):
    """Debounced hysteresis latch for plasma-ignition detection via
    variac-side AC current draw (PZEM-004T-100A on the sputtering supply's
    variac output). Mirrors the turbo enable opto's ON/OFF hysteresis
    pattern: a rising edge must clear PZEM_PLASMA_CURRENT_ON_A for
    PZEM_PLASMA_CONFIRM_SAMPLES consecutive ticks before `detected` latches
    True -- a single Modbus CRC glitch must not flip a status other
    cross-sensor checks (and the operator-armed auto-confirm toggle) key
    off of. A falling edge below PZEM_PLASMA_CURRENT_OFF_A clears
    immediately -- no debounce needed to fail safe toward "not detected".
    Between the two thresholds, holds the previous state and resets the
    confirm count (same dead-band behavior as the opto hysteresis).
    Returns (detected, confirm_ticks).
    """
    if current_a >= PZEM_PLASMA_CURRENT_ON_A:
        if detected:
            return True, 0
        confirm_ticks += 1
        return confirm_ticks >= PZEM_PLASMA_CONFIRM_SAMPLES, confirm_ticks
    if current_a <= PZEM_PLASMA_CURRENT_OFF_A:
        return False, 0
    return detected, 0


def pzem_plasma_absent_check(state, plasma_detected, absent_since, now):
    """Cross-sensor check: SPUTTER_READY/SPUTTERING both assume plasma is
    already lit (PLASMA_IGNITING's own PLASMA_IGNITION_TIMEOUT owns the
    "never struck" case and is deliberately excluded here -- flagging a
    still-igniting discharge as "absent" would just duplicate that abort
    path with a shorter, unrelated grace period). If PZEM stops seeing
    plasma-level current once actually in one of those two states, that's
    an arc dropout or the plasma extinguishing mid-run. Flags after
    PZEM_PLASMA_ABSENT_GRACE_SECONDS. Returns (flagged, absent_since).
    """
    if state not in ("SPUTTER_READY", "SPUTTERING"):
        return False, None
    if plasma_detected:
        return False, None
    if absent_since is None:
        absent_since = now
    flagged = (now - absent_since) >= PZEM_PLASMA_ABSENT_GRACE_SECONDS
    return flagged, absent_since


def pzem_power_unexpected_check(state, plasma_detected, unexpected_since, now):
    """Cross-sensor check: plasma-level current draw on the variac while the
    process isn't attempting ignition at all (IDLE/PUMP_DOWN/READY/
    ARGON_FLUSH/VENTING) means the HV/RF supply is energized when it
    shouldn't be -- a stuck relay, a supply left live, or a wiring fault --
    not a legitimate reading at any of those states. Flags after
    PZEM_POWER_UNEXPECTED_GRACE_SECONDS. Returns (flagged, unexpected_since).
    """
    if state in _PZEM_PLASMA_EXPECTED_STATES:
        return False, None
    if not plasma_detected:
        return False, None
    if unexpected_since is None:
        unexpected_since = now
    flagged = (now - unexpected_since) >= PZEM_POWER_UNEXPECTED_GRACE_SECONDS
    return flagged, unexpected_since


def sensor_range_check(voltages, bad_ticks):
    """Global gate check: are all given ADC-derived voltages within a
    physically sane range (post-divider signals referenced to the Pi's
    3.3V rail)? A reading outside SANE_VOLTAGE_MIN/MAX means a
    disconnected, shorted, or miswired sensor -- not a legitimate reading
    at any pressure/flow/speed. Debounced like every other hardware-facing
    check here: a single corrupted I2C sample (EMI bit-flip, no raised
    exception) must not immediately trip a full shutdown; requires
    SENSOR_RANGE_CONFIRM_SAMPLES consecutive bad ticks. Returns
    (flagged, bad_ticks).
    """
    bad = any(v < SANE_VOLTAGE_MIN or v > SANE_VOLTAGE_MAX for v in voltages)
    if not bad:
        return False, 0
    bad_ticks += 1
    return bad_ticks >= SENSOR_RANGE_CONFIRM_SAMPLES, bad_ticks

def config_sanity_failures():
    """Pure config-invariant checks, no I/O. Returns a list of failure
    strings (empty if all pass). Mirrors TestConfigSanity in the test
    suite but also runs live at startup, so a bad future edit to config.py
    is caught before the app runs with a broken safety threshold -- this is
    exactly the class of bug that once shipped here: VENTING_COMPLETE_VOLTAGE
    was 2.5V (~5 mbar, nowhere near atmosphere) instead of near-atmosphere.
    """
    failures = []
    if not (PUMP_DOWN_COMPLETE_VOLTAGE < ARGON_FLUSH_TARGET_VOLTAGE
            < IDLE_PRESSURE_MAX_VOLTAGE < VENTING_COMPLETE_VOLTAGE):
        failures.append(
            "Pressure thresholds out of order: expected "
            "PUMP_DOWN_COMPLETE_VOLTAGE < ARGON_FLUSH_TARGET_VOLTAGE < "
            "IDLE_PRESSURE_MAX_VOLTAGE < VENTING_COMPLETE_VOLTAGE."
        )
    if not (0 < TURBO_VALVE_OPEN_RPM_MAX < TURBO_RPM_FULL_SCALE):
        failures.append("TURBO_VALVE_OPEN_RPM_MAX out of range (0, TURBO_RPM_FULL_SCALE).")
    pins = [GPIO_PIRANI_PIN, GPIO_MFC_VALVE_CLOSE_PIN, GPIO_TURBO_VALVE_PIN]
    if len(pins) != len(set(pins)):
        failures.append("GPIO pin collision in config.py.")
    if any(pin in (2, 3, 4, 23, 24) for pin in pins):
        failures.append("A configured GPIO pin is on the dead/reserved list (2, 3, 4, 23, 24).")
    if not (PZEM_PLASMA_CURRENT_OFF_A < PZEM_PLASMA_CURRENT_ON_A):
        failures.append(
            "PZEM_PLASMA_CURRENT_OFF_A must be less than PZEM_PLASMA_CURRENT_ON_A (hysteresis)."
        )
    return failures

from pirani        import PiraniController
from mfc_control   import MFCController
from turbo_rpm     import TurboRPMController
from pzem_meter    import PZEMController
from state_machine import SputterStateMachine, STATE_COLORS
from graph         import ScrollingGraph


# ════════════════════════════════════════════════════════
#  PERSISTENT EVENT LOG
# ════════════════════════════════════════════════════════
# Append-only record of every operator-visible error/warning and every
# state transition -- the GUI status bar only shows the latest message and
# auto-expires it after ERROR_DISPLAY_SECONDS, so this is the only place an
# incident can be reconstructed after the fact.
logging.basicConfig(
    filename=os.path.join(os.path.dirname(os.path.abspath(__file__)), EVENT_LOG_FILENAME),
    level=logging.INFO,
    format="%(asctime)s %(message)s",
)
_event_log = logging.getLogger("sputter_ctrl")

def _log_event(msg):
    """Logging must never crash the control loop."""
    try:
        _event_log.info(msg)
    except Exception:
        pass


# ════════════════════════════════════════════════════════
#  HARDWARE INIT
# ════════════════════════════════════════════════════════
GPIO.setmode(GPIO.BCM)

# Turbo inlet valve (GPIO 4 -> BC547 -> relay, valve on NC contact):
# LOW = relay released = NC shorted = valve CLOSED. Held closed from startup;
# driven HIGH (valve open) only at/below TURBO_VALVE_OPEN_RPM_MAX during VENTING (see _poll).
GPIO.setup(GPIO_TURBO_VALVE_PIN, GPIO.OUT)
GPIO.output(GPIO_TURBO_VALVE_PIN, GPIO.LOW)

i2c = ExtendedI2C(I2C_BUS_NUMBER)
ads = ADS.ADS1115(i2c, address=ADS1115_I2C_ADDRESS)
ads.gain = ADC_GAIN

pirani    = PiraniController(ads)
mfc       = MFCController(ads, i2c=i2c)
turbo_rpm = TurboRPMController(ads)
pzem      = PZEMController()  # optional hardware — see pzem_meter.py; never blocks startup
sm        = SputterStateMachine()

def _startup_self_test():
    """Runs once, before the poll thread starts. Validates config
    invariants (config_sanity_failures()) and confirms the ADS1115 and
    MCP4725 DAC actually respond on the I2C bus. Refuses to start on
    failure rather than silently running with a broken safety threshold or
    a missing sensor. Console-only -- no GUI exists yet at this point in
    startup, so failures print to stderr/console and exit.
    """
    failures = config_sanity_failures()

    try:
        if i2c.try_lock():
            try:
                addrs = i2c.scan()
            finally:
                i2c.unlock()
        else:
            addrs = []
            failures.append("Could not lock I2C bus to scan for devices.")
    except Exception as e:
        addrs = []
        failures.append(f"I2C scan failed: {e}")

    if ADS1115_I2C_ADDRESS not in addrs:
        failures.append(f"ADS1115 not detected on I2C bus at 0x{ADS1115_I2C_ADDRESS:02X}.")
    if ARGON_DAC_I2C_ADDRESS not in addrs:
        failures.append(f"MCP4725 DAC not detected on I2C bus at 0x{ARGON_DAC_I2C_ADDRESS:02X}.")

    if failures:
        msg = "Startup self-test FAILED:\n  - " + "\n  - ".join(failures)
        print(msg, file=sys.stderr)
        _log_event("STARTUP SELF-TEST FAILED: " + "; ".join(failures))
        sys.exit(1)

    _log_event("Startup self-test passed.")


_startup_self_test()


def handle_hardware_interlocks(old_state, new_state):
    """Executes instantaneous safety overrides on the main thread when states shift."""
    _log_event(f"State transition: {old_state} -> {new_state}")
    if new_state in ["IDLE", "VENTING"]:
        # Safety cutoff: kill gas flow, drop DAC to 0V, turn off opto
        mfc.set_flow(0.0)
        mfc.valve_close()
        pirani.set_opto(False)
        # Reset plasma ignition tracking so the next flush cycle starts clean —
        # a stale triggered flag would block _poll from re-arming the ignition
        # timeout on the next ARGON_FLUSH cycle. Same reasoning for the vent-
        # complete flags: a stale vent_complete_confirmed=True left over from
        # a prior cycle must not silently auto-complete the *next* vent the
        # instant it reaches atmosphere.
        with _lock:
            _state["plasma_ignition_start"] = None
            _state["plasma_ignition_triggered"] = False
            _state["vent_complete_pending"] = False
            _state["vent_complete_confirmed"] = False

    if new_state == "PUMP_DOWN":
        # Reset opto so it can re-trigger at 1.2 V on the next pump-down cycle
        pirani.set_opto(False)

# Register the hook into the state machine
sm.on_transition_callback = handle_hardware_interlocks

# ════════════════════════════════════════════════════════
#  IP ADDRESS
# ════════════════════════════════════════════════════════
def _get_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "Unavailable"


# ════════════════════════════════════════════════════════
#  SHARED STATE
# ════════════════════════════════════════════════════════
_lock  = threading.Lock()
_state = {
    "pirani_adc":      0,
    "pirani_voltage":  0.0,
    "opto_enabled":    False,
    "mfc_adc":         0,
    "mfc_voltage":     0.0,
    "mfc_flow":        0.0,
    "mfc_valve_closed": False,

    "dac_ready":       False,
    "dac_voltage":     0.0,
    "dac_code":        0,

    "flow_target":     0.0,
    "turbo_valve_open": False,
    "turbo_rpm_adc":   0,
    "turbo_rpm_voltage": 0.0,
    "turbo_rpm":       0.0,
    "argon_pressure":  0.0,
    "sputter_target_mbar": 0.007,  # default 0.007 mbar
    "plasma_ignition_start": None,
    "plasma_ignition_triggered": False,

    "vent_complete_pending":   False,  # VENTING has reached atmosphere; waiting on operator confirmation
    "vent_complete_confirmed": False,  # operator has confirmed the primary/roughing pump is off

    "pzem_ready":       False,
    "pzem_voltage":     0.0,
    "pzem_current":     0.0,
    "pzem_power":       0.0,
    "pzem_energy":      0.0,
    "pzem_frequency":   0.0,
    "pzem_power_factor": 0.0,
    "plasma_detected":  False,   # PZEM current-draw detection, see plasma_detect_step()
    "pzem_auto_confirm": False,  # operator-armed: auto-advance PLASMA_IGNITING -> SPUTTER_READY

    "sm_state":        "IDLE",
    "error":           "",
    "last_tick":       time.time(),  # Poll-loop heartbeat; see _refresh() watchdog check
    "safety_tripped":  False,        # Latched by _force_safe_shutdown(); see Clear Fault button
    "safety_trip_reason": "",
}


_last_logged_error = None  # De-dupes repeated identical log lines while a
                           # condition persists tick after tick


def _set_error(msg):
    """Single choke point for every operator-visible error/warning: writes
    _state["error"] under the lock and appends to the persistent event log.
    A repeated identical message is logged once, not every tick, while a
    condition persists; the de-dupe resets once the message changes or is
    cleared, so a later recurrence of the same message logs again.
    """
    global _last_logged_error
    with _lock:
        _state["error"] = msg
    if msg and msg != _last_logged_error:
        _log_event(msg)
        _last_logged_error = msg
    elif not msg:
        _last_logged_error = None


def _force_safe_shutdown(reason):
    """Unconditional hardware-safe cutoff. Callable from either thread: the
    poll thread's global gate, or the GUI thread's heartbeat watchdog.

    Forces the turbo valve closed directly via GPIO -- bypassing _poll()'s
    local latch state, which may be stale or (if the poll thread itself has
    hung) never updating again -- then routes everything else through the
    existing E-STOP path. mfc/pirani cutoffs inside emergency_stop() are
    already exception-safe (SputterStateMachine catches callback errors
    internally), so no extra guarding is needed around that call. Safe to
    call every tick for as long as the underlying condition persists --
    GPIO/state-machine calls are idempotent and _set_error() de-dupes the
    repeated log line.

    Also LATCHES _state["safety_tripped"]. A bare emergency_stop() forces
    IDLE, but IDLE's own auto-resume-to-PUMP_DOWN logic in _poll() would
    otherwise undo that within one tick whenever pressure is already low
    (e.g. from an external roughing pump) -- silently continuing the
    process as if the trip never happened. The latch blocks that auto-
    resume until the operator explicitly clicks Clear Fault. If the
    underlying condition is still true when cleared, it re-trips
    immediately (the per-check "since" timestamps aren't reset by
    clearing) -- that's intentional, not a bug: clearing without fixing
    the root cause should not let the process quietly continue.
    """
    try:
        GPIO.output(GPIO_TURBO_VALVE_PIN, GPIO.LOW)
    except Exception as e:
        print(f"[SAFETY] Turbo valve cutoff failed during forced shutdown: {e}")
    sm.emergency_stop()
    with _lock:
        _state["safety_tripped"] = True
        _state["safety_trip_reason"] = reason
    _set_error(f"[FORCED SHUTDOWN] {reason}")


def _on_clear_fault():
    """Operator acknowledgment for a latched safety trip -- see
    _force_safe_shutdown(). Unblocks IDLE's auto-resume-to-PUMP_DOWN; does
    NOT reset the underlying check's own timers, so an unaddressed fault
    re-trips immediately rather than silently continuing.
    """
    with _lock:
        _state["safety_tripped"] = False
        _state["safety_trip_reason"] = ""
    _log_event("Safety fault cleared by operator.")


def _on_start_argon_flush():
    with _lock:
        argon_pressure = _state["argon_pressure"]

    if argon_pressure < ARGON_PRESSURE_MIN_PSI:
        messagebox.showwarning(
            "Argon pressure low",
            f"Argon pressure is {argon_pressure:.1f} psi. "
            f"Increase it to at least {ARGON_PRESSURE_MIN_PSI:.0f} psi before starting argon flush."
        )
        _set_error(f"Argon pressure below {ARGON_PRESSURE_MIN_PSI:.0f} psi; cannot start argon flush.")
        return

    if not sm.transition("ARGON_FLUSH"):
        _set_error("Cannot start argon flush in the current state.")


def _on_start_sputter():
    with _lock:
        argon_pressure = _state["argon_pressure"]

    if argon_pressure < ARGON_PRESSURE_MIN_PSI:
        messagebox.showwarning(
            "Argon pressure low",
            f"Argon pressure is {argon_pressure:.1f} psi. "
            f"Increase it to at least {ARGON_PRESSURE_MIN_PSI:.0f} psi before starting sputtering."
        )
        _set_error(f"Argon pressure below {ARGON_PRESSURE_MIN_PSI:.0f} psi; cannot start sputtering.")
        return

    if not sm.transition("SPUTTERING"):
        _set_error("Cannot start sputtering in the current state.")

def _complete_plasma_confirmation(source):
    """Shared by the manual Confirm Plasma button and the PZEM auto-confirm
    path in _poll() (armed via the Auto-Confirm toggle) so both reset
    ignition tracking identically -- a stale plasma_ignition_triggered flag
    would otherwise block re-arming the timeout on the next ARGON_FLUSH
    cycle regardless of which path confirmed it. Returns True on success.
    """
    if sm.transition("SPUTTER_READY"):
        with _lock:
            _state["plasma_ignition_start"] = None
            _state["plasma_ignition_triggered"] = False
        _set_error("")
        _log_event(f"Plasma confirmed ({source}).")
        return True
    return False


def _on_confirm_plasma():
    """Operator manually confirms plasma is ignited; proceed to SPUTTER_READY."""
    if not _complete_plasma_confirmation("operator"):
        _set_error("Cannot confirm plasma in current state.")


def _on_confirm_vent_complete():
    """Operator confirms the primary/roughing pump has been turned off.
    Unblocks the VENTING -> IDLE transition (and therefore the turbo inlet
    valve re-closing, see turbo_valve_step()) that _poll() otherwise holds
    open once atmospheric pressure is reached — see vent_complete_ready().
    A no-op if nothing is actually pending, so a stray click can't arm a
    future vent cycle early.
    """
    with _lock:
        if not _state["vent_complete_pending"]:
            return
        _state["vent_complete_confirmed"] = True
    _log_event("Vent complete confirmed by operator (primary pump off).")


def _on_toggle_auto_confirm():
    """Operator opt-in: when armed, a PZEM-confirmed plasma detection during
    PLASMA_IGNITING auto-advances to SPUTTER_READY (see _poll()) instead of
    requiring the Confirm Plasma click. Off by default -- the PZEM current
    threshold is an unvalidated placeholder (see config.py) until bench-
    measured against the real supply.
    """
    with _lock:
        _state["pzem_auto_confirm"] = not _state["pzem_auto_confirm"]
        enabled = _state["pzem_auto_confirm"]
    _log_event(f"PZEM auto-confirm {'armed' if enabled else 'disarmed'} by operator.")


_sim_plasma_strike_on = False


def _on_toggle_sim_plasma_strike():
    """SIM_MODE only: manually toggles the simulated variac current --
    see ChamberSim.set_plasma_struck(). Deliberately manual rather than
    automatic: _ignite_plasma() is still a stub on real hardware too, so
    nothing decides this automatically there either; a first attempt at
    an automatic flow+pressure heuristic in the simulation fired during
    ARGON_FLUSH itself (before any real ignition attempt), which was
    wrong. Button is only created/gridded when SIM_MODE is on.
    """
    global _sim_plasma_strike_on
    if _sim_chamber is None:
        return
    _sim_plasma_strike_on = not _sim_plasma_strike_on
    _sim_chamber.set_plasma_struck(_sim_plasma_strike_on)
    if btn_sim_plasma_strike is not None:
        btn_sim_plasma_strike.config(
            text=f"Plasma Strike (SIM): {'ON' if _sim_plasma_strike_on else 'OFF'}",
            bg="#FFA500" if _sim_plasma_strike_on else "#dddddd",
        )

# ════════════════════════════════════════════════════════
#  POLLING THREAD
# ════════════════════════════════════════════════════════
_stop_event = threading.Event()

def _poll():
    poll_start_time = time.time()  # For STARTUP_GRACE_SECONDS -- see escalation section below
    turbo_valve_open = False  # Pin driven LOW (valve closed) during hardware init
    turbo_below_ticks = 0     # Consecutive VENTING reads at/below TURBO_VALVE_OPEN_RPM_MAX
    turbo_opto_on_since = None       # For turbo_spinup_check (cross-sensor: opto vs RPM)
    turbo_confirmed_healthy = False  # For turbo_rpm_drop_check
    turbo_dropped_since = None       # For turbo_rpm_drop_check
    mfc_stalled_since = None         # For mfc_flow_check (cross-sensor: commanded vs measured flow)
    dac_saturated_since = None       # For dac_saturation_flow_check
    pressure_diverging_since = None  # For pressure_convergence_check (cross-sensor: PID vs achieved)
    sensor_range_bad_ticks = 0       # For sensor_range_check (global gate: physically impossible reading)
    consecutive_exceptions = 0      # For the global gate's exception-limit escalation
    plasma_detected = False          # For plasma_detect_step (PZEM current-draw hysteresis)
    pzem_confirm_ticks = 0           # For plasma_detect_step
    pzem_absent_since = None         # For pzem_plasma_absent_check
    pzem_unexpected_since = None     # For pzem_power_unexpected_check
    while not _stop_event.is_set():
        start_time = time.time()  # Track start time for precise loop interval timing
        try:
            if _sim_chamber is not None:
                # Advance the simulated chamber's physics once per tick,
                # before reading any of the (simulated) sensors below --
                # see sim_hardware.ChamberSim.step().
                _sim_chamber.step(POLLING_INTERVAL)

            current = sm.state
            pressure_target_voltage = None  # Set below only in states running the PID loop

            # Read every tick regardless of state (display + plasma detection
            # + auto-confirm below) — same reasoning as turbo_rpm's read
            # further down: this signal isn't specific to any one branch.
            e = pzem.read()
            plasma_detected, pzem_confirm_ticks = plasma_detect_step(
                e["current"], plasma_detected, pzem_confirm_ticks)

            if current == "IDLE":
                p = pirani.read(auto_opto=False)
                pirani.set_opto(False)
                mfc.set_flow(0.0)  # Ensure DAC is driven to 0V when not in use
                mfc.valve_close()
                with _lock:
                    safety_tripped = _state["safety_tripped"]
                # A latched safety trip blocks auto-resume -- otherwise a
                # forced shutdown could be silently undone within one tick
                # whenever pressure is already low (e.g. external roughing
                # pump). See _force_safe_shutdown().
                if not safety_tripped and p["voltage"] <= IDLE_PRESSURE_MAX_VOLTAGE:
                    sm.transition("PUMP_DOWN", pirani_voltage=p["voltage"])
            elif current == "PUMP_DOWN":
                p = pirani.read(auto_opto=False)
                # Transition-based opto: fire HIGH only when voltage first drops below 1.3 V
                if not p["opto_enabled"] and p["voltage"] <= 1.3:
                    pirani.set_opto(True)
                    p["opto_enabled"] = True
                mfc.set_flow(0.0)
                mfc.valve_close()
                # Auto return to IDLE if pressure rises back above threshold (pump failure/leak)
                if p["voltage"] >= IDLE_PRESSURE_MAX_VOLTAGE:
                    sm.transition("IDLE")
            elif current == "READY":
                p = pirani.read(auto_opto=True)
                mfc.set_flow(0.0)  # Ensure DAC is driven to 0V when not in use
                mfc.valve_close()
            elif current == "ARGON_FLUSH":
                p = pirani.read(auto_opto=True)
                mfc.valve_release()
                # PD-loop: raise flow until Pirani reaches target (0.09 mbar ≈ 1.20 V)
                mfc.pressure_control_step(p["voltage"], ARGON_FLUSH_TARGET_VOLTAGE)
                pressure_target_voltage = ARGON_FLUSH_TARGET_VOLTAGE
                with _lock:
                    already_triggered = _state["plasma_ignition_triggered"]
                if p["voltage"] >= ARGON_FLUSH_TARGET_VOLTAGE and not already_triggered:
                    _ignite_plasma()
                    if sm.transition("PLASMA_IGNITING"):
                        with _lock:
                            _state["plasma_ignition_start"] = time.time()
                            _state["plasma_ignition_triggered"] = True
            elif current == "PLASMA_IGNITING":
                p = pirani.read(auto_opto=True)
                mfc.valve_release()
                # Hold pressure at 0.09 mbar while waiting for plasma to ignite
                mfc.pressure_control_step(p["voltage"], ARGON_FLUSH_TARGET_VOLTAGE)
                pressure_target_voltage = ARGON_FLUSH_TARGET_VOLTAGE
                with _lock:
                    t_start = _state["plasma_ignition_start"]
                    auto_confirm_armed = _state["pzem_auto_confirm"]
                # Auto-abort if plasma not confirmed within timeout
                if t_start is not None and (time.time() - t_start) >= PLASMA_IGNITION_TIMEOUT:
                    mfc.set_flow(0.0)
                    mfc.valve_close()
                    sm.transition("READY")
                    with _lock:
                        _state["plasma_ignition_start"] = None
                        _state["plasma_ignition_triggered"] = False
                    _set_error("Plasma ignition timed out; returned to READY.")
                elif auto_confirm_armed and plasma_detected:
                    # Operator-armed (Auto-Confirm toggle, off by default): a
                    # debounced PZEM current-draw detection advances the
                    # state machine the same way the operator's Confirm
                    # Plasma click does — see _complete_plasma_confirmation().
                    _complete_plasma_confirmation("PZEM auto-confirm")
            elif current == "SPUTTER_READY":
                p = pirani.read(auto_opto=True)
                mfc.valve_release()
                with _lock:
                    sputter_target_mbar = _state["sputter_target_mbar"]
                # Convert mbar to ADC voltage via calibration table
                sputter_target_voltage = mbar_to_adc_voltage(sputter_target_mbar)
                mfc.pressure_control_step(p["voltage"], sputter_target_voltage)
                pressure_target_voltage = sputter_target_voltage
            elif current == "SPUTTERING":
                p = pirani.read(auto_opto=True)
                mfc.valve_release()
                with _lock:
                    sputter_target_mbar = _state["sputter_target_mbar"]
                sputter_target_voltage = mbar_to_adc_voltage(sputter_target_mbar)
                mfc.pressure_control_step(p["voltage"], sputter_target_voltage)
                pressure_target_voltage = sputter_target_voltage
            elif current == "VENTING":
                p = pirani.read(auto_opto=False)
                mfc.set_flow(0.0)  # Ensure DAC is driven to 0V during venting operations
                mfc.valve_close()  # Keep MFC valve closed throughout venting
                pirani.set_opto(False)
                # Reaching atmosphere alone does not complete the vent -- see
                # vent_complete_ready(). Recomputed fresh every tick so the
                # pending flag tracks the live pressure reading (e.g. clears
                # itself if pressure were to dip back below atmosphere).
                with _lock:
                    vent_complete_confirmed = _state["vent_complete_confirmed"]
                    _state["vent_complete_pending"] = p["voltage"] >= VENTING_COMPLETE_VOLTAGE
                if vent_complete_ready(p["voltage"], vent_complete_confirmed):
                    if sm.transition("IDLE"):
                        with _lock:
                            _state["vent_complete_pending"] = False
                            _state["vent_complete_confirmed"] = False
            else:
                p = pirani.read(auto_opto=True)
                mfc.valve_release()

            r = turbo_rpm.read()  # Read every tick regardless of state (display + valve gate)

            # Turbo inlet valve: held CLOSED (LOW) at all times, except latched
            # OPEN (HIGH) during VENTING after TURBO_VALVE_CONFIRM_SAMPLES
            # consecutive reads at/below TURBO_VALVE_OPEN_RPM_MAX (see turbo_valve_step).
            # Re-closed automatically on leaving VENTING.
            was_open = turbo_valve_open
            turbo_valve_open, turbo_below_ticks = turbo_valve_step(
                current == "VENTING", r["rpm"],
                turbo_valve_open, turbo_below_ticks)
            if turbo_valve_open != was_open:
                GPIO.output(GPIO_TURBO_VALVE_PIN,
                            GPIO.HIGH if turbo_valve_open else GPIO.LOW)

            m = mfc.read()

            # Cross-sensor consistency checks: catch readings that are each
            # individually "in range" but disagree with each other (see
            # each check's docstring).
            turbo_flag, turbo_opto_on_since = turbo_spinup_check(
                p["opto_enabled"], r["rpm"], turbo_opto_on_since, start_time)
            turbo_drop_flag, turbo_confirmed_healthy, turbo_dropped_since = turbo_rpm_drop_check(
                p["opto_enabled"], r["rpm"], turbo_confirmed_healthy, turbo_dropped_since, start_time)
            mfc_flag, mfc_stalled_since = mfc_flow_check(
                m.get("flow_target", 0.0), m["flow"], mfc_stalled_since, start_time)
            dac_sat_flag, dac_saturated_since = dac_saturation_flow_check(
                m.get("dac_voltage", 0.0), m["flow"], dac_saturated_since, start_time)
            if pressure_target_voltage is not None:
                pressure_flag, pressure_diverging_since = pressure_convergence_check(
                    p["voltage"], pressure_target_voltage, pressure_diverging_since, start_time)
            else:
                pressure_flag, pressure_diverging_since = False, None

            # PZEM cross-sensor checks only run once the meter is actually
            # responding (e["ready"]) -- this sensor is optional hardware
            # that may not be wired up yet, and a permanently-zero reading
            # must never be mistaken for "plasma confirmed absent" (which
            # would force a shutdown on every single sputter attempt before
            # the PZEM is even installed).
            if e["ready"]:
                pzem_absent_flag, pzem_absent_since = pzem_plasma_absent_check(
                    current, plasma_detected, pzem_absent_since, start_time)
                pzem_unexpected_flag, pzem_unexpected_since = pzem_power_unexpected_check(
                    current, plasma_detected, pzem_unexpected_since, start_time)
            else:
                pzem_absent_flag, pzem_absent_since = False, None
                pzem_unexpected_flag, pzem_unexpected_since = False, None

            # Global safety gate: is any ADC-derived voltage physically
            # impossible for this hardware (disconnected/shorted/miswired
            # sensor)? Evaluated every tick regardless of state. Suppressed
            # for STARTUP_GRACE_SECONDS after the poll loop starts -- a bench
            # run showed real analog settling transients (MFC feedback
            # observed at -0.15 to -0.23V for a few seconds after power-on)
            # trip this otherwise, which is a real transient, not a fault.
            in_startup_grace = (start_time - poll_start_time) < STARTUP_GRACE_SECONDS
            if in_startup_grace:
                sensor_range_flag, sensor_range_bad_ticks = False, 0
            else:
                sensor_range_flag, sensor_range_bad_ticks = sensor_range_check(
                    (p["voltage"], m["voltage"], r["voltage"]), sensor_range_bad_ticks)

            # Escalation: any sustained flag above forces the full E-STOP-
            # equivalent cutoff via _force_safe_shutdown(), not just a
            # status message -- these used to be warn-only; now they can
            # actually pull the plug. The trip LATCHES (see
            # _force_safe_shutdown docstring) so it can't be silently
            # undone by IDLE's own auto-resume on the very next tick.
            if turbo_flag:
                _force_safe_shutdown(
                    f"Turbo enabled but RPM still {r['rpm']:.0f} after "
                    f"{TURBO_SPINUP_GRACE_SECONDS:.0f}s — check turbo pump."
                )
            elif turbo_drop_flag:
                _force_safe_shutdown(
                    f"Turbo RPM dropped to {r['rpm']:.0f} (below "
                    f"{TURBO_RPM_STALL_THRESHOLD}) after confirmed healthy spin — "
                    "check turbo pump."
                )
            elif mfc_flag:
                _force_safe_shutdown(
                    f"MFC commanded {m.get('flow_target', 0.0):.0f} sccm but reads "
                    f"{m['flow']:.0f} sccm — check MFC / gas supply."
                )
            elif dac_sat_flag:
                _force_safe_shutdown(
                    f"DAC pegged at {m.get('dac_voltage', 0.0):.2f} V but flow reads "
                    f"only {m['flow']:.0f} sccm — check for a stuck valve or blocked line."
                )
            elif pressure_flag:
                _force_safe_shutdown(
                    f"Pressure not converging on target "
                    f"({p['voltage']:.3f} V vs {pressure_target_voltage:.3f} V) — "
                    "check for a leak or stuck valve."
                )
            elif pzem_absent_flag:
                _force_safe_shutdown(
                    f"Plasma not detected via variac current ({e['current']:.2f} A) "
                    f"while in {current} — check for an arc dropout or supply fault."
                )
            elif pzem_unexpected_flag:
                _force_safe_shutdown(
                    f"Unexpected variac current draw ({e['current']:.2f} A) while in "
                    f"{current} — HV/RF supply may be energized when it shouldn't be."
                )
            elif sensor_range_flag:
                _force_safe_shutdown(
                    f"Sensor reading out of physical range (Pirani {p['voltage']:.3f} V, "
                    f"MFC {m['voltage']:.3f} V, Turbo {r['voltage']:.3f} V) — check wiring."
                )

            # Pass the raw ADC counts into the state machine to satisfy the atmosphere transition rule
            sm.update(p["voltage"], pirani_adc=p["adc"], opto_enabled=p["opto_enabled"])

            with _lock:
                _state.update({
                    "pirani_adc":       p["adc"],
                    "pirani_voltage":   p["voltage"],
                    "opto_enabled":     p["opto_enabled"],
                    "mfc_adc":          m["adc"],
                    "mfc_voltage":      m["voltage"],
                    "mfc_flow":         m["flow"],
                    "mfc_valve_closed": m.get("valve_closed", False),
                    "dac_ready":        m.get("dac_ready", False),
                    "dac_voltage":      m.get("dac_voltage", 0.0),
                    "dac_code":         m.get("dac_code", 0),
                    "flow_target":      m.get("flow_target", 0.0),
                    "turbo_valve_open": turbo_valve_open,
                    "turbo_rpm_adc":    r["adc"],
                    "turbo_rpm_voltage": r["voltage"],
                    "turbo_rpm":        r["rpm"],
                    "pzem_ready":       e["ready"],
                    "pzem_voltage":     e["voltage"],
                    "pzem_current":     e["current"],
                    "pzem_power":       e["power"],
                    "pzem_energy":      e["energy"],
                    "pzem_frequency":   e["frequency"],
                    "pzem_power_factor": e["power_factor"],
                    "plasma_detected":  plasma_detected,
                    "sm_state":         sm.state,
                    "last_tick":        start_time,  # Heartbeat; see _refresh() watchdog check
                })

            # Re-assert the safety cutoff if an E-STOP / vent transition landed
            # mid-tick: the branch above read `current` before the transition and
            # may have called valve_release()/set_flow() *after* the interlock
            # callback already cut everything off. Last write must be the cutoff.
            if sm.state in ("IDLE", "VENTING") and current not in ("IDLE", "VENTING"):
                mfc.set_flow(0.0)
                mfc.valve_close()
                pirani.set_opto(False)

            consecutive_exceptions = 0  # This tick completed cleanly

        except Exception as e:
            # Occasional I2C/EMI glitches are tolerated so the thread never
            # dies from a transient fault -- but CONSECUTIVE_EXCEPTION_LIMIT
            # in a row means something is actually broken, not noise, and
            # forces the same global cutoff as the other gate conditions.
            print(f"[HW ERROR] Polling loop glitch (likely EMI): {e}")
            consecutive_exceptions += 1
            if (time.time() - poll_start_time) < STARTUP_GRACE_SECONDS:
                pass  # Startup settling; see STARTUP_GRACE_SECONDS comment above
            elif consecutive_exceptions >= CONSECUTIVE_EXCEPTION_LIMIT:
                _force_safe_shutdown(
                    f"{consecutive_exceptions} consecutive poll-loop errors "
                    f"(latest: {e}) — check hardware/I2C bus."
                )
            else:
                _set_error(str(e))

        # Enforce consistent execution timing based on configurations
        elapsed = time.time() - start_time
        sleep_time = max(0.01, POLLING_INTERVAL - elapsed)
        time.sleep(sleep_time)


_poll_thread = threading.Thread(target=_poll, daemon=True)
_poll_thread.start()


# ════════════════════════════════════════════════════════
#  TKINTER GUI
# ════════════════════════════════════════════════════════
root = tk.Tk()
root.title("Sputter Vacuum Controller — SIMULATION" if SIM_MODE else "Sputter Vacuum Controller")
# resizable(False, False) is deliberately NOT set here yet -- on this Tk
# build, locking resizability before any widgets exist pins the native
# window frame at whatever near-empty size it has at that instant (Tk's
# internal "requested size" bookkeeping keeps tracking the real content
# size correctly, but the actual OS-level window never grows to match).
# Set once, after every widget below has been created and gridded --
# see the call right before root.mainloop().
root.columnconfigure(0, weight=1)
root.columnconfigure(1, weight=1)

# ── IP bar ───────────────────────────────────────────
ip_frame = tk.Frame(root)
ip_frame.grid(row=0, column=0, columnspan=2, padx=12, pady=(10, 0), sticky="ew")
tk.Label(ip_frame, text="Pi IP  :", font=("Courier", 13, "bold")).pack(side="left")
tk.Label(ip_frame, text=_get_ip(), font=("Courier", 13), fg="blue").pack(side="left")
if SIM_MODE:
    # Unmistakable, always-visible: never let a simulated run be confused
    # with real hardware control.
    tk.Label(ip_frame, text="  SIMULATION MODE — no hardware attached  ",
             font=("Courier", 13, "bold"), fg="white", bg="#FFA500").pack(side="right")

# ── State display ─────────────────────────────────────
sf = tk.LabelFrame(root, text=" Process State ", padx=10, pady=8)
sf.grid(row=1, column=0, padx=12, pady=(10, 6), sticky="ew")
lbl_state = tk.Label(sf, text="IDLE", font=("Courier", 20, "bold"),
                     width=16, relief="sunken", fg="grey")
lbl_state.grid(row=0, column=0, columnspan=2)

# Safety-trip banner: hidden (grid_remove) unless a global-gate/cross-sensor
# check has forced a shutdown. A forced shutdown alone isn't enough to stop
# the process -- IDLE's own auto-resume-to-PUMP_DOWN logic would otherwise
# silently undo it within one tick. This banner + Clear Fault button is the
# visible, deliberate acknowledgment gate that unblocks that auto-resume.
lbl_safety_trip = tk.Label(
    sf, text="", font=("Courier", 12, "bold"),
    fg="white", bg="red", wraplength=280, justify="left", anchor="w",
)
btn_clear_fault = tk.Button(
    sf, text="Clear Fault", font=("Courier", 12, "bold"),
    fg="white", bg="red", command=lambda: _on_clear_fault()
)

# Status/error bar — lives in the Process State frame (row 3, below the
# safety-trip banner slot) rather than at the bottom of the window: with
# the taller graphs/fonts the window can exceed the display height, and a
# bottom-anchored bar would be the first thing scrolled off-screen. Kept
# near the top so it's always visible regardless of window height.
lbl_status = tk.Label(sf, text="OK", font=("Courier", 12),
                      anchor="w", width=32, fg="grey")
lbl_status.grid(row=3, column=0, columnspan=2, sticky="ew", pady=(6, 0))

# ── Control buttons ───────────────────────────────────
bf = tk.LabelFrame(root, text=" Controls ", padx=10, pady=8)
bf.grid(row=2, column=0, padx=12, pady=(0, 6), sticky="ew")

btn_flush = tk.Button(
    bf,
    text="Start Argon Flush",
    width=16,
    font=("Courier", 12),
    command=_on_start_argon_flush
)

btn_confirm_plasma = tk.Button(
    bf,
    text="Confirm Plasma",
    width=16,
    font=("Courier", 12),
    command=_on_confirm_plasma
)
_confirm_plasma_default_bg = btn_confirm_plasma.cget("bg")  # restored in _refresh() once
                                                             # PZEM stops confirming detection

btn_sputter = tk.Button(
    bf,
    text="Start Sputter",
    width=16,
    font=("Courier", 12),
    command=_on_start_sputter
)

btn_stop = tk.Button(
    bf,
    text="Stop Sputter",
    width=16,
    font=("Courier", 12),
    command=lambda: sm.transition("VENTING")
)

btn_vent = tk.Button(
    bf,
    text="Vent",
    width=16,
    font=("Courier", 12),
    command=lambda: sm.transition("VENTING")
)

# Hidden (grid_remove) unless vent_complete_pending -- see
# vent_complete_ready() / _on_confirm_vent_complete(). Atmospheric pressure
# alone no longer completes a vent; this is the explicit acknowledgment
# gate that unblocks it, so the turbo inlet valve can't reclose before the
# operator has actually turned off the primary/roughing pump.
btn_confirm_vent = tk.Button(
    bf, text="Confirm Pump Off", width=16, font=("Courier", 12, "bold"),
    fg="white", bg="#FFA500", command=lambda: _on_confirm_vent_complete()
)

btn_estop = tk.Button(
    bf,
    text="E-STOP",
    width=16,
    font=("Courier", 12, "bold"),
    fg="white",
    bg="red",
    command=sm.emergency_stop
)

# SIM_MODE only: manual control over the simulated variac current -- see
# ChamberSim.set_plasma_struck() / _on_toggle_sim_plasma_strike().
btn_sim_plasma_strike = None
if SIM_MODE:
    btn_sim_plasma_strike = tk.Button(
        bf, text="Plasma Strike (SIM): OFF", width=34, font=("Courier", 12, "bold"),
        fg="black", bg="#dddddd", command=lambda: _on_toggle_sim_plasma_strike()
    )

btn_flush          .grid(row=0, column=0, padx=4, pady=4)
btn_confirm_plasma .grid(row=0, column=1, padx=4, pady=4)
btn_sputter        .grid(row=1, column=0, padx=4, pady=4)
btn_stop           .grid(row=1, column=1, padx=4, pady=4)
btn_vent           .grid(row=2, column=0, padx=4, pady=4)
# btn_confirm_vent intentionally not grid()'d here -- _refresh() grids/
# grid_removes it based on vent_complete_pending, same show/hide pattern
# as btn_clear_fault.
if SIM_MODE:
    btn_sim_plasma_strike.grid(row=3, column=0, columnspan=2, padx=4, pady=4)
btn_estop          .grid(row=4 if SIM_MODE else 3, column=0, columnspan=2, padx=4, pady=(4, 0), sticky="ew")

# ── Pirani frame ──────────────────────────────────────
pf = tk.LabelFrame(root, text=" Pirani Gauge  [A0] ", padx=10, pady=6)
pf.grid(row=3, column=0, padx=12, pady=(6, 6), sticky="nsew")

lbl_p_adc      = tk.Label(pf, text="ADC     : ——",      font=("Courier", 14), anchor="w", width=32)
lbl_p_volt     = tk.Label(pf, text="Voltage : ——.—— V", font=("Courier", 14), anchor="w", width=32)
lbl_p_mbar     = tk.Label(pf, text="Pressure: ——.—— mbar", font=("Courier", 14, "bold"), anchor="w", width=32)
lbl_p_message  = tk.Label(pf, text="",               font=("Courier", 12), anchor="w", width=80, fg="blue")
lbl_p_adc .grid(row=0, sticky="w")
lbl_p_volt.grid(row=1, sticky="w")
lbl_p_mbar.grid(row=2, sticky="w")
lbl_p_message.grid(row=3, sticky="w", pady=(6, 0))

# Row 4 is a deliberately empty, weighted spacer: when this frame is
# stretched taller than its own content (to match the taller Plasma panel
# alongside it), the slack collects here instead of as dead space below
# the graph — pinning the graph to the bottom of the frame like its siblings.
pf.rowconfigure(4, weight=1)
p_canvas = tk.Canvas(pf, height=120, bg="#1a1a1a", highlightthickness=0)
p_canvas.grid(row=5, column=0, sticky="sew", pady=(6, 2))
pf.columnconfigure(0, weight=1)

p_graph = ScrollingGraph(p_canvas, maxlen=GRAPH_MAX_SAMPLES,
                         min_val=GRAPH_PIRANI_MIN, max_val=GRAPH_PIRANI_MAX,
                         line_color=GRAPH_PIRANI_COLOR, unit="V")

# ── MFC frame ─────────────────────────────────────────
mff = tk.LabelFrame(root, text=" MFC  Argon Flow  [A1] ", padx=10, pady=6)
mff.grid(row=4, column=0, padx=12, pady=(0, 6), sticky="nsew")

lbl_m_adc    = tk.Label(mff, text="ADC     : ——",            font=("Courier", 14), anchor="w", width=32)
lbl_m_volt   = tk.Label(mff, text="Voltage : ——.—— V",       font=("Courier", 14), anchor="w", width=32)
lbl_m_flow   = tk.Label(mff, text="Flow    : ——.—— sccm Ar", font=("Courier", 16, "bold"), anchor="w", width=32)
lbl_m_dac    = tk.Label(mff, text="DAC     : UNKNOWN",      font=("Courier", 14), anchor="w", width=32)
lbl_m_valve  = tk.Label(mff, text="Valve   : RELEASED",      font=("Courier", 16, "bold"), anchor="w", width=32)
lbl_m_sputter_target  = tk.Label(mff, text="Sputter P (mbar):", font=("Courier", 14), anchor="w")
entry_sputter_target  = tk.Entry(mff, width=10, font=("Courier", 14))
btn_sputter_target    = tk.Button(mff, text="Set", font=("Courier", 12), command=lambda: _update_sputter_target())
lbl_sputter_set_val   = tk.Label(mff, text="Set: 0.007 mbar", font=("Courier", 12), fg="black", anchor="w")
lbl_m_argon  = tk.Label(mff, text="Argon PSI:",            font=("Courier", 14), anchor="w")
entry_argon  = tk.Entry(mff, width=10, font=("Courier", 14))
btn_argon    = tk.Button(mff, text="Update", font=("Courier", 12), command=lambda: _update_argon_pressure())
lbl_argon_set_val     = tk.Label(mff, text="Set: 0.0 psi", font=("Courier", 12), fg="black", anchor="w")

lbl_m_adc   .grid(row=0, sticky="w")
lbl_m_volt  .grid(row=1, sticky="w")
lbl_m_flow  .grid(row=2, sticky="w", pady=(6, 0))
lbl_m_dac   .grid(row=3, sticky="w", pady=(6, 0))
lbl_m_valve .grid(row=4, sticky="w", pady=(6, 0))

lbl_m_sputter_target.grid(row=5, column=0, sticky="w", pady=(6, 0))
entry_sputter_target.grid(row=5, column=1, sticky="w", pady=(6, 0))
btn_sputter_target.grid(row=5, column=2, padx=(6, 0), pady=(6, 0))
lbl_sputter_set_val.grid(row=5, column=3, sticky="w", padx=(8, 0), pady=(6, 0))
entry_sputter_target.bind("<Return>", lambda e: _update_sputter_target())
lbl_m_argon.grid(row=6, column=0, sticky="w", pady=(6, 0))
entry_argon.grid(row=6, column=1, sticky="w", pady=(6, 0))
btn_argon.grid(row=6, column=2, padx=(6, 0), pady=(6, 0))
lbl_argon_set_val.grid(row=6, column=3, sticky="w", padx=(8, 0), pady=(6, 0))
entry_argon.bind("<Return>", lambda e: _update_argon_pressure())

# Row 8 is a deliberately empty, weighted spacer — see the matching
# comment on pf.rowconfigure(6, ...) above; same reasoning here.
mff.rowconfigure(8, weight=1)
m_canvas = tk.Canvas(mff, height=120, bg="#1a1a1a", highlightthickness=0)
m_canvas.grid(row=9, column=0, columnspan=4, sticky="sew", pady=(6, 2))
mff.columnconfigure(0, weight=1)

# Flow (solid, cyan, left-scale sccm) overlaid with DAC output voltage
# (dashed, yellow, right-scale volts) on the same canvas.
m_graph = ScrollingGraph(m_canvas, maxlen=GRAPH_MAX_SAMPLES,
                         min_val=GRAPH_MFC_MIN, max_val=GRAPH_MFC_MAX,
                         line_color=GRAPH_MFC_COLOR, unit="sccm",
                         secondary_min=0.0, secondary_max=ARGON_DAC_VREF,
                         secondary_color="#FFFF00", secondary_unit="V DAC")

# ── Plasma frame (PZEM-004T-100A energy meter, variac output) ─────────
plf = tk.LabelFrame(root, text=" Plasma  [PZEM variac current] ", padx=10, pady=6)
plf.grid(row=1, column=1, rowspan=3, padx=12, pady=(10, 6), sticky="nsew")
plf.columnconfigure(0, weight=1)

lbl_plasma_detected = tk.Label(plf, text="PLASMA  : ——", font=("Courier", 16, "bold"),
                               anchor="w", width=30)
btn_auto_confirm = tk.Button(
    plf, text="Auto-Confirm: OFF", font=("Courier", 11, "bold"),
    fg="black", bg="#dddddd", command=lambda: _on_toggle_auto_confirm()
)
lbl_plasma_detected.grid(row=0, column=0, sticky="w")
btn_auto_confirm   .grid(row=0, column=1, sticky="e", padx=(6, 0))

lbl_pzem_ready = tk.Label(plf, text="PZEM    : NOT DETECTED", font=("Courier", 12, "bold"),
                          fg="red", anchor="w", width=30)
lbl_pzem_v  = tk.Label(plf, text="Voltage : ———.— V",   font=("Courier", 13), anchor="w", width=30)
lbl_pzem_i  = tk.Label(plf, text="Current : —.—— A",     font=("Courier", 13), anchor="w", width=30)
lbl_pzem_p  = tk.Label(plf, text="Power   : ————.— W",   font=("Courier", 13), anchor="w", width=30)
lbl_pzem_e  = tk.Label(plf, text="Energy  : ———————— Wh", font=("Courier", 13), anchor="w", width=30)
lbl_pzem_fpf = tk.Label(plf, text="Freq/PF : ——.— Hz  —.—— PF", font=("Courier", 13), anchor="w", width=30)

lbl_pzem_ready.grid(row=1, sticky="w", pady=(6, 0))
lbl_pzem_v    .grid(row=2, sticky="w")
lbl_pzem_i    .grid(row=3, sticky="w")
lbl_pzem_p    .grid(row=4, sticky="w")
lbl_pzem_e    .grid(row=5, sticky="w")
lbl_pzem_fpf  .grid(row=6, sticky="w")

# Row 7 is a deliberately empty, weighted spacer — this frame is the
# tallest of the four (it's what forces its root-grid rowspan taller than
# sf+bf+pf's own natural height in the first place), but keep the same
# spacer pattern for consistency so its graph stays bottom-aligned with
# the others too if content above it ever changes.
plf.rowconfigure(7, weight=1)
pzem_canvas = tk.Canvas(plf, height=140, bg="#1a1a1a", highlightthickness=0)
pzem_canvas.grid(row=8, column=0, columnspan=2, sticky="sew", pady=(6, 2))

# One graph, all live PZEM stats, independently scaled per series with a
# legend row -- cumulative Energy (Wh) is deliberately left off (monotonic
# session total, not a comparable scale to the others) and shown as the
# numeric readout above instead.
pzem_graph = ScrollingGraph(
    pzem_canvas, maxlen=GRAPH_MAX_SAMPLES,
    series=[
        {"name": "V",  "color": "#00FFFF",    "min": 0.0, "max": GRAPH_PZEM_VOLTAGE_MAX, "unit": "V"},
        {"name": "I",  "color": "#FFFF00",  "min": 0.0, "max": GRAPH_PZEM_CURRENT_MAX, "unit": "A"},
        {"name": "P",  "color": "#FF00FF", "min": 0.0, "max": GRAPH_PZEM_POWER_MAX,   "unit": "W"},
        {"name": "Hz", "color": "#00FF00",    "min": GRAPH_PZEM_FREQ_MIN, "max": GRAPH_PZEM_FREQ_MAX, "unit": "Hz"},
        {"name": "PF", "color": "#FFA500",  "min": 0.0, "max": 1.0, "unit": ""},
    ],
)

# ── Turbo Pump frame ──────────────────────────────────
# Beside MFC (col0, row4), below the Plasma frame (col1, rows1-3).
tpf = tk.LabelFrame(root, text=" Turbo Pump  [A2] ", padx=10, pady=6)
tpf.grid(row=4, column=1, padx=12, pady=(0, 6), sticky="nsew")
tpf.columnconfigure(0, weight=1)

lbl_turbo_rpm      = tk.Label(tpf, text="T-RPM   : ——.—— RPM", font=("Courier", 14), anchor="w", width=32)
lbl_turbo_rpm_volt = tk.Label(tpf, text="T-RPM V : —.——— V",   font=("Courier", 14), anchor="w", width=32)
# Turbo enable opto (GPIO 17) and inlet valve (GPIO 22) state — moved here
# from the Pirani frame since both are turbo-side actuators/sensors, not
# pressure readings.
lbl_opto           = tk.Label(tpf, text="OPTO    : ——",      font=("Courier", 16, "bold"), anchor="w", width=32)
lbl_tvalve         = tk.Label(tpf, text="T-VALVE : ——",      font=("Courier", 16, "bold"), anchor="w", width=32)
lbl_turbo_rpm     .grid(row=0, sticky="w")
lbl_turbo_rpm_volt.grid(row=1, sticky="w")
lbl_opto          .grid(row=2, sticky="w", pady=(6, 0))
lbl_tvalve        .grid(row=3, sticky="w")

# Row 4 is a deliberately empty, weighted spacer — see the matching
# comment on pf.rowconfigure(4, ...) above; same reasoning here.
tpf.rowconfigure(4, weight=1)
t_canvas = tk.Canvas(tpf, height=120, bg="#1a1a1a", highlightthickness=0)
t_canvas.grid(row=5, column=0, sticky="sew", pady=(6, 2))

t_graph = ScrollingGraph(t_canvas, maxlen=GRAPH_MAX_SAMPLES,
                         min_val=GRAPH_TURBO_RPM_MIN, max_val=GRAPH_TURBO_RPM_MAX,
                         line_color=GRAPH_TURBO_RPM_COLOR, unit="RPM")


# ════════════════════════════════════════════════════════
#  BUTTON STATE MAP
# ════════════════════════════════════════════════════════
# Button order: flush, confirm_plasma, sputter, stop, vent, estop
BUTTON_STATES = {
    "IDLE":            ("disabled", "disabled", "disabled", "disabled", "disabled", "normal"),
    "PUMP_DOWN":       ("disabled", "disabled", "disabled", "disabled", "disabled", "normal"),
    "READY":           ("normal",   "disabled", "disabled", "disabled", "normal",   "normal"),
    "ARGON_FLUSH":     ("disabled", "disabled", "disabled", "disabled", "normal",   "normal"),
    "PLASMA_IGNITING": ("disabled", "normal",   "disabled", "disabled", "normal",   "normal"),
    "SPUTTER_READY":   ("disabled", "disabled", "normal",   "normal",   "normal",   "normal"),
    "SPUTTERING":      ("disabled", "disabled", "disabled", "normal",   "normal",   "normal"),
    "VENTING":         ("disabled", "disabled", "disabled", "disabled", "disabled", "normal"),
}


# ════════════════════════════════════════════════════════
#  GUI REFRESH
# ════════════════════════════════════════════════════════

def _update_sputter_target():
    raw = entry_sputter_target.get().strip()
    try:
        v = float(raw)
    except ValueError:
        _set_error("Invalid sputter target; enter pressure in mbar.")
        return
    if v <= 0.0:
        _set_error("Sputter target must be > 0 mbar.")
        return
    if v > SPUTTER_TARGET_MAX_MBAR:
        messagebox.showwarning(
            "Sputter target too high",
            f"{v:.4f} mbar exceeds the {SPUTTER_TARGET_MAX_MBAR:.2f} mbar limit for Sputter P.\n"
            "Check you didn't enter an Argon PSI value in this field."
        )
        _set_error(f"Sputter target {v:.4f} mbar exceeds the {SPUTTER_TARGET_MAX_MBAR:.2f} mbar limit.")
        return
    with _lock:
        _state["sputter_target_mbar"] = v
    _set_error("")
    lbl_sputter_set_val.config(text=f"Set: {v:.4f} mbar")

def _update_argon_pressure():
    raw_value = entry_argon.get().strip()
    try:
        value = float(raw_value)
    except ValueError:
        _set_error("Invalid argon pressure; enter a number in psi.")
        return
    if value < ARGON_PRESSURE_MIN_PSI:
        messagebox.showwarning(
            "Argon pressure too low",
            f"{value:.2f} psi is below the {ARGON_PRESSURE_MIN_PSI:.0f} psi minimum for Argon PSI.\n"
            "Check you didn't enter a Sputter P value in this field."
        )
        _set_error(f"Argon pressure {value:.2f} psi is below the {ARGON_PRESSURE_MIN_PSI:.0f} psi minimum.")
        return

    with _lock:
        _state["argon_pressure"] = value
    _set_error("")
    lbl_argon_set_val.config(text=f"Set: {value:.1f} psi")


def _ignite_plasma():
    """Trigger plasma ignition via control signal.

    Integrates with RF power supply, pressure feedback, and safety interlocks.
    Placeholder for hardware control implementation.
    """
    # TODO: implement RF trigger/control logic when hardware is defined
    pass


# Tracks the currently displayed error and when it first appeared (GUI thread only)
_error_display = {"text": "", "since": 0.0}

# Tracks whether the modal safety-trip alert has already been shown for the
# CURRENT trip, so it pops once per trip (rising edge) rather than every
# refresh tick while the operator hasn't yet clicked Clear Fault.
_safety_trip_alert_shown = False


def _refresh():
    with _lock:
        s = dict(_state)

    # — Heartbeat watchdog —
    # If the poll thread hasn't updated last_tick in over
    # HEARTBEAT_TIMEOUT_SECONDS, it's presumed stalled/hung. Checked from
    # the GUI thread specifically because a hung poll thread can't detect
    # its own hang. Forces the same global cutoff as every other gate
    # condition; safe to call every refresh for as long as the stall
    # persists (idempotent GPIO/state-machine calls, de-duped logging).
    stall_time = time.time() - s["last_tick"]
    if stall_time > HEARTBEAT_TIMEOUT_SECONDS:
        _force_safe_shutdown(
            f"Poll loop stalled (no update in {stall_time:.1f}s) — "
            "restart the application."
        )
        with _lock:
            s = dict(_state)

    # — Safety trip banner —
    # Latched by _force_safe_shutdown(); see that docstring for why a bare
    # emergency_stop() isn't enough. The modal only pops on the rising edge
    # (global _safety_trip_alert_shown) so it doesn't re-appear every 200ms
    # while the operator is looking at it or has already dismissed it.
    global _safety_trip_alert_shown
    if s["safety_tripped"]:
        lbl_safety_trip.config(text=f"SAFETY TRIP:\n{s['safety_trip_reason']}")
        lbl_safety_trip.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(6, 0))
        btn_clear_fault.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(4, 0))
        if not _safety_trip_alert_shown:
            _safety_trip_alert_shown = True
            messagebox.showerror("Safety shutdown", s["safety_trip_reason"])
    else:
        lbl_safety_trip.grid_remove()
        btn_clear_fault.grid_remove()
        _safety_trip_alert_shown = False

    # — State —
    st = s["sm_state"]
    lbl_state.config(text=st, fg=STATE_COLORS.get(st, "grey"))

    # — Buttons —
    bs = BUTTON_STATES.get(st, BUTTON_STATES["IDLE"])
    btn_flush          .config(state=bs[0])
    btn_confirm_plasma .config(state=bs[1])
    btn_sputter        .config(state=bs[2])
    btn_stop           .config(state=bs[3])
    btn_vent           .config(state=bs[4])
    btn_estop          .config(state=bs[5])

    # Confirm Plasma highlights (hybrid UI cue) once PZEM confirms a
    # plasma-level current draw during PLASMA_IGNITING — operator still has
    # to click unless Auto-Confirm is armed, in which case _poll() has
    # already advanced the state before this paints.
    if st == "PLASMA_IGNITING" and s["plasma_detected"]:
        btn_confirm_plasma.config(bg="lightgreen")
    else:
        btn_confirm_plasma.config(bg=_confirm_plasma_default_bg)

    btn_auto_confirm.config(
        text = "Auto-Confirm: ON" if s["pzem_auto_confirm"] else "Auto-Confirm: OFF",
        bg   = "#FFA500"           if s["pzem_auto_confirm"] else "#dddddd",
    )

    # Confirm Pump Off — shown only while atmospheric pressure has been
    # reached during VENTING but the operator hasn't yet confirmed the
    # primary/roughing pump is off (see vent_complete_ready()). Same
    # show/hide pattern as btn_clear_fault above.
    if s["vent_complete_pending"]:
        btn_confirm_vent.grid(row=2, column=1, padx=4, pady=4)
    else:
        btn_confirm_vent.grid_remove()

    # — Pirani —
    lbl_p_adc .config(text=f"ADC     : {s['pirani_adc']}")
    lbl_p_volt.config(text=f"Voltage : {s['pirani_voltage']:.6f} V")
    mbar = adc_voltage_to_mbar(s["pirani_voltage"])
    if mbar >= 999:
        lbl_p_mbar.config(text="Pressure: ATM (>999 mbar)")
    else:
        lbl_p_mbar.config(text=f"Pressure: {mbar:.3g} mbar")
    lbl_opto  .config(
        text = "OPTO    : ON " if s["opto_enabled"] else "OPTO    : OFF",
        fg   = "green"         if s["opto_enabled"] else "red",
    )
    lbl_tvalve.config(
        text = "T-VALVE : OPEN (venting)" if s["turbo_valve_open"] else "T-VALVE : CLOSED",
        fg   = "#FFA500"                   if s["turbo_valve_open"] else "green",
    )
    lbl_turbo_rpm.config(text=f"T-RPM   : {s['turbo_rpm']:.0f} RPM")
    lbl_turbo_rpm_volt.config(text=f"T-RPM V : {s['turbo_rpm_voltage']:.3f} V")
    t_graph.push(s["turbo_rpm"])
    t_graph.draw()

    # — PZEM / Plasma —
    lbl_plasma_detected.config(
        text = "PLASMA  : DETECTED"     if s["plasma_detected"] else "PLASMA  : not detected",
        fg   = "green"                  if s["plasma_detected"] else "grey",
    )
    lbl_pzem_ready.config(
        text = "PZEM    : OK" if s["pzem_ready"] else "PZEM    : NOT DETECTED",
        fg   = "green"        if s["pzem_ready"] else "red",
    )
    lbl_pzem_v.config(text=f"Voltage : {s['pzem_voltage']:7.1f} V")
    lbl_pzem_i.config(text=f"Current : {s['pzem_current']:6.2f} A")
    lbl_pzem_p.config(text=f"Power   : {s['pzem_power']:8.1f} W")
    lbl_pzem_e.config(text=f"Energy  : {s['pzem_energy']:10.0f} Wh")
    lbl_pzem_fpf.config(
        text=f"Freq/PF : {s['pzem_frequency']:4.1f} Hz  {s['pzem_power_factor']:.2f} PF")
    pzem_graph.push_series({
        "V":  s["pzem_voltage"],
        "I":  s["pzem_current"],
        "P":  s["pzem_power"],
        "Hz": s["pzem_frequency"],
        "PF": s["pzem_power_factor"],
    })
    pzem_graph.draw()

    p_message_color = "blue"
    if st == "IDLE":
        p_message = "IDLE: MFC valve closed, turbo opto off."
    elif st == "PUMP_DOWN":
        if s["opto_enabled"]:
            p_message = "Turbo opto ON at {:.4f} V; pump down continues.".format(s["pirani_voltage"])
        else:
            p_message = "Pump down in progress; turbo opto will enable when pressure drops sufficiently."
    elif st == "READY":
        p_message = "READY: Start Argon Flush once inlet pressure is ≥ 15 psi."
    elif st == "ARGON_FLUSH":
        if s["pirani_voltage"] >= ARGON_FLUSH_TARGET_VOLTAGE:
            p_message = "Target 0.09 mbar reached; igniting plasma automatically."
        else:
            p_message = "Argon flush: controlling flow to reach 0.09 mbar (now {:.3f} V).".format(s["pirani_voltage"])
    elif st == "PLASMA_IGNITING":
        elapsed = time.time() - s["plasma_ignition_start"] if s["plasma_ignition_start"] else 0
        remaining = max(0, int(PLASMA_IGNITION_TIMEOUT - elapsed))
        p_message = f"Waiting for plasma ignition ({remaining}s remaining). Confirm or wait for auto-abort."
    elif st == "SPUTTER_READY":
        p_message = "SPUTTER READY: Plasma stable at target pressure. Press Start Sputter."
    elif st == "VENTING":
        if s["vent_complete_pending"]:
            p_message = "Atmosphere reached — turn OFF the primary/roughing pump, then click Confirm Pump Off."
            p_message_color = "red"
        elif s["turbo_rpm"] <= VENTING_PUMP_OFF_PROMPT_RPM:
            p_message = "Turbo has spun down — turn OFF the primary/roughing pump now."
            p_message_color = "#FFA500"
        else:
            p_message = f"Venting: turbo at {s['turbo_rpm']:.0f} RPM, waiting for it to spin down."
    else:
        p_message = ""

    lbl_p_message.config(text=p_message, fg=p_message_color)
    p_graph.push(s["pirani_voltage"])
    p_graph.draw()

    # — MFC —
    lbl_m_adc .config(text=f"ADC     : {s['mfc_adc']}")
    lbl_m_volt.config(text=f"Voltage : {s['mfc_voltage']:.6f} V")
    lbl_m_flow.config(text=f"Flow    : {s['mfc_flow']:.2f} sccm Ar")
    lbl_m_dac.config(
        text=f"DAC OUT : {s['dac_voltage']:.3f} V ({s['dac_code']})",
        fg   = "green" if s.get('dac_ready') else "red",
    )
    lbl_m_valve.config(
        text = "Valve   : CLOSED" if s['mfc_valve_closed'] else "Valve   : RELEASED",
        fg   = "red" if s['mfc_valve_closed'] else "green",
    )
    m_graph.push(
        s['mfc_flow'],
        s['dac_voltage']
    )
    m_graph.draw()

    # — Status —
    # Errors are no longer wiped by the polling thread; expire them here after
    # ERROR_DISPLAY_SECONDS so the operator has time to read them but a single
    # transient glitch doesn't leave the status bar red forever.
    if s["error"]:
        now = time.time()
        if s["error"] != _error_display["text"]:
            _error_display["text"]  = s["error"]
            _error_display["since"] = now
        elif now - _error_display["since"] >= ERROR_DISPLAY_SECONDS:
            with _lock:
                if _state["error"] == s["error"]:
                    _state["error"] = ""
            s["error"] = ""
            _error_display["text"] = ""
    else:
        _error_display["text"] = ""

    lbl_status.config(
        text = f"ERR: {s['error']}" if s["error"] else "OK",
        fg   = "red"                if s["error"] else "grey",
    )

    root.after(GUI_REFRESH_INTERVAL, _refresh)


# ════════════════════════════════════════════════════════
#  CLEAN SHUTDOWN
# ════════════════════════════════════════════════════════
def _on_close():
    _stop_event.set()
    _poll_thread.join(timeout=2.0)  # Wait for poll thread to exit cleanly
    mfc.set_flow(0.0)
    mfc.valve_close()
    pirani.shutdown()
    pzem.close()
    GPIO.cleanup()
    root.destroy()


root.protocol("WM_DELETE_WINDOW", _on_close)

_refresh()
# Lock resizing only now that every widget above has been created and
# gridded -- see the comment next to root.columnconfigure() near the top
# of the GUI section for why this can't happen any earlier.
root.update_idletasks()
root.resizable(False, False)
root.mainloop()
