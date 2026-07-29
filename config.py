# config.py — Centralized configuration for Sputter Controller

# ════════════════════════════════════════════════════════
#  GPIO PINS (BCM numbering)
# ════════════════════════════════════════════════════════
GPIO_PIRANI_PIN           = 17   # Input from Pirani gauge / control line for opto logic
GPIO_MFC_VALVE_CLOSE_PIN  = 27   # MFC valve close (emergency shut)
GPIO_TURBO_VALVE_PIN      = 22   # Turbo inlet valve relay (BC547 driver, valve on NC
                                 # contact): GPIO HIGH = valve OPEN, LOW = valve CLOSED.
                                 # Held closed at all times; opened only at/below
                                 # TURBO_VALVE_OPEN_RPM_MAX during VENTING.
                                 # Physical pin 15.
TURBO_VALVE_OPEN_RPM_MAX  = 35000  # During VENTING, open the turbo inlet valve once
                                 # turbo pump RPM (tach on ADC A2) drops to or below this
                                 # speed (latched until the state machine leaves VENTING).
                                 # Replaces the earlier pressure-based (mbar) trigger —
                                 # RPM reflects rotor state directly instead of inferring
                                 # it from chamber pressure.
TURBO_VALVE_CONFIRM_SAMPLES = 3  # Consecutive polling reads at/below TURBO_VALVE_OPEN_RPM_MAX
                                 # required before the valve opens (3 × 0.2 s poll = 0.6 s).
                                 # A corrupted read on the software I2C bus (EMI bit-flips
                                 # return garbage without raising) must never open the valve.
VENTING_PUMP_OFF_PROMPT_RPM = 2000  # During VENTING, once turbo RPM drops to/below
                                 # this, the GUI prompts the operator to turn off the
                                 # primary/roughing pump (main.py's _refresh()). Separate
                                 # from TURBO_RPM_STALL_THRESHOLD -- this is an operator
                                 # cue during a normal vent, not a fault threshold.

# ════════════════════════════════════════════════════════
#  I2C BUS
# ════════════════════════════════════════════════════════
I2C_BUS_NUMBER = 3   # software i2c-gpio on GPIO23/24 (physical pins 16/18),
                     # enabled via dtoverlay in /boot/firmware/config.txt

# ════════════════════════════════════════════════════════
#  ADC CONFIGURATION
# ════════════════════════════════════════════════════════
ADC_VREF  = 3.33      # Reference voltage (volts)
ADC_GAIN  = 1         # ADS1115 gain setting

# ADC Channel Assignment
ADC_CHANNEL_PIRANI    = 0   # Pirani gauge on A0
ADC_CHANNEL_MFC       = 1   # MFC flow on A1
ADC_CHANNEL_TURBO_RPM = 2   # Turbo pump RPM tach output on A2

# ════════════════════════════════════════════════════════
#  TURBO PUMP RPM (tach output on ADC A2)
# ════════════════════════════════════════════════════════
TURBO_RPM_VOLTAGE_FULL_SCALE = 3.3      # Volts at A2 for full-scale RPM
TURBO_RPM_FULL_SCALE         = 90000.0  # RPM at TURBO_RPM_VOLTAGE_FULL_SCALE

# ════════════════════════════════════════════════════════
#  MFC CONTROLLER PARAMETERS
# ════════════════════════════════════════════════════════
MFC_FULL_SCALE = 700.0    # Full-scale flow in sccm (MKS 1179A Ar: 0–5V setpoint = 0–700 sccm)
MFC_SETPOINT_VOLTAGE_FULL_SCALE = 5.0  # Volts at the MFC's analog setpoint pin for full-scale flow
MFC_GAS_CORRECTION_FACTOR = 1.39  # For Argon

# ════════════════════════════════════════════════════════
#  TURBO PUMP OPTO THRESHOLDS (ADC values)
# ════════════════════════════════════════════════════════
TURBOOPTO_ON_THRESHOLD  = 10500   # ADC ≤ this: vacuum safe → turbo opto ON
TURBOOPTO_OFF_THRESHOLD = 15000   # ADC ≥ this: vacuum unsafe → turbo opto OFF
                                  # (must be > ON_THRESHOLD)

# ════════════════════════════════════════════════════════
#  STATE MACHINE THRESHOLDS
# ════════════════════════════════════════════════════════
VACUUM_THRESHOLD            = 10500   # ADC ≤ this → PUMP_DOWN to READY (legacy count-based threshold)
ATMOSPHERE_THRESHOLD        = 30000   # ADC ≥ this → VENTING to IDLE (legacy)
VENTING_COMPLETE_VOLTAGE    = 3.0     # Pirani voltage for near-atmosphere; VENTING to IDLE.
                                      # True atmosphere (999 mbar) is ADC saturation at 3.30V
                                      # per the calibration table; 3.0V is a safety margin below
                                      # that so the VENTING -> IDLE transition reliably fires once
                                      # the chamber is genuinely back at atmosphere, rather than
                                      # the old 2.5V (~5 mbar — nowhere near atmosphere, the bug
                                      # this replaces).
IDLE_PRESSURE_MAX_VOLTAGE   = 2.71    # Voltage threshold for safe pump-down start (10 mbar: 8.20V × 0.33)
PUMP_DOWN_COMPLETE_VOLTAGE  = 0.3   # Pirani voltage for ~0.01 mbar; threshold for PUMP_DOWN -> READY transition
ARGON_FLUSH_TARGET_VOLTAGE  = 1.287   # Pirani voltage target for argon flush (0.09 mbar: 3.90V × 0.33)
ARGON_FLUSH_FLOW_SETPOINT   = 150.0   # Initial MFC flow on flush entry; control loop takes over
PRESSURE_CONTROL_KP         = 8.0    # Proportional gain for MFC pressure control loop (sccm/V-error)
PRESSURE_CONTROL_KD         = 3.0     # Derivative gain for MFC pressure control loop (set >0 to dampen oscillation)
PRESSURE_CONTROL_KI         = 0.005     # Integral gain (sccm/(V.s))
PRESSURE_CONTROL_ICLAMP     = 50.0    # Anti-windup clamp (sccm)
PLASMA_IGNITION_TIMEOUT     = 120.0   # Seconds to wait for manual plasma confirmation before returning to READY
SPUTTER_READY_TARGET_VOLTAGE = 0.6  # Pirani voltage target for sputtering (0.007 mbar: 1.10V × 0.33)
SPUTTER_TARGET_MAX_MBAR     = 0.05    # Upper bound on the operator-settable Sputter P entry field.
                                      # Also catches an Argon PSI value (tens) fat-fingered into
                                      # this field, since any such value is far above this cap.
ARGON_PRESSURE_MIN_PSI      = 15.0    # Lower bound on the operator-settable Argon PSI entry field
                                      # (also the minimum required to start flush/sputter). Also
                                      # catches a Sputter P mbar value fat-fingered into this field,
                                      # since any such value is far below this floor.
ARGON_DAC_I2C_ADDRESS       = 0x60    # I2C address for the argon MFC DAC (MCP4725 default)
ARGON_DAC_VREF              = 5.0     # Max DAC output after the op-amp level-shifter stage —
                                      # matches MFC_SETPOINT_VOLTAGE_FULL_SCALE, so commanded
                                      # flow covers the MFC's full 0-MFC_FULL_SCALE range.
ARGON_DAC_RESOLUTION        = 4096    # DAC resolution for MCP4725 / 12-bit output
ADS1115_I2C_ADDRESS         = 0x48    # ADS1115 I2C address (default for most boards)

# ════════════════════════════════════════════════════════
#  CROSS-SENSOR CONSISTENCY CHECKS
# ════════════════════════════════════════════════════════
# Each pair of readings here is individually "in range" but can still
# disagree with each other in a way a single-value threshold can't catch
# (e.g. we told the turbo to spin up, but RPM says it never did). All
# three feed the global safety gate below: once flagged, they force a full
# hardware cutoff, not just a status-bar warning. Grace periods are
# conservative placeholders (favor fewer false positives over noise from
# normal settling time) -- tune once measured against real spin-up/
# response/settling behavior on the bench. Because a false positive here
# now forces a real shutdown (not just a warning), treat these as
# unvalidated until confirmed on the bench.
TURBO_SPINUP_GRACE_SECONDS   = 60.0    # How long the turbo enable opto can be
                                       # ON before RPM must show real spin-up
TURBO_RPM_STALL_THRESHOLD    = 10000   # RPM below this counts as "not
                                       # spinning" even with opto enabled.
                                       # Raised from 1000 -> 10000 after a
                                       # bench run: a healthy turbo should
                                       # clear this well within the grace
                                       # window, so a threshold this low let
                                       # a genuinely-not-spinning turbo pass.
TURBO_RPM_DROP_GRACE_SECONDS = 10.0    # See turbo_rpm_drop_check: once RPM has
                                       # confirmed healthy spin (exceeded
                                       # TURBO_RPM_STALL_THRESHOLD), how long
                                       # it may stay dropped back below that
                                       # before flagging. Shorter than the
                                       # spin-up grace -- a regression after
                                       # already running is a sharper signal
                                       # than still-ramping-up.

MFC_FLOW_MIN_COMMANDED_SCCM  = 20.0    # Ignore small commanded flows (PID
                                       # settling noise near zero)
MFC_FLOW_RESPONSE_FRACTION   = 0.2     # Measured flow must reach at least
                                       # this fraction of commanded flow
MFC_FLOW_GRACE_SECONDS       = 5.0     # How long commanded can exceed
                                       # measured flow before flagging

DAC_SATURATION_MARGIN_V      = 0.1     # Within this of ARGON_DAC_VREF counts
                                       # as "pegged at ceiling"
DAC_SATURATION_FLOW_FRACTION = 0.5     # Measured flow must reach at least
                                       # this fraction of MFC_FULL_SCALE when
                                       # the DAC is pegged, or it's suspicious
DAC_SATURATION_GRACE_SECONDS = 10.0    # How long the DAC may stay pegged
                                       # without flow responding before flagging

PRESSURE_CONVERGENCE_TOLERANCE_V   = 0.3   # |current - target| (ADC volts)
                                            # beyond which "not converging"
                                            # starts being timed
PRESSURE_CONVERGENCE_GRACE_SECONDS = 90.0  # How long the PID loop can run
                                            # without closing in on target.
                                            # Raised from 20.0 after a bench
                                            # run: a large setpoint step (e.g.
                                            # PLASMA_IGNITING's 0.09 mbar target
                                            # -> SPUTTER_READY's much lower one)
                                            # legitimately takes longer than 20s
                                            # to traverse given the deliberately
                                            # gentle, anti-overshoot PID tuning.

# ════════════════════════════════════════════════════════
#  GLOBAL SAFETY GATE (watchdog, sensor sanity, exception limit)
# ════════════════════════════════════════════════════════
# Evaluated every poll tick regardless of current state. Unlike the
# per-condition checks elsewhere, tripping any of these forces the same
# full E-STOP-equivalent cutoff (turbo valve closed, MFC flow/valve/opto
# off, state forced to IDLE) rather than a specific corrective action. A
# trip now LATCHES (see SAFETY_LATCH below) rather than silently clearing
# the moment conditions look fine -- a bench run showed a forced shutdown
# being undone within one tick by IDLE's own auto-resume-to-PUMP_DOWN
# logic, with no visible sign anything had happened.
STARTUP_GRACE_SECONDS        = 5.0     # sensor_range_check and the exception
                                       # limit are suppressed for this long
                                       # after the poll loop starts, since
                                       # analog readings (esp. MFC feedback)
                                       # were observed settling for a few
                                       # seconds after power-on -- a real
                                       # transient, not a fault, that doesn't
                                       # need the same debounce-only handling
                                       # as an ongoing miswired sensor
HEARTBEAT_TIMEOUT_SECONDS    = 2.0     # If the GUI thread sees no poll-tick
                                       # update within this long (10x
                                       # POLLING_INTERVAL), the poll loop is
                                       # presumed stalled/hung -- checked
                                       # from the GUI thread since a hung
                                       # poll thread can't detect its own hang
CONSECUTIVE_EXCEPTION_LIMIT  = 5       # Consecutive poll ticks that may raise
                                       # (transient I2C/EMI glitches) before
                                       # it's treated as a real fault, not
                                       # noise, and forces a cutoff
SANE_VOLTAGE_MIN             = -0.1    # ADC-derived voltages (Pirani, MFC
SANE_VOLTAGE_MAX             = ADC_VREF + 0.2  # feedback, turbo tach) outside
                                       # this band indicate a disconnected,
                                       # shorted, or miswired sensor -- not a
                                       # legitimate reading at any pressure/
                                       # flow/speed
SENSOR_RANGE_CONFIRM_SAMPLES = 5       # Consecutive out-of-range reads
                                       # required before flagging (same
                                       # single-corrupted-sample protection
                                       # as every other check here)

# ════════════════════════════════════════════════════════
#  ENERGY METER (PZEM-004T-100A, plasma ignition sensing)
# ════════════════════════════════════════════════════════
# Reads AC voltage/current/power/energy/frequency/power-factor on the
# sputtering supply's variac output via a CP2102 USB-TTL adapter (Modbus-
# RTU, see drivers/pzem_meter.py). Optional hardware: unlike the ADS1115/MCP4725
# checked in main.py's startup self-test, this sensor may not be wired up
# yet -- PZEMController never raises and reports ready=False until a real
# meter responds, so nothing here fires (see main.py's `if e["ready"]:`
# guard around the two cross-sensor checks below) until it's actually
# installed.
PZEM_SERIAL_PORT   = "/dev/ttyUSB0"  # CP2102 USB-TTL adapter; check `ls /dev/ttyUSB*`
                                      # after plugging in, or use a stable
                                      # /dev/serial/by-id/... symlink if more than
                                      # one USB-serial device is ever attached to the Pi
PZEM_BAUDRATE       = 9600           # fixed by the PZEM-004T v3.0 Modbus-RTU interface
PZEM_SLAVE_ADDRESS  = 0xF8           # factory-default single-device broadcast address;
                                      # only re-address (Modbus function 0x06 to register
                                      # 0x0002) if a second PZEM ever shares this bus
PZEM_SERIAL_TIMEOUT = 0.15           # seconds; deliberately short so a disconnected or
                                      # unresponsive meter can't stall the 0.2s poll tick --
                                      # PZEMController swallows the resulting timeout
                                      # internally rather than raising into the poll loop's
                                      # CONSECUTIVE_EXCEPTION_LIMIT counter, since this
                                      # sensor being absent must never block startup or the
                                      # rest of the control loop

# Plasma-ignition detection: variac-side AC current draw steps up once the
# discharge strikes (load impedance drops). Clamp the CT around the
# variac's OUTPUT line (not its input) and wire the PZEM's own L/N voltage
# sense across that same output -- so voltage/current/power all read what's
# actually delivered to the sputtering supply. See README's Energy Meter
# wiring section.
PZEM_PLASMA_CURRENT_ON_A    = 1.0   # UNVALIDATED PLACEHOLDER -- current at/above this
                                     # latches plasma_detected True (after the debounce
                                     # below). Must be measured on the bench against the
                                     # real supply's no-load vs. struck-plasma current
                                     # before this means anything -- same "guess until
                                     # confirmed" caveat as every other cross-sensor
                                     # threshold in this file.
PZEM_PLASMA_CURRENT_OFF_A   = 0.6   # Below this, plasma_detected clears immediately --
                                     # no debounce needed to fail toward "not detected".
                                     # Must be < PZEM_PLASMA_CURRENT_ON_A (hysteresis, same
                                     # shape as TURBOOPTO_ON/OFF_THRESHOLD above).
PZEM_PLASMA_CONFIRM_SAMPLES = 3     # Consecutive above-threshold reads required before a
                                     # rising edge latches -- a single Modbus CRC glitch
                                     # must not flip plasma_detected (same reasoning as
                                     # TURBO_VALVE_CONFIRM_SAMPLES).

PZEM_PLASMA_ABSENT_GRACE_SECONDS    = 5.0   # SPUTTER_READY/SPUTTERING both assume plasma
                                             # is lit already (PLASMA_IGNITING's own
                                             # PLASMA_IGNITION_TIMEOUT handles "never
                                             # struck"); this long without PZEM confirming
                                             # current draw in either state means an arc
                                             # dropout, not normal settling.
PZEM_POWER_UNEXPECTED_GRACE_SECONDS = 5.0   # Plasma-level current seen in a state where the
                                             # HV/RF supply should not be energized at all
                                             # (stuck relay, live supply, wiring fault) for
                                             # this long.

# Sane-range bounds for the AC mains reading itself (distinct from
# SANE_VOLTAGE_MIN/MAX above, which are 0-3.3V DC ADC-side signals) --
# catches a disconnected/miswired PZEM, not a real reading at any variac
# tap position. India mains: nominal 230V/50Hz; the variac can legitimately
# sweep output down toward 0V, so there is no floor check here, only a
# ceiling/plausibility band.
PZEM_VOLTAGE_MAX   = 300.0   # V -- above this the reading itself is not credible
PZEM_FREQUENCY_MIN  = 45.0   # Hz
PZEM_FREQUENCY_MAX  = 65.0   # Hz

# PZEM graph (multi-series: voltage, current, power, frequency, power
# factor -- cumulative energy (Wh) is shown as a running-total readout
# instead of on the scrolling graph, since a monotonically-increasing
# session total doesn't share a meaningful scale with the other live
# quantities).
GRAPH_PZEM_VOLTAGE_MAX = 300.0
GRAPH_PZEM_CURRENT_MAX = 10.0     # A -- update once the real supply's max draw is known
GRAPH_PZEM_POWER_MAX   = 2500.0   # W -- update once the real supply's max draw is known
GRAPH_PZEM_FREQ_MIN     = 45.0
GRAPH_PZEM_FREQ_MAX     = 65.0

# ════════════════════════════════════════════════════════
#  PERSISTENT EVENT LOG
# ════════════════════════════════════════════════════════
EVENT_LOG_FILENAME = "sputter_ctrl.log"  # Appended next to main.py; every
                                          # operator-visible error/warning
                                          # and every state transition is
                                          # logged here with a timestamp, so
                                          # incidents can be reconstructed
                                          # after the fact (the GUI status
                                          # bar auto-expires messages after
                                          # ERROR_DISPLAY_SECONDS)

# ════════════════════════════════════════════════════════
#  POLLING & TIMING
# ════════════════════════════════════════════════════════
POLLING_INTERVAL = 0.2  # Seconds between sensor reads

# ════════════════════════════════════════════════════════
#  GUI SCROLLING GRAPH PARAMETERS
# ════════════════════════════════════════════════════════
GRAPH_MAX_SAMPLES = 60   # Number of data points in scrolling view

# Pirani graph
GRAPH_PIRANI_MIN = 0.0
GRAPH_PIRANI_MAX = 4.0
GRAPH_PIRANI_COLOR = "#00FF00"  # lime -- hex, not the X11 name: some Tk builds
                                # (e.g. macOS's old bundled Tk 8.5) don't ship
                                # the extended X11 color-name table "lime" needs;
                                # hex codes need no external lookup at all

# MFC graph
GRAPH_MFC_MIN = 0.0
GRAPH_MFC_MAX = 700.0
GRAPH_MFC_COLOR = "#00FFFF"  # cyan

# Turbo RPM graph
GRAPH_TURBO_RPM_MIN = 0.0
GRAPH_TURBO_RPM_MAX = TURBO_RPM_FULL_SCALE
GRAPH_TURBO_RPM_COLOR = "#FF00FF"  # magenta

# ════════════════════════════════════════════════════════
#  WEB DASHBOARD (browser view, parallel to the Tkinter GUI)
# ════════════════════════════════════════════════════════
WEB_UI_ENABLED = True    # Serve the browser dashboard (ui/web_ui.py). The local
                         # Tkinter GUI always runs regardless -- the web UI is
                         # a second view of the same process, never a
                         # replacement, so network loss only costs the browser
                         # page. Set False to not open the port at all.
WEB_UI_PORT = 8080       # http://<pi>:<port>/ -- LAN-trust, NO authentication
                         # (same trust model as the Pi's own VNC/SSH); see the
                         # README's Web Dashboard section
WEB_UI_UPDATE_INTERVAL = 0.2  # Seconds between SSE state pushes to each
                              # connected browser -- matches the local GUI's
                              # refresh cadence (GUI_REFRESH_INTERVAL)

# ════════════════════════════════════════════════════════
#  GUI REFRESH INTERVAL
# ════════════════════════════════════════════════════════
GUI_REFRESH_INTERVAL = 200  # milliseconds
ERROR_DISPLAY_SECONDS = 10  # How long an error message stays in the status bar before auto-clearing
