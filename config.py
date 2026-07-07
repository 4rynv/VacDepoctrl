# config.py — Centralized configuration for Sputter Controller

# ════════════════════════════════════════════════════════
#  GPIO PINS (BCM numbering)
# ════════════════════════════════════════════════════════
GPIO_PIRANI_PIN           = 17   # Input from Pirani gauge / control line for opto logic
GPIO_MFC_VALVE_CLOSE_PIN  = 27   # MFC valve close (emergency shut)
GPIO_TURBO_VALVE_PIN      = 22   # Turbo inlet valve relay (BC547 driver, valve on NC
                                 # contact): GPIO HIGH = valve OPEN, LOW = valve CLOSED.
                                 # Held closed at all times; opened only above
                                 # TURBO_VALVE_OPEN_MBAR during VENTING.
                                 # (Was GPIO 4 / physical pin 7 — pad affected in the
                                 # early hardware issue along with GPIO 2/3. GPIO 22
                                 # = physical pin 15.)
TURBO_VALVE_OPEN_MBAR     = 1.0  # During VENTING, open the turbo inlet valve once
                                 # chamber pressure rises above this (latched until
                                 # the state machine leaves VENTING)
TURBO_VALVE_CONFIRM_SAMPLES = 3  # Consecutive polling reads above TURBO_VALVE_OPEN_MBAR
                                 # required before the valve opens (3 × 0.2 s poll = 0.6 s).
                                 # A corrupted read on the software I2C bus (EMI bit-flips
                                 # return garbage without raising) must never open the valve.

# ════════════════════════════════════════════════════════
#  I2C BUS
# ════════════════════════════════════════════════════════
I2C_BUS_NUMBER = 3   # 1 = hardware I2C (GPIO2/3, physical pins 3/5) — configured
                     # by early hardware issue. 3 = software i2c-gpio on GPIO23/24
                     # (physical pins 16/18), enabled via dtoverlay in /boot/firmware/config.txt

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
VENTING_COMPLETE_VOLTAGE    = 2.5     # Pirani voltage for atmosphere; VENTING to IDLE
IDLE_PRESSURE_MAX_VOLTAGE   = 2.71    # Voltage threshold for safe pump-down start (10 mbar: 8.20V × 0.33)
PUMP_DOWN_COMPLETE_VOLTAGE  = 0.2   # Pirani voltage for ~0.01 mbar; threshold for PUMP_DOWN -> READY transition
ARGON_FLUSH_TARGET_VOLTAGE  = 1.287   # Pirani voltage target for argon flush (0.09 mbar: 3.90V × 0.33)
ARGON_FLUSH_FLOW_SETPOINT   = 150.0   # Initial MFC flow on flush entry; control loop takes over
PRESSURE_CONTROL_KP         = 8.0    # Proportional gain for MFC pressure control loop (sccm/V-error)
PRESSURE_CONTROL_KD         = 3.0     # Derivative gain for MFC pressure control loop (set >0 to dampen oscillation)
PRESSURE_CONTROL_KI         = 0.005     # Integral gain (sccm/(V.s))
PRESSURE_CONTROL_ICLAMP     = 50.0    # Anti-windup clamp (sccm)
PLASMA_IGNITION_TIMEOUT     = 120.0   # Seconds to wait for manual plasma confirmation before returning to READY
SPUTTER_READY_TARGET_VOLTAGE = 0.6  # Pirani voltage target for sputtering (0.007 mbar: 1.10V × 0.33)
ARGON_DAC_I2C_ADDRESS       = 0x60    # I2C address for the argon MFC DAC (MCP4725 default)
ARGON_DAC_VREF              = 5.0     # MCP4725's real max output — it's powered off the Pi's 3.3V rail
                                      # (no level shifter fitted). This is NOT the MFC's 5V setpoint
                                      # scale (see MFC_SETPOINT_VOLTAGE_FULL_SCALE) — it's the hard
                                      # ceiling on what we can actually command out of the DAC today.
ARGON_DAC_RESOLUTION        = 4096    # DAC resolution for MCP4725 / 12-bit output
ADS1115_I2C_ADDRESS         = 0x48    # ADS1115 I2C address (default for most boards)

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
GRAPH_PIRANI_COLOR = "lime"

# MFC graph
GRAPH_MFC_MIN = 0.0
GRAPH_MFC_MAX = 700.0
GRAPH_MFC_COLOR = "cyan"

# ════════════════════════════════════════════════════════
#  GUI REFRESH INTERVAL
# ════════════════════════════════════════════════════════
GUI_REFRESH_INTERVAL = 200  # milliseconds
ERROR_DISPLAY_SECONDS = 10  # How long an error message stays in the status bar before auto-clearing
