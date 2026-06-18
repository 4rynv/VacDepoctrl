# config.py — Centralized configuration for Sputter Controller

# ════════════════════════════════════════════════════════
#  GPIO PINS (BCM numbering)
# ════════════════════════════════════════════════════════
GPIO_PIRANI_PIN           = 17   # Input from Pirani gauge / control line for opto logic
GPIO_MFC_VALVE_CLOSE_PIN  = 27   # MFC valve close (emergency shut)

# ════════════════════════════════════════════════════════
#  ADC CONFIGURATION
# ════════════════════════════════════════════════════════
ADC_VREF  = 3.33      # Reference voltage (volts)
ADC_GAIN  = 1         # ADS1115 gain setting

# ADC Channel Assignment
ADC_CHANNEL_PIRANI = 0   # Pirani gauge on A0
ADC_CHANNEL_MFC    = 1   # MFC flow on A1

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
ATMOSPHERE_THRESHOLD        = 30000   # ADC ≥ this → VENTING to IDLE
IDLE_PRESSURE_MAX_VOLTAGE   = 2.5     # Voltage threshold for safe pump-down start (≈10 mbar)
PUMP_DOWN_COMPLETE_VOLTAGE  = 0.1     # Near-zero Pirani voltage before releasing MFC valve
ARGON_FLUSH_TARGET_VOLTAGE  = 1.19    # Pirani voltage target for argon flush (≈0.1 mbar)
ARGON_FLUSH_FLOW_SETPOINT   = 150.0   # Placeholder MFC flow target during argon flush (sccm)
SPUTTER_READY_TARGET_VOLTAGE = 0.466   # Pirani voltage target for sputter-ready range (≤0.466 V)
ARGON_DAC_I2C_ADDRESS       = 0x60    # I2C address for the argon MFC DAC (MCP4725 default)
ARGON_DAC_VREF              = 3.3    # MCP4725's real max output — it's powered off the Pi's 3.3V rail
                                      # (no level shifter fitted). This is NOT the MFC's 5V setpoint
                                      # scale (see MFC_SETPOINT_VOLTAGE_FULL_SCALE) — it's the hard
                                      # ceiling on what we can actually command out of the DAC today.
ARGON_DAC_RESOLUTION        = 4096    # DAC resolution for MCP4725 / 12-bit output
ADS1115_I2C_ADDRESS         = 0x48    # ADS1115 I2C address (default for most boards)

# ════════════════════════════════════════════════════════
#  POLLING & TIMING
# ════════════════════════════════════════════════════════
POLLING_INTERVAL = 0.5  # Seconds between sensor reads

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
GUI_REFRESH_INTERVAL = 500  # milliseconds
