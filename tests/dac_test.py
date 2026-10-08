import time
from adafruit_extended_bus import ExtendedI2C

# --------------------------------------------------
#  Test MCP4725 / I2C DAC output on 0-3.3V range
# --------------------------------------------------
DAC_I2C_ADDRESS = 0x60
DAC_RESOLUTION = 4096
DAC_MAX_VOLTAGE = 3.3
WRITE_STEPS = [0, 256, 512, 1024, 2048, 3072, 3584, DAC_RESOLUTION - 1]
WRITE_MODES = ["fast", "eeprom"]


def write_dac_fast(code, i2c):
    """Write a 12-bit value to the MCP4725 via fast-mode write (volatile only)."""
    data = bytes([(code >> 8) & 0x0F, code & 0xFF])
    i2c.writeto(DAC_I2C_ADDRESS, data)


def write_dac_eeprom(code, i2c):
    """Write a 12-bit value to the MCP4725 DAC register and EEPROM."""
    data = bytes([0x60, (code >> 4) & 0xFF, (code & 0x0F) << 4])
    i2c.writeto(DAC_I2C_ADDRESS, data)


def decode_readback(buf):
    """Decode MCP4725 readback bytes into status and DAC register value."""
    settings = buf[0]
    ready = bool(settings & 0x80)
    pd = (settings >> 1) & 0x03
    dac_value = (buf[1] << 4) | (buf[2] >> 4)
    return ready, pd, dac_value


def read_dac(i2c):
    """Read three status/DAC bytes back from the DAC for diagnostics."""
    buf = bytearray(3)
    i2c.readfrom_into(DAC_I2C_ADDRESS, buf)
    return buf


def wait_dac_ready(i2c, timeout=2.0):
    """Read until the DAC reports the EEPROM write is complete, or timeout."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        buf = bytearray(3)
        try:
            i2c.readfrom_into(DAC_I2C_ADDRESS, buf)
        except Exception as exc:
            print("Wait read failed:", exc)
            time.sleep(0.05)
            continue
        ready, pd, dac_value = decode_readback(buf)
        print("  wait readback:", [f"0x{b:02X}" for b in buf], f"ready={ready}", f"pd={pd}", f"dac={dac_value}")
        if ready:
            return True, buf
        time.sleep(0.05)
    return False, buf


# bus 1 = hardware I2C: SDA=GPIO2 (pin 3), SCL=GPIO3 (pin 5)
with ExtendedI2C(1) as i2c:
    print("Scanning I2C bus...")
    while not i2c.try_lock():
        pass
    try:
        addresses = i2c.scan()
    finally:
        i2c.unlock()
    print("Found I2C addresses:", [hex(a) for a in addresses])
    if DAC_I2C_ADDRESS not in addresses:
        raise RuntimeError(f"DAC at address 0x{DAC_I2C_ADDRESS:02X} not found")

    print("Starting continuous DAC sweep through multiple voltages...")
    steps = WRITE_STEPS
    while True:
        for mode in WRITE_MODES:
            for code in steps:
                voltage = (code / (DAC_RESOLUTION - 1)) * DAC_MAX_VOLTAGE
                print(f"Mode={mode:8s} code={code:4d} -> target {voltage:.3f} V")

                while not i2c.try_lock():
                    time.sleep(0.01)
                try:
                    try:
                        if mode == "fast":
                            write_dac_fast(code, i2c)
                        else:
                            write_dac_eeprom(code, i2c)
                    except OSError as exc:
                        print("Write failed:", exc)
                        continue

                    time.sleep(0.1)
                    try:
                        ready, buf = wait_dac_ready(i2c)
                        if ready:
                            _, pd, dac_value = decode_readback(buf)
                            actual_voltage = (dac_value / (DAC_RESOLUTION - 1)) * DAC_MAX_VOLTAGE
                            print(
                                "Readback bytes:", [f"0x{b:02X}" for b in buf],
                                f"pd={pd}", f"dac_value={dac_value}",
                                f"actual_voltage={actual_voltage:.3f} V"
                            )
                        else:
                            print("DAC EEPROM write did not complete in time.")
                    except Exception as exc:
                        print("Readback failed:", exc)
                finally:
                    i2c.unlock()
                time.sleep(1.5)