# pzem_meter.py — PZEM-004T-100A energy meter over Modbus-RTU (CP2102 USB-TTL)
#
# Reads AC voltage/current/power/energy/frequency/power-factor from the
# sputtering system's variac output. Feeds live GUI stats and, via
# plasma_detect_step() in main.py, plasma-ignition detection from the
# current-draw step when the discharge strikes.
#
# Hand-rolled Modbus RTU (request framing + CRC16) rather than a general
# Modbus library, matching how mfc_control.py hand-rolls the MCP4725 I2C
# write -- one fixed exchange doesn't need a general-purpose client.
#
# This sensor is OPTIONAL at startup, unlike the ADS1115/MCP4725 checked in
# main.py's _startup_self_test(): it may not be wired up yet. If the serial
# port can't be opened, or the meter doesn't respond, `ready` stays False
# and read() returns a zeroed reading instead of raising -- callers must not
# assume this hardware is present.

import serial

from config import (
    PZEM_SERIAL_PORT,
    PZEM_BAUDRATE,
    PZEM_SLAVE_ADDRESS,
    PZEM_SERIAL_TIMEOUT,
)

_READ_INPUT_REGISTERS = 0x04
_REGISTER_COUNT = 10  # 0x0000-0x0009: V, I(lo,hi), P(lo,hi), E(lo,hi), Hz, PF, alarm


def _crc16_modbus(data):
    """Standard Modbus RTU CRC16 (poly 0xA001, init 0xFFFF)."""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def _build_read_frame(slave_addr, start_reg, count):
    frame = bytes([
        slave_addr, _READ_INPUT_REGISTERS,
        (start_reg >> 8) & 0xFF, start_reg & 0xFF,
        (count >> 8) & 0xFF, count & 0xFF,
    ])
    crc = _crc16_modbus(frame)
    return frame + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def _parse_response(resp, slave_addr, count):
    """Validate and decode a read-input-registers response. Raises ValueError
    on any framing/length/CRC mismatch -- the caller treats this identically
    to a serial timeout (meter not ready)."""
    expected_len = 3 + count * 2 + 2
    if len(resp) != expected_len:
        raise ValueError(f"short PZEM response: {len(resp)} of {expected_len} bytes")
    if resp[0] != slave_addr or resp[1] != _READ_INPUT_REGISTERS:
        raise ValueError("unexpected PZEM response header")
    byte_count = resp[2]
    if byte_count != count * 2:
        raise ValueError("unexpected PZEM byte count")
    payload, crc_recv = resp[:-2], resp[-2] | (resp[-1] << 8)
    if _crc16_modbus(payload) != crc_recv:
        raise ValueError("PZEM response CRC mismatch")
    regs = []
    for i in range(count):
        hi = resp[3 + i * 2]
        lo = resp[4 + i * 2]
        regs.append((hi << 8) | lo)
    return regs


def _decode_registers(regs):
    return {
        "voltage":      regs[0] / 10.0,
        "current":      ((regs[2] << 16) | regs[1]) / 1000.0,
        "power":        ((regs[4] << 16) | regs[3]) / 10.0,
        "energy":       float((regs[6] << 16) | regs[5]),
        "frequency":    regs[7] / 10.0,
        "power_factor": regs[8] / 100.0,
        "alarm":        regs[9] != 0,
    }


_EMPTY_READING = {
    "voltage": 0.0, "current": 0.0, "power": 0.0, "energy": 0.0,
    "frequency": 0.0, "power_factor": 0.0, "alarm": False,
}


class PZEMController:
    """Optional hardware -- read() never raises. `ready` reflects whether the
    last transaction succeeded (mirrors MFCController.dac_ready, for the same
    "may not be installed yet" reason). Automatically retries opening the
    serial port on the next read() after a failure, so plugging the CP2102
    in later recovers without a restart.
    """

    def __init__(self, port=None, baudrate=None, slave_addr=None):
        self.port = port or PZEM_SERIAL_PORT
        self.baudrate = baudrate or PZEM_BAUDRATE
        self.slave_addr = slave_addr if slave_addr is not None else PZEM_SLAVE_ADDRESS
        self.ready = False
        self._ser = None
        self._open()

    def _open(self):
        try:
            self._ser = serial.Serial(
                self.port, self.baudrate,
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE, timeout=PZEM_SERIAL_TIMEOUT,
            )
        except Exception as e:
            print(f"[WARN] PZEM serial port {self.port} not available: {e}")
            self._ser = None

    def read(self):
        """Returns the decoded reading plus 'ready' and 'error'. Never
        raises -- any failure (port missing, meter not wired, timeout, CRC
        error) degrades to ready=False and a zeroed reading."""
        if self._ser is None:
            self._open()
        if self._ser is None:
            self.ready = False
            return dict(_EMPTY_READING, ready=False, error="serial port not open")

        try:
            self._ser.reset_input_buffer()
            self._ser.write(_build_read_frame(self.slave_addr, 0x0000, _REGISTER_COUNT))
            resp = self._ser.read(3 + _REGISTER_COUNT * 2 + 2)
            regs = _parse_response(resp, self.slave_addr, _REGISTER_COUNT)
            reading = _decode_registers(regs)
            self.ready = True
            return dict(reading, ready=True, error=None)
        except Exception as e:
            self.ready = False
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None
            return dict(_EMPTY_READING, ready=False, error=str(e))

    def close(self):
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None
