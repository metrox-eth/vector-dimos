"""Minimal ZLAC8015D driver - MODBUS RTU over RS485, velocity mode only.

Clean rewrite for VECTOR (register map inherited from the first robot code
by Sam, proven on this chassis). One Controller instance per ZLAC8015D
(dual-channel: L and R wheel). pymodbus<3.0 sync API.

Failure convention (the adapter depends on it):
  * reads  -> None  when the drive did not answer or answered an error;
  * writes -> False when the drive did not answer or answered an error.
Nothing in this module raises on bus trouble. pymodbus 2.5 is inconsistent
about it - a timeout usually comes back as a ModbusIOException *object*
(which is both an Exception and a response exposing isError()), but a
closed port raises - so both paths are handled.
"""
from __future__ import annotations

# Register map (ZLAC8015D manual, proven in the field)
CONTROL_REG = 0x200E
OPR_MODE = 0x200D
L_ACL_TIME, R_ACL_TIME = 0x2080, 0x2081
L_DCL_TIME, R_DCL_TIME = 0x2082, 0x2083
L_CMD_RPM = 0x2088          # int16 pair written together (L, R)
L_FB_RPM = 0x20AB           # int16 pair, 0.1 RPM units
L_FAULT, R_FAULT = 0x20A5, 0x20A6

VEL_CONTROL = 3
ENABLE, DISABLE, STOP = 0x08, 0x07, 0x05
MAX_RPM = 3000


def _to_u16(v: int) -> int:
    v = max(-MAX_RPM, min(MAX_RPM, int(v)))
    return v & 0xFFFF


def _to_i16(raw: int) -> int:
    raw &= 0xFFFF
    return raw - 0x10000 if raw >= 0x8000 else raw


def _resp_ok(rr) -> bool:
    """False when the client did not hand back a usable response.

    None is a failure, not a pass. Every client this driver runs against
    answers with a response object - pymodbus and vector_dimos.mock alike -
    so a None means the call went nowhere and must not be reported as a
    successful bus transaction (fail-open would hide a dead bus).
    """
    if rr is None:
        return False
    is_error = getattr(rr, "isError", None)
    if callable(is_error):
        return not is_error()
    return True


class Controller:
    """One ZLAC8015D (two wheels) on a shared MODBUS RTU bus."""

    def __init__(self, unit_id: int, client=None, port: str = "/dev/ttyTHS1",
                 baudrate: int = 115200):
        self.unit = unit_id
        if client is not None:
            self.client = client
        else:
            from pymodbus.client.sync import ModbusSerialClient
            self.client = ModbusSerialClient(method="rtu", port=port,
                                             baudrate=baudrate, timeout=0.5)
            self.client.connect()

    # ── raw bus access (never raises) ──────────────────────────────────
    def _read(self, addr: int, count: int) -> list[int] | None:
        try:
            rr = self.client.read_holding_registers(addr, count, unit=self.unit)
        except Exception:
            return None
        if not _resp_ok(rr):
            return None
        regs = getattr(rr, "registers", None)
        if regs is None or len(regs) < count:
            return None
        return regs

    def _write_register(self, addr: int, value: int) -> bool:
        try:
            return _resp_ok(self.client.write_register(addr, value,
                                                       unit=self.unit))
        except Exception:
            return False

    def _write_registers(self, addr: int, values: list[int]) -> bool:
        try:
            return _resp_ok(self.client.write_registers(addr, values,
                                                        unit=self.unit))
        except Exception:
            return False

    # ── commands ───────────────────────────────────────────────────────
    def set_mode_velocity(self) -> bool:
        return self._write_register(OPR_MODE, VEL_CONTROL)

    def set_accel_ms(self, accel_ms: int, decel_ms: int) -> bool:
        acl = self._write_registers(L_ACL_TIME, [accel_ms, accel_ms])
        dcl = self._write_registers(L_DCL_TIME, [decel_ms, decel_ms])
        return acl and dcl

    def enable(self) -> bool:
        return self._write_register(CONTROL_REG, ENABLE)

    def disable(self) -> bool:
        return self._write_register(CONTROL_REG, DISABLE)

    def set_rpm(self, l_rpm: float, r_rpm: float) -> bool:
        return self._write_registers(
            L_CMD_RPM, [_to_u16(round(l_rpm)), _to_u16(round(r_rpm))])

    # ── feedback (None = no valid answer) ──────────────────────────────
    def get_rpm(self) -> tuple[float, float] | None:
        regs = self._read(L_FB_RPM, 2)
        if regs is None:
            return None
        return _to_i16(regs[0]) / 10.0, _to_i16(regs[1]) / 10.0

    def get_faults(self) -> tuple[int, int] | None:
        regs = self._read(L_FAULT, 2)
        if regs is None:
            return None
        return regs[0], regs[1]
