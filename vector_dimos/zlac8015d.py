"""Minimal ZLAC8015D driver - MODBUS RTU over RS485, velocity mode only.

Clean rewrite for VECTOR (register map inherited from the first robot code
by Sam, proven on this chassis). One Controller instance per ZLAC8015D
(dual-channel: L and R wheel). pymodbus<3.0 sync API.
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

    def set_mode_velocity(self) -> None:
        self.client.write_register(OPR_MODE, VEL_CONTROL, unit=self.unit)

    def set_accel_ms(self, accel_ms: int, decel_ms: int) -> None:
        self.client.write_registers(L_ACL_TIME, [accel_ms, accel_ms], unit=self.unit)
        self.client.write_registers(L_DCL_TIME, [decel_ms, decel_ms], unit=self.unit)

    def enable(self) -> None:
        self.client.write_register(CONTROL_REG, ENABLE, unit=self.unit)

    def disable(self) -> None:
        self.client.write_register(CONTROL_REG, DISABLE, unit=self.unit)

    def set_rpm(self, l_rpm: float, r_rpm: float) -> None:
        self.client.write_registers(
            L_CMD_RPM, [_to_u16(round(l_rpm)), _to_u16(round(r_rpm))],
            unit=self.unit)

    def get_rpm(self) -> tuple[float, float]:
        rr = self.client.read_holding_registers(L_FB_RPM, 2, unit=self.unit)
        regs = rr.registers
        return _to_i16(regs[0]) / 10.0, _to_i16(regs[1]) / 10.0

    def get_faults(self) -> tuple[int, int]:
        rr = self.client.read_holding_registers(L_FAULT, 2, unit=self.unit)
        return rr.registers[0], rr.registers[1]
