"""MockZLAC bus: a pymodbus-shaped fake for cold benches (no hardware)."""
from __future__ import annotations


class _ReadResult:
    def __init__(self, registers):
        self.registers = registers


class MockModbusClient:
    """Register-level mock shared by both controllers on the 'bus'.

    Written RPM commands are echoed back on the feedback registers
    (x10, feedback unit is 0.1 RPM) - a perfect no-slip bus.
    """

    def __init__(self):
        self.regs = {}          # (unit, addr) -> value
        self.writes = []        # log of (unit, addr, values)

    def write_register(self, addr, value, unit=0):
        self.regs[(unit, addr)] = value
        self.writes.append((unit, addr, [value]))

    def write_registers(self, addr, values, unit=0):
        for i, v in enumerate(values):
            self.regs[(unit, addr + i)] = v
        self.writes.append((unit, addr, list(values)))
        # echo RPM commands to feedback registers (0.1 RPM units)
        from vector_dimos.zlac8015d import L_CMD_RPM, L_FB_RPM, _to_i16
        if addr == L_CMD_RPM:
            for i, v in enumerate(values):
                self.regs[(unit, L_FB_RPM + i)] = (_to_i16(v) * 10) & 0xFFFF

    def read_holding_registers(self, addr, count, unit=0):
        return _ReadResult([self.regs.get((unit, addr + i), 0)
                            for i in range(count)])

    def connect(self):
        return True

    def close(self):
        pass
