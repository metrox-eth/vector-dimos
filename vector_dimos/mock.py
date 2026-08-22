"""MockZLAC bus: a pymodbus-shaped fake for cold benches (no hardware).

Also the bus used when VECTOR_MOCK_BUS is set, so the real dimOS runtime can
be driven end to end on a machine with no motors wired.
"""
from __future__ import annotations


class _Result:
    """pymodbus-shaped response: .registers + .isError()."""

    def __init__(self, registers=None):
        self.registers = registers if registers is not None else []

    def isError(self) -> bool:
        return False


class MockModbusClient:
    """Register-level mock shared by both controllers on the 'bus'.

    Written RPM commands are echoed back on the feedback registers
    (x10, feedback unit is 0.1 RPM) - a perfect no-slip bus.

    `writes` and `reads` are transaction logs: tests count bus round-trips
    with them (e.g. to prove the feedback cache collapses read_velocities +
    read_odometry into one read per controller).
    """

    def __init__(self):
        self.regs = {}          # (unit, addr) -> value
        self.writes = []        # log of (unit, addr, values)
        self.reads = []         # log of (unit, addr, count)

    def write_register(self, addr, value, unit=0):
        self.regs[(unit, addr)] = value
        self.writes.append((unit, addr, [value]))
        return _Result()

    def write_registers(self, addr, values, unit=0):
        for i, v in enumerate(values):
            self.regs[(unit, addr + i)] = v
        self.writes.append((unit, addr, list(values)))
        # echo RPM commands to feedback registers (0.1 RPM units)
        from vector_dimos.zlac8015d import L_CMD_RPM, L_FB_RPM, _to_i16
        if addr == L_CMD_RPM:
            for i, v in enumerate(values):
                self.regs[(unit, L_FB_RPM + i)] = (_to_i16(v) * 10) & 0xFFFF
        return _Result()

    def read_holding_registers(self, addr, count, unit=0):
        self.reads.append((unit, addr, count))
        return _Result([self.regs.get((unit, addr + i), 0)
                        for i in range(count)])

    def connect(self):
        return True

    def close(self):
        pass
