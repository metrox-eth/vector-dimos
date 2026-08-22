"""Read-only dump of the ZLAC8015D parameters that decide what the wheels do
when nobody is talking to them. Writes nothing; safe with motors powered.

    $ python tests/read_zlac_params.py                # defaults to /dev/ttyTHS1

Registers (ZLAC8015D manual): 0x2000 communication-offline time [ms] (the
drive-side watchdog: what the drive does after that long without a MODBUS
frame depends on its own settings - observe it, do not assume), 0x2008 max
RPM, 0x200D operating mode (3 = velocity), 0x2080/0x2082 accel/decel ramps
[ms], 0x20A5/0x20A6 fault words, 0x20AB feedback RPM x10.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from pymodbus.client.sync import ModbusSerialClient

from vector_dimos.adapter import BACK_ID, BAUDRATE, FRONT_ID, SERIAL_TIMEOUT_S

REGS = (
    (0x2000, 1, "comm offline time [ms]"),
    (0x2008, 1, "max rpm"),
    (0x200D, 1, "operating mode"),
    (0x200E, 1, "control word"),
    (0x2080, 2, "accel L/R [ms]"),
    (0x2082, 2, "decel L/R [ms]"),
    (0x20A5, 2, "faults L/R"),
    (0x20AB, 2, "feedback rpm x10 L/R"),
)

port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyTHS1"
client = ModbusSerialClient(method="rtu", port=port, baudrate=BAUDRATE,
                            timeout=SERIAL_TIMEOUT_S)
if not client.connect():
    print(f"port {port} did not open")
    raise SystemExit(1)
for unit, side in ((FRONT_ID, "front"), (BACK_ID, "back")):
    print(f"unit {unit} ({side}):")
    for addr, count, name in REGS:
        rr = client.read_holding_registers(addr, count, unit=unit)
        val = f"no answer ({rr})" if rr.isError() else rr.registers
        print(f"  0x{addr:04X} {name:24s} {val}")
client.close()
