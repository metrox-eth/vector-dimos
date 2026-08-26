"""Write the ZLAC8015D communication-offline time (0x2000) on both drives.

    $ python tests/set_zlac_comm_timeout.py 1000        # value as the drive counts it; 0 = off
    $ python tests/set_zlac_comm_timeout.py 1000 /dev/ttyUSB0

What the drive does when that long passes without a MODBUS frame is the
drive's business (stop? alarm? both?) - measure it on blocks before relying
on it. Reads the register back and prints it.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from pymodbus.client.sync import ModbusSerialClient

from vector_dimos.adapter import DEFAULT_PORT, BACK_ID, BAUDRATE, FRONT_ID, SERIAL_TIMEOUT_S
from vector_dimos.zlac8015d import Controller

COMM_OFFLINE_TIME = 0x2000

value = int(sys.argv[1])
port = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_PORT
client = ModbusSerialClient(method="rtu", port=port, baudrate=BAUDRATE,
                            timeout=SERIAL_TIMEOUT_S)
if not client.connect():
    print(f"port {port} did not open")
    raise SystemExit(1)
rc = 0
for unit, side in ((FRONT_ID, "front"), (BACK_ID, "back")):
    c = Controller(unit, client=client)
    before = c._read(COMM_OFFLINE_TIME, 1)
    ok = c._write_register(COMM_OFFLINE_TIME, value)
    after = c._read(COMM_OFFLINE_TIME, 1)
    print(f"unit {unit} ({side}): 0x2000 {before} -> write {value} acked={ok} -> reads back {after}")
    if not ok or after != [value]:
        rc = 1
client.close()
raise SystemExit(rc)
