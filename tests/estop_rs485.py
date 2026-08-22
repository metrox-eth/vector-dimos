"""Software e-stop for the VECTOR RS485 bus: zero RPM, then DISABLE, both drives.

For the one case the runtime cannot cover: the dimOS daemon is gone (crash,
SIGKILL, lost ssh) while a non-zero RPM was commanded. A ZLAC8015D holds its
last setpoint until told otherwise or powered off. `dimos stop` already does
these two writes through the adapter's disconnect(); this is the same two
writes with no runtime in between. Safe to run on idle, disabled drives.

    $ python tests/estop_rs485.py                 # defaults to /dev/ttyTHS1
    $ python tests/estop_rs485.py /dev/ttyUSB0

Exit code 0 only when both drives acknowledged both writes. Anything else:
cut the motor power.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from pymodbus.client.sync import ModbusSerialClient

from vector_dimos.adapter import BACK_ID, BAUDRATE, FRONT_ID, SERIAL_TIMEOUT_S
from vector_dimos.zlac8015d import Controller

port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyTHS1"
client = ModbusSerialClient(method="rtu", port=port, baudrate=BAUDRATE,
                            timeout=SERIAL_TIMEOUT_S)
if not client.connect():
    print(f"port {port} did not open - cut the motor power")
    raise SystemExit(1)
rc = 0
for unit, side in ((FRONT_ID, "front"), (BACK_ID, "back")):
    c = Controller(unit, client=client)
    stopped = c.set_rpm(0, 0)
    disabled = c.disable()
    feedback = c.get_rpm()
    print(f"unit {unit} ({side}): rpm 0/0 acked={stopped} disable acked={disabled} "
          f"feedback rpm={feedback}")
    if not (stopped and disabled):
        rc = 1
client.close()
print("E-STOP " + ("DONE: both drives stopped and disabled" if rc == 0
                   else "INCOMPLETE - cut the motor power"))
raise SystemExit(rc)
