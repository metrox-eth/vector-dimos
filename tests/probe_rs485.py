"""Read-only MODBUS probe of the VECTOR RS485 bus - writes nothing.

Answers one question before anything else is attempted: is a ZLAC8015D
actually talking on this port? It opens the serial port and reads the
feedback-RPM register pair (0x20AB) on both driver ids. No register is
written and no drive is enabled, so it is safe to run with the motors
powered.

    $ python tests/probe_rs485.py                 # defaults to the adapter's port (FTDI dongle)
    $ python tests/probe_rs485.py /dev/ttyUSB0

A driver that answers prints `unit N: regs=[...]` in a few milliseconds. A
driver that is not there times out after the pymodbus timeout below (~500 ms)
and prints the ModbusIOException. Note that `open: True` proves nothing about
the wiring - a UART always opens.
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from pymodbus.client.sync import ModbusSerialClient

from vector_dimos.adapter import DEFAULT_PORT, BACK_ID, BAUDRATE, FRONT_ID, SERIAL_TIMEOUT_S
from vector_dimos.zlac8015d import L_FB_RPM

port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
client = ModbusSerialClient(method="rtu", port=port, baudrate=BAUDRATE,
                            timeout=SERIAL_TIMEOUT_S)
opened = client.connect()
print(f"port {port} open: {opened}")
if not opened:
    print("the port itself did not open: wrong path, busy, or no permission "
          "(the user must be in the dialout group)")
    raise SystemExit(1)
for unit, side in ((BACK_ID, "back"), (FRONT_ID, "front")):
    t0 = time.monotonic()
    rr = client.read_holding_registers(L_FB_RPM, 2, unit=unit)
    dt_ms = (time.monotonic() - t0) * 1000.0
    if rr.isError():
        print(f"unit {unit} ({side}): no answer ({dt_ms:.0f} ms) -> {rr}")
    else:
        print(f"unit {unit} ({side}): regs={rr.registers} ({dt_ms:.0f} ms)")
client.close()
