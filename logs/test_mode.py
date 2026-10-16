#!/usr/bin/env python3
"""Send one mode command to the board and stream its output.

  python logs/test_mode.py COM23 H 30

The port is opened with DTR/RTS held low so the board is NOT reset -
otherwise the mode set by the command would be lost.
"""

import sys, time
import serial

port = sys.argv[1]
cmd = sys.argv[2].upper()
seconds = float(sys.argv[3]) if len(sys.argv) > 3 else 15.0

VALID = ("M", "H", "A", "S", "L")
if cmd not in VALID:
    sys.exit(f"unknown command '{cmd}', expected one of {VALID}")

ser = serial.Serial()
ser.port = port
ser.baudrate = 115200
ser.timeout = 0.05
ser.dtr = False
ser.rts = False
ser.open()

time.sleep(0.3)
ser.reset_input_buffer()

ser.write((cmd + "\n").encode())
ser.flush()
print(f">>> sent '{cmd}' to {port}, reading {seconds:.0f} s", flush=True)

end = time.time() + seconds
while time.time() < end:
    data = ser.read(512)
    if data:
        sys.stdout.write(data.decode("utf-8", errors="replace"))
        sys.stdout.flush()

ser.close()
