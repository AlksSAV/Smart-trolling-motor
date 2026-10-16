#!/usr/bin/env python3
"""\u0410\u0432\u0442\u043e\u043c\u0430\u0442\u0438\u0447\u0435\u0441\u043a\u0438\u0439 \u0442\u0435\u0441\u0442 \u0440\u0435\u0436\u0438\u043c\u043e\u0432: M \u2192 H \u2192 A \u2192 S \u2192 M, \u043f\u043e 10 \u0441\u0435\u043a \u043a\u0430\u0436\u0434\u044b\u0439.

  python logs/test_all_modes.py COM23

\u041f\u043e\u0440\u0442 \u043e\u0442\u043a\u0440\u044b\u0432\u0430\u0435\u0442\u0441\u044f \u0431\u0435\u0437 \u0441\u0431\u0440\u043e\u0441\u0430 (DTR/RTS \u043d\u0438\u0437\u043a\u0438\u0435).
"""

import sys, time
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM23"
MODES = ["M", "H", "S", "A"]  # \u043f\u043e\u0440\u044f\u0434\u043e\u043a: Manual \u2192 Heading \u2192 Simulated \u2192 Anchor
DURATION = 10  # \u0441\u0435\u043a\u0443\u043d\u0434 \u043d\u0430 \u0440\u0435\u0436\u0438\u043c

ser = serial.Serial()
ser.port = PORT
ser.baudrate = 115200
ser.timeout = 0.1
ser.dtr = False
ser.rts = False
ser.open()

time.sleep(0.3)
ser.reset_input_buffer()

print("=" * 60)
print(f"  \u0410\u0432\u0442\u043e-\u0442\u0435\u0441\u0442 \u0440\u0435\u0436\u0438\u043c\u043e\u0432 \u043d\u0430 {PORT}")
print(f"  \u041f\u043e\u0440\u044f\u0434\u043e\u043a: {' \u2192 '.join(MODES)} (\u043f\u043e {DURATION} \u0441 \u043a\u0430\u0436\u0434\u044b\u0439)")
print("=" * 60)

for mode in MODES:
    print(f"\n\n{'='*60}")
    print(f"  >>> \u0420\u0435\u0436\u0438\u043c: {mode}, {DURATION} \u0441  <<<")
    print(f"{'='*60}")
    sys.stdout.flush()

    ser.write((mode + "\n").encode())
    ser.flush()

    end = time.time() + DURATION
    while time.time() < end:
        data = ser.read(1024)
        if data:
            sys.stdout.write(data.decode("utf-8", errors="replace"))
            sys.stdout.flush()

time.sleep(1)
ser.close()
print("\n  \u0422\u0435\u0441\u0442 \u0437\u0430\u0432\u0435\u0440\u0448\u0451\u043d.")