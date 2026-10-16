import sys, time
import serial

port, seconds = sys.argv[1], float(sys.argv[2])
with serial.Serial(port, 115200, timeout=1) as s:
    # Toggle DTR/RTS to hard-reset the board and capture boot output.
    s.dtr = False; s.rts = True;  time.sleep(0.1)
    s.rts = False; time.sleep(0.1)
    end = time.time() + seconds
    while time.time() < end:
        data = s.read(512)
        if data:
            sys.stdout.write(data.decode("utf-8", errors="replace"))
            sys.stdout.flush()
