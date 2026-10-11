#!/usr/bin/env python3
"""Capture and count what the GNSS module actually sends through the ESP32-S3
USB-serial bridge (env gnss_bridge).

Why this exists: docs/TODO.md item 18 claims "~60 sentences/sec" on a module
whose CFG-RATE measRate reads 200 ms. Those two numbers cannot both be true
for one sentence type, so the rate has to be measured, not assumed. This tool
answers three things with one 10 second run:
  1. which sentence/message types are on the wire,
  2. how many of each per second (and how many bytes each costs),
  3. whether the bytes are really NMEA (checksum verified) or binary UBX.

The port must be FREE: close it in u-center first (Receiver -> Disconnect),
two programs cannot open the same COM port on Windows.

Usage:
    python tools/log_gnss_stream.py                # COM23, 10 s
    python tools/log_gnss_stream.py -p COM23 -d 20 -o logs/gnss.raw
"""

import argparse
import os
import re
import sys
import time
from collections import Counter

try:
    import serial
except ImportError:
    sys.exit("pyserial is required: python -m pip install pyserial")

NMEA_ID_RE = re.compile(rb"[A-Z]{5}")
NMEA_CKSUM_RE = re.compile(rb"\*([0-9A-Fa-f]{2})\s*$")


def nmea_checksum(body: bytes) -> int:
    """XOR of every byte between '$' and '*' (exclusive of both)."""
    ck = 0
    for b in body:
        ck ^= b
    return ck


class StreamSplitter:
    """Splits a byte stream into NMEA sentences, UBX messages and junk.

    feed() accepts any chunk size, including one byte at a time, and only
    consumes frames that are complete. A trailing partial frame stays buffered
    for the next feed(), so frame boundaries survive arbitrary chunking.
    """

    MAX_LINE = 1024  # longest sane NMEA sentence (GSV can reach ~200)

    def __init__(self):
        self.buf = bytearray()
        self.total = 0

    def feed(self, data: bytes):
        self.total += len(data)
        self.buf.extend(data)
        events = []

        while self.buf:
            start = self.buf.find(b"$")
            ubx = self.buf.find(b"\xb5\x62")

            # No start marker at all: everything but the last byte is junk
            # (the last byte may be the first half of 0xB5 0x62).
            if start == -1 and ubx == -1:
                drop = max(0, len(self.buf) - 1)
                if drop:
                    events.append(("JUNK", None, drop))
                    del self.buf[:drop]
                break

            if start == -1:
                start = len(self.buf)
            if ubx == -1:
                ubx = len(self.buf)

            # Earlier of the two markers wins.
            if ubx < start:
                if ubx > 0:
                    events.append(("JUNK", None, ubx))
                    del self.buf[:ubx]
                    continue
                if len(self.buf) < 6:
                    break  # length field has not arrived yet
                cls, mid = self.buf[2], self.buf[3]
                length = self.buf[4] | (self.buf[5] << 8)
                size = 8 + length
                if len(self.buf) < size:
                    break  # payload still arriving
                events.append(("UBX", f"{cls:02X}{mid:02X}", size, True))
                del self.buf[:size]
                continue

            # NMEA candidate
            if start > 0:
                events.append(("JUNK", None, start))
                del self.buf[:start]
                continue

            nl = self.buf.find(b"\n", 1)
            if nl == -1:
                if len(self.buf) > self.MAX_LINE:
                    # No terminator within a sane length -> not a sentence.
                    events.append(("JUNK", None, 1))
                    del self.buf[:1]
                    continue
                break  # terminator may still be in flight

            raw = bytes(self.buf[: nl + 1])
            match = NMEA_ID_RE.match(raw, 1)
            if not match:
                events.append(("JUNK", None, 1))
                del self.buf[:1]
                continue

            ident = match.group().decode("ascii", "replace")
            ck_ok = False
            star = raw.find(b"*")
            if star != -1:
                found = NMEA_CKSUM_RE.search(raw)
                if found:
                    ck_ok = int(found.group(1), 16) == nmea_checksum(raw[1:star])
            events.append(("NMEA", ident, len(raw), ck_ok))
            del self.buf[: nl + 1]

        return events


def main() -> int:
    ap = argparse.ArgumentParser(description="count NMEA/UBX traffic on a serial port")
    ap.add_argument("-p", "--port", default="COM23")
    ap.add_argument("-b", "--baud", type=int, default=115200,
                    help="speed of the bridge's USB port, not the GNSS UART baud")
    ap.add_argument("-d", "--duration", type=float, default=10.0, help="seconds to capture")
    ap.add_argument("-o", "--out", default=None, help="also dump raw bytes to this file")
    args = ap.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.2)
    except serial.SerialException as exc:
        print(f"cannot open {args.port}: {exc}", file=sys.stderr)
        print("close the port in u-center first (Receiver -> Disconnect)", file=sys.stderr)
        return 2

    counts = Counter()
    bytes_ = Counter()
    ck_bad = Counter()
    junk_runs = 0
    junk_bytes = 0
    raw_file = None
    if args.out:
        # Created only now, so a busy port reports the port, not the file.
        parent = os.path.dirname(args.out)
        if parent:
            os.makedirs(parent, exist_ok=True)
        raw_file = open(args.out, "wb")
    splitter = StreamSplitter()

    print(f"listening {args.port} @ {args.baud} for {args.duration:g} s ...")
    started = time.monotonic()
    try:
        while time.monotonic() < started + args.duration:
            try:
                chunk = ser.read(4096)
            except serial.SerialException as exc:
                print(f"\nport dropped: {exc}", file=sys.stderr)
                break
            if not chunk:
                continue
            if raw_file:
                raw_file.write(chunk)
            for ev in splitter.feed(chunk):
                if ev[0] == "JUNK":
                    junk_runs += 1
                    junk_bytes += ev[2]
                    continue
                kind, ident, size, valid = ev
                counts[(kind, ident)] += 1
                bytes_[(kind, ident)] += size
                if kind == "NMEA" and not valid:
                    ck_bad[(kind, ident)] += 1
    finally:
        if raw_file:
            raw_file.close()
        ser.close()

    elapsed = max(time.monotonic() - started, 1e-6)
    total_msgs = sum(counts.values())
    print()
    print(f"captured {splitter.total} bytes in {elapsed:.2f} s -> "
          f"{splitter.total / elapsed:,.0f} B/s ({splitter.total * 8 / elapsed:,.0f} bit/s), "
          f"{total_msgs} frames")
    if not total_msgs:
        print("no frames recognised - wrong port/baud, or nothing is sending")
        return 1

    print()
    print(f"{'type':<12} {'kind':<6} {'msg/s':>8} {'B/s':>10} {'share':>7}  cksum")
    print("-" * 58)
    for (kind, ident), n in counts.most_common():
        bps = bytes_[(kind, ident)] / elapsed
        share = 100.0 * bytes_[(kind, ident)] / splitter.total
        if kind == "UBX":
            note = "-"
        elif ck_bad[(kind, ident)]:
            note = f"{ck_bad[(kind, ident)]} bad"
        else:
            note = "ok"
        print(f"{ident:<12} {kind:<6} {n / elapsed:>8.2f} {bps:>10,.0f} {share:>6.1f}%  {note}")

    if junk_runs:
        print(f"\n{junk_runs} junk runs, {junk_bytes} bytes outside any frame")
    if args.out:
        print(f"raw bytes saved to {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
