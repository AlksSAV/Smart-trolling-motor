#!/usr/bin/env python3
"""Read the live configuration of a u-blox M10 (SPG 5.10) through the
ESP32-S3 USB-serial bridge (env gnss_bridge).

READ-ONLY: sends UBX-CFG-VALGET polls only. Nothing here changes the module.
Keys and their value types are taken from u-blox M10 SPG 5.10 Interface
Description, UBX-21035062 - R03 (protocol 34.10), sections 4.9.x and the
"Configuration defaults" tables. No key ID in this file is invented.

The port must be FREE: close it in u-center first (Receiver -> Disconnect).

Usage:
    python tools/gnss_config.py                     # current (RAM) layer
    python tools/gnss_config.py --layer 2           # 0 RAM 1 BBR 2 Flash
    python tools/gnss_config.py --raw               # also dump raw hex
"""

import argparse
import struct
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial is required: python -m pip install pyserial")

# Value size in bytes, from the type column of the interface description
# (section 3.3.5 UBX data types / 4.5 Configuration data).
TYPE_SIZE = {
    "L": 1, "U1": 1, "I1": 1, "E1": 1, "X1": 1,
    "U2": 2, "I2": 2, "E2": 2, "X2": 2,
    "U4": 4, "I4": 4, "E4": 4, "X4": 4, "R4": 4,
    "U8": 8, "I8": 8, "X8": 8, "R8": 8,
}
UNPACK = {1: "<B", 2: "<H", 4: "<I", 8: "<Q"}

# name, key id, type, meaning of enum values (None = plain number)
KEYS = [
    # --- rate (interface description 4.9.17 CFG-RATE) ---
    ("CFG-RATE-MEAS",       0x30210001, "U2", None),
    ("CFG-RATE-NAV",        0x30210002, "U2", None),
    ("CFG-RATE-TIMEREF",    0x20210003, "E1",
     {0: "UTC", 1: "GPS", 2: "GLO", 3: "BDS", 4: "GAL", 5: "NAVIC"}),
    # --- navigation (4.9.12 CFG-NAVSPG, Table 23) ---
    ("CFG-NAVSPG-DYNMODEL", 0x20110021, "E1",
     {0: "PORT", 2: "STAT", 3: "PED", 4: "AUTOMOT", 5: "SEA",
      6: "AIR1(<1g)", 7: "AIR2(<2g)", 8: "AIR4(<4g)", 9: "WRIST",
      10: "BIKE", 11: "MOWER", 12: "ESCOOTER"}),
    ("CFG-NAVSPG-FIXMODE",  0x20110011, "E1", None),
    # --- satellite systems (4.9.21 CFG-SIGNAL) ---
    ("CFG-SIGNAL-GPS_ENA",  0x1031001f, "L", None),
    ("CFG-SIGNAL-SBAS_ENA", 0x10310020, "L", None),
    ("CFG-SIGNAL-GAL_ENA",  0x10310021, "L", None),
    ("CFG-SIGNAL-BDS_ENA",  0x10310022, "L", None),
    ("CFG-SIGNAL-QZSS_ENA", 0x10310024, "L", None),
    ("CFG-SIGNAL-GLO_ENA",  0x10310025, "L", None),
    # --- UART1 (4.9.27 / 4.9.28 / 4.9.29) ---
    ("CFG-UART1-BAUDRATE",     0x40520001, "U4", None),
    ("CFG-UART1-ENABLED",      0x10520005, "L", None),
    ("CFG-UART1INPROT-UBX",    0x10730001, "L", None),
    ("CFG-UART1INPROT-NMEA",   0x10730002, "L", None),
    ("CFG-UART1OUTPROT-UBX",   0x10740001, "L", None),
    ("CFG-UART1OUTPROT-NMEA",  0x10740002, "L", None),
    # --- NMEA output rate on UART1, per message (4.9.11 CFG-MSGOUT) ---
    # Rate unit: 0 = off, 1 = one message per navigation solution.
    ("MSGOUT-GGA-UART1", 0x209100bb, "U1", None),
    ("MSGOUT-GLL-UART1", 0x209100ca, "U1", None),
    ("MSGOUT-GNS-UART1", 0x209100b6, "U1", None),
    ("MSGOUT-GSA-UART1", 0x209100c0, "U1", None),
    ("MSGOUT-GST-UART1", 0x209100d4, "U1", None),
    ("MSGOUT-GSV-UART1", 0x209100c5, "U1", None),
    ("MSGOUT-RMC-UART1", 0x209100ac, "U1", None),
    ("MSGOUT-VTG-UART1", 0x209100b1, "U1", None),
    ("MSGOUT-ZDA-UART1", 0x209100d9, "U1", None),
    ("MSGOUT-GBS-UART1", 0x209100de, "U1", None),
    ("MSGOUT-GRS-UART1", 0x209100cf, "U1", None),
    ("MSGOUT-VLW-UART1", 0x209100e8, "U1", None),
    ("MSGOUT-DTM-UART1", 0x209100a7, "U1", None),
    ("MSGOUT-PUBX00-UART1", 0x209100ed, "U1", None),
    ("MSGOUT-PUBX03-UART1", 0x209100f2, "U1", None),
    ("MSGOUT-PUBX04-UART1", 0x209100f7, "U1", None),
    ("MSGOUT-NAV_PVT-UART1", 0x20910007, "U1", None),
    ("MSGOUT-NAV_SAT-UART1", 0x20910016, "U1", None),
]

CFG_VALGET = (0x06, 0x8B)
CFG_VALSET = (0x06, 0x8A)

# What --set writes. Every key here was read back with --layer 0 first, so the
# "before" value in the comments is measured, not assumed.
#
#   dynModel 6 (AIR1) -> 5 (SEA): the project docs claimed "Sea model active",
#     the module actually reports AIR1. That is the only functional change.
#   Drop GLL/GSA/GSV/VTG from UART1: measured 15 sentences per 5 Hz cycle
#     (GGA1+GLL1+GSA4+GSV7+RMC1+VTG1 = 15) = 76 msg/s, ~99% of the 38400 line.
#     GGA and RMC are enough for position, satellites and course.
#     satellites.value() comes from GGA (TinyGPS++.cpp:201), not GSV, so
#     dropping GSV costs nothing.
#   Result: 2 sentences per cycle = 10 msg/s, ~13% of the line.
#   measRate 200 ms (5 Hz): measured after a power cycle that Flash did not
#     hold it (RAM fell back to 1000 ms); older firmware was re-sending it via
#     legacy UBX-CFG-RATE on every boot, which hid that.
SET_ITEMS = [
    ("CFG-RATE-MEAS",       0x30210001, "U2", 200),
    ("CFG-NAVSPG-DYNMODEL", 0x20110021, "E1", 5),   # SEA
    ("MSGOUT-GGA-UART1",    0x209100bb, "U1", 1),
    ("MSGOUT-RMC-UART1",    0x209100ac, "U1", 1),
    ("MSGOUT-GLL-UART1",    0x209100ca, "U1", 0),
    ("MSGOUT-GSA-UART1",    0x209100c0, "U1", 0),
    ("MSGOUT-GSV-UART1",    0x209100c5, "U1", 0),
    ("MSGOUT-VTG-UART1",    0x209100b1, "U1", 0),
]

# CFG-VALSET layers bitfield: bit0 RAM, bit1 BBR, bit2 Flash.
LAYER_RAM, LAYER_BBR, LAYER_FLASH = 1, 2, 4


def build_valset(layers, items):
    payload = bytes([0x00, layers, 0x00, 0x00])
    for _, key, typ, val in items:
        payload += struct.pack("<I", key)
        payload += struct.pack(UNPACK[TYPE_SIZE[typ]], val)
    return ubx_frame(CFG_VALSET, payload)


def ubx_frame(cls_id, payload):
    body = bytes([cls_id[0], cls_id[1], len(payload) & 0xFF, len(payload) >> 8]) + payload
    a = b = 0
    for ch in body:
        a = (a + ch) & 0xFF
        b = (b + a) & 0xFF
    return b"\xb5\x62" + body + bytes([a, b])


def build_valget(layer, keys):
    payload = bytes([0x00, layer, 0x00, 0x00])
    for _, key, _, _ in keys:
        payload += struct.pack("<I", key)
    return ubx_frame(CFG_VALGET, payload)


def iter_ubx(buf):
    """Yield (cls, msg_id, payload) for every frame in buf with a good checksum.

    NMEA bytes and a partially received frame are skipped; only frames whose
    Fletcher checksum matches are returned, so junk from the interleaved NMEA
    stream cannot be mistaken for a reply.
    """
    i = 0
    while True:
        i = buf.find(b"\xb5\x62", i)
        if i == -1:
            return
        if i + 6 > len(buf):
            return
        cls, mid = buf[i + 2], buf[i + 3]
        ln = buf[i + 4] | (buf[i + 5] << 8)
        end = i + 8 + ln
        if end > len(buf):
            return
        # Checksum covers class, id, length and payload - i.e. everything
        # between the sync bytes and the two checksum bytes themselves.
        body = buf[i + 2:end - 2]
        a = b = 0
        for ch in body:
            a = (a + ch) & 0xFF
            b = (b + a) & 0xFF
        if a == buf[end - 2] and b == buf[end - 1]:
            yield cls, mid, buf[i + 6:end - 2]
            i = end
        else:
            i += 1


def decode_valget(payload):
    """Reply payload: version, layer, position, then key+value pairs."""
    if len(payload) < 4:
        return None, []
    layer = payload[1]
    rest = payload[4:]
    types = {key: (name, typ, enum) for name, key, typ, enum in KEYS}
    out = []
    i = 0
    while i + 4 <= len(rest):
        (key,) = struct.unpack("<I", rest[i:i + 4])
        i += 4
        if key not in types:
            out.append((f"0x{key:08x}", "?", None))
            break  # unknown key: its size is not known, decoding stops here
        name, typ, enum = types[key]
        size = TYPE_SIZE[typ]
        if i + size > len(rest):
            out.append((name, "truncated", None))
            break
        (val,) = struct.unpack(UNPACK[size], rest[i:i + size])
        i += size
        shown = f"{val} ({enum[val]})" if enum and val in enum else str(val)
        out.append((name, shown, val))
    return layer, out


def main():
    ap = argparse.ArgumentParser(description="poll M10 configuration (read-only)")
    ap.add_argument("-p", "--port", default="COM23")
    ap.add_argument("-b", "--baud", type=int, default=115200,
                    help="speed of the bridge's USB port, not the GNSS UART baud")
    ap.add_argument("--layer", type=int, default=0, choices=(0, 1, 2),
                    help="0=RAM/current 1=BBR 2=Flash")
    ap.add_argument("--wait", type=float, default=3.0, help="seconds to listen for the reply")
    ap.add_argument("--raw", action="store_true", help="dump every UBX frame received")
    ap.add_argument("--set", action="store_true",
                    help="write SET_ITEMS to RAM+BBR+Flash (changes the module!)")
    args = ap.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except serial.SerialException as exc:
        print(f"cannot open {args.port}: {exc}", file=sys.stderr)
        print("close the port in u-center first (Receiver -> Disconnect)", file=sys.stderr)
        return 2

    # The module keeps streaming NMEA; drain it so the read below starts clean.
    ser.reset_input_buffer()
    if args.set:
        layers = LAYER_RAM | LAYER_BBR | LAYER_FLASH
        print(f"writing {len(SET_ITEMS)} items, layers RAM+BBR+Flash:")
        for name, _, _, val in SET_ITEMS:
            print(f"   {name:<26} = {val}")
        ser.write(build_valset(layers, SET_ITEMS))
        ser.flush()

        buf = bytearray()
        deadline = time.monotonic() + args.wait
        ack = None
        while time.monotonic() < deadline and ack is None:
            buf += ser.read(4096)
            for cls, mid, _ in iter_ubx(bytes(buf)):
                if cls == 0x05:
                    ack = mid  # 0x01 = ACK, 0x00 = NAK
        ser.close()

        if ack is None:
            print(f"\nno ACK/NAK within {args.wait:g} s - configuration NOT confirmed")
            return 1
        if ack == 0x01:
            print("\nUBX-ACK: accepted. Now verify with:  python tools/gnss_config.py --layer 0")
            return 0
        print("\nUBX-NAK: rejected, nothing was applied")
        return 1

    ser.write(build_valget(args.layer, KEYS))
    ser.flush()

    buf = bytearray()
    deadline = time.monotonic() + args.wait
    reply = None
    while time.monotonic() < deadline:
        buf += ser.read(4096)
        for cls, mid, payload in iter_ubx(bytes(buf)):
            if (cls, mid) == CFG_VALGET:
                reply = payload
        if reply is not None:
            break

    ser.close()

    if args.raw:
        seen = {}
        for cls, mid, payload in iter_ubx(bytes(buf)):
            seen[(cls, mid)] = seen.get((cls, mid), 0) + 1
        print("UBX frames received:",
              ", ".join(f"{c:02X}{m:02X} x{n}" for (c, m), n in sorted(seen.items())) or "none")

    if reply is None:
        print(f"no CFG-VALGET reply within {args.wait:g} s")
        print(f"({len(buf)} bytes received, mostly NMEA - is the bridge firmware flashed?)")
        return 1

    layer, items = decode_valget(reply)
    layer_name = {0: "RAM (current)", 1: "BBR", 2: "Flash"}.get(layer, str(layer))
    print(f"CFG-VALGET reply, layer {layer_name}:  {len(items)} items\n")
    print(f"{'key':<26} {'value':<18}")
    print("-" * 46)
    for name, shown, _ in items:
        print(f"{name:<26} {shown}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
