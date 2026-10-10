# PlatformIO: build and flashing

## Environment (verified on 2026-10-10)

- PlatformIO Core 6.2.0 (`python -m platformio --version`), Python 3.12.4
- Platform `espressif32@55.3.311` (Espressif official, esptool v5.x)
- Framework `arduino @ 3.3.8` (resolved from `framework-arduinoespressif32@3.3.8`)
- `pio` is available in PATH (updated from 6.1.19)
- ESP32-S3-N16R8 connected on COM23
- Build: success, no packages downloaded (all present on disk)

## Version drift policy

`platformio.ini` pins platform and framework versions. Pins are maintained in
this file and verified against the installed environment.

```ini
platform = espressif32 @ 55.3.311
framework = arduino @ 3.3.8
```

Platform 55.x is Espressif's official PlatformIO package (espressif/openocd-esp32
v5.x, esptool v5.x). Platform 6.x is community-maintained and has been reported
to fail on Windows with `esptool.exe` "SyntaxError: unexpected EOF while parsing"
when reading the chip bootloader; 55.x does not have that failure.

`arduino @ 3.3.8` is the framework core version actually resolved by PlatformIO.
`framework-arduinoespressif32@3.3.3` may still be present on disk as an unused
package; the active build uses 3.3.8 (see the global package storage
`platformio-pkgstate.json`). Do not assume the on-disk directory listing reflects
the version used by a given build — check the build state.

When bumping either pin, update this file, run `pio run`, and verify the
board still boots and the serial monitor works.

## Commands

Run from `D:\Troll Motor`:

| Command | Purpose |
|---------|---------|
| `python -m platformio run` | Build |
| `python -m platformio run -t upload` | Flash |
| `python -m platformio device monitor` | Monitor (115200) |
| `python -m platformio device list` | Ports |

## Why the board is overridden

The `esp32-s3-devkitc-1` board in platform 55.3.311 describes an **N8** module.
Actual `board.json` in the installed platform:

| Field | Board JSON | Our module (N16R8) |
|-------|-----------|--------------------|
| `flash_size` | 8MB | 16MB |
| `memory_type` | `qio_qspi` | `qio_opi` (octal PSRAM) |
| `partitions_file` | `default.csv` | `default_16MB.csv` |
| `BOARD_HAS_PSRAM` | defined | required |

`BOARD_HAS_PSRAM` is already defined by the board's `extra_flags`, but
`memory_type` is wrong: `qio_qspi` configures quad-SPI PSRAM, while the R8
module has **octal** PSRAM. Combined with the 8 MB flash size and the 8 MB
partition table, the stock board definition cannot address this module.

The overrides in `platformio.ini` correct exactly those fields. They are not
optional and must not be removed when bumping platform versions.

### Applied overrides (platformio.ini)

| Override | Purpose |
|----------|---------|
| `board_upload.flash_size = 16MB` | Tell esptool the real flash size |
| `board_build.partitions = default_16MB.csv` | 16 MB partition layout |
| `board_build.arduino.memory_type = qio_opi` | Octal PSRAM (not quad-SPI) |
| `board_build.filesystem = littlefs` | LittleFS over SPIFFS |

### Partition table verification

`board_build.partitions = default_16MB.csv` is confirmed applied.
The generated `.pio/build/esp32s3/partitions.bin` contains 6 entries spanning
exactly 16 MB:

| label    | type | offset | size |
|----------|------|--------|------|
| nvs      | data | 0x9000 | 20 KB |
| otadata  | data | 0xE000 | 8 KB |
| app0     | app  | 0x10000 | 6400 KB |
| app1     | app  | 0x650000 | 6400 KB |
| spiffs   | data | 0xC90000 | 3456 KB |
| coredump | data | 0xFF0000 | 64 KB |

Total: 0x9000+20K, 0xE000+8K, 0x10000+6400K, 0x650000+6400K, 0xC90000+3456K, 0xFF0000+64K = 0x1000000 (16 MB).

## Verification after first flash

Expected output in monitor:

```
chip:    ESP32-S3 rev <number>
flash:   16 MB
psram:   found, size 8xxxxxx bytes, free ...
```

If flash is not 16 MB or psram says NOT FOUND — override is wrong,
verify board marking.

## Build status

Build succeeded on 2026-10-10:
- `pio run` — RAM 1.3% (4544 B), Flash 4% (287805 B)
- No packages downloaded, all present on disk
- `esptool.py v4.12.0` invoked with `--flash_size detect`

## Not verified

- Flash and monitor not verified — board connected on COM23 but not yet flashed.
