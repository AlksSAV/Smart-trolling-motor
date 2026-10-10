// Smart Trolling Motor - GNSS (NEO-M10N) driver.
// UART1 pins 17/18 from docs/pinmap.md.
// Baud rate is NOT known: docs/TODO.md lists 38400 and 115200 as candidates,
// both unverified. No default is given here on purpose.

#ifndef GNSS_H
#define GNSS_H

#include <Arduino.h>

// Pins from docs/pinmap.md
#define PIN_GNSS_TX  17
#define PIN_GNSS_RX  18

class GnssDriver {
public:
    void begin(long baud = 9600);
    bool readNmea(char *buf, size_t len, uint32_t timeout_ms = 100);
    HardwareSerial& serial() { return *_serial; }

private:
    HardwareSerial* _serial = nullptr;
};

#endif // GNSS_H
