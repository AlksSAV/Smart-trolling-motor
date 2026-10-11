#include <Arduino.h>

// Raw byte tunnel: USB CDC (Serial) <-> UART1 (GNSS module).
//
// Purpose: there is no USB-UART adapter, so u-center talks to the GNSS module
// through the ESP32-S3 native USB port. Built as a separate firmware
// (env gnss_bridge) because it must print NOTHING: u-center parses this very
// stream, and any log byte from ESP32 breaks UBX/NMEA framing.
//
// Pins per docs/pinmap.md. Baud 38400 is the verified module setting.

#define GNSS_RX 18
#define GNSS_TX 17
#define GNSS_BAUD 38400

HardwareSerial gnssSerial(1);

void setup() {
    Serial.begin(115200);
    // No while (!Serial): the tunnel must forward bytes even with no host
    // attached, and must never hold reset waiting for DTR.
    gnssSerial.begin(GNSS_BAUD, SERIAL_8N1, GNSS_RX, GNSS_TX);
}

void loop() {
    static uint8_t buf[256];

    // GNSS -> u-center
    int n = gnssSerial.available();
    if (n > 0) {
        if (n > (int) sizeof(buf)) n = sizeof(buf);
        n = gnssSerial.read(buf, n);
        if (n > 0) Serial.write(buf, n);
    }

    // u-center -> GNSS
    n = Serial.available();
    if (n > 0) {
        if (n > (int) sizeof(buf)) n = sizeof(buf);
        n = Serial.read(buf, n);
        if (n > 0) gnssSerial.write(buf, n);
    }
}
