// Smart Trolling Motor - main entry point.
// Tests GNSS (UART1) and BNO085 (I2C) per TODO.md.

#include <Arduino.h>
#include "gnss.h"
#include "imu.h"

GnssDriver gnss;
ImuDriver imu;

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Smart Trolling Motor - GNSS + IMU test");

    // Board check per docs/platformio.md "Verification after first flash".
    Serial.printf("chip:    %s\n", ESP.getChipModel());
    Serial.printf("flash:   %lu MB\n", ESP.getFlashChipSize() / 1048576UL);
    if (psramFound()) {
        Serial.printf("psram:   found, size %lu bytes, free %lu bytes\n",
                      ESP.getPsramSize(), ESP.getFreePsram());
    } else {
        Serial.println("psram:   NOT FOUND");
    }

    // IMU I2C for BNO085 (pins 12, 13 from docs/pinmap.md)
    if (!imu.begin()) {
        Serial.println("IMU: no device on I2C bus");
    }

    // GNSS UART1
    gnss.begin(9600);
    Serial.println("GNSS UART1 started at 9600 baud");
}

void loop() {
    // Read GNSS NMEA
    char nmea[128];
    if (gnss.readNmea(nmea, sizeof(nmea), 100)) {
        Serial.print("GNSS: ");
        Serial.println(nmea);
    }

    // Read IMU
    int16_t ax, ay, az, gx, gy, gz, qx, qy, qz, qw, mx, my, mz;
    if (imu.getRawSensorData(&ax, &ay, &az, &gx, &gy, &gz, &qx, &qy, &qz, &qw, &mx, &my, &mz)) {
        Serial.printf("IMU: A=%d,%d,%d G=%d,%d,%d Q=%d,%d,%d,%d\n", ax, ay, az, gx, gy, gz, qx, qy, qz, qw);
    }

    delay(500);
}
