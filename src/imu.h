// Smart Trolling Motor - IMU (BNO085) driver.
// I2C pins 12/13 from docs/pinmap.md.
// I2C address of the module is not documented in this repo - no default given.
// BNO085 data protocol is not implemented; no register layout is claimed here.

#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>

// Pins from docs/pinmap.md
#define I2C_SDA_PIN  12
#define I2C_SCL_PIN  13

class ImuDriver {
public:
    // addr must come from a bus scan or the module datasheet.
    bool begin(uint8_t addr);

private:
    uint8_t _addr;
};

#endif // IMU_H
