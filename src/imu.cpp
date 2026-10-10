// Smart Trolling Motor - IMU (BNO085) driver implementation.

#include "imu.h"

bool ImuDriver::begin(uint8_t addr) {
    _addr = addr;
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    // ACK probe only: reports whether anything answers at this address.
    Wire.beginTransmission(_addr);
    return Wire.endTransmission() == 0;
}
