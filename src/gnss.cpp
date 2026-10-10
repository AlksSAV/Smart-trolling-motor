// Smart Trolling Motor - GNSS (NEO-M10N) driver implementation.

#include "gnss.h"

void GnssDriver::begin(long baud) {
    _serial = &Serial1;
    _serial->begin(baud, SERIAL_8N1, PIN_GNSS_RX, PIN_GNSS_TX);
}

bool GnssDriver::readNmea(char *buf, size_t len, uint32_t timeout_ms) {
    if (!_serial->available()) return false;
    uint32_t start = millis();
    size_t i = 0;
    while (i < len - 1) {
        if (_serial->available()) {
            char c = _serial->read();
            buf[i++] = c;
            if (c == '\n') break;
        }
        if (millis() - start > timeout_ms) break;
    }
    buf[i] = '\0';
    return i > 0;
}
