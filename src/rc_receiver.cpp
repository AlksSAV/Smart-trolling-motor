#include "rc_receiver.h"

// ISR-visible state. The ISR runs from IRAM and may fire on either core while
// loop() runs on the other, so these are the only variables it touches.
uint32_t RcReceiver::rising_us_[RC_CHANNELS];
volatile uint32_t RcReceiver::pulse_us_isr_[RC_CHANNELS];
volatile uint32_t RcReceiver::pulse_seq_[RC_CHANNELS];

// GPIO input status register, bits 0..31. ESP32-S3 has a second register
// (GPIO_IN1_REG at 0x60008040) for GPIO 32..48; every RC pin here is below 32,
// so the low register covers all of them.
static inline uint32_t gpioInStatus() {
    return *reinterpret_cast<volatile uint32_t *>(0x6000803C);
}

// arg packs (gpio number << 8) | channel index, so the handler reads no
// flash-resident table: only micros() (ARDUINO_ISR_ATTR in the core) and one
// MMIO register.
void IRAM_ATTR RcReceiver::handleEdge(void *arg) {
    uintptr_t packed = reinterpret_cast<uintptr_t>(arg);
    int ch  = static_cast<int>(packed & 0xFF);
    int pin = static_cast<int>(packed >> 8);
    if (ch < 0 || ch >= RC_CHANNELS) return;

    // A GPIO interrupt status latches until cleared, so a CHANGE handler can
    // see both edges. Read the pin to tell them apart: attachInterruptArg
    // enables the input path, so the register reflects the live level.
    bool high = (gpioInStatus() >> pin) & 1U;

    if (high) {
        rising_us_[ch] = micros();
        return;
    }

    // Wrap-safe: unsigned subtraction, so a micros() rollover (~71 min)
    // between the edges still yields the true width.
    uint32_t width = (uint32_t)(micros() - rising_us_[ch]);
    if (width >= RC_PULSE_MIN_ACCEPTED_US && width <= RC_PULSE_MAX_ACCEPTED_US) {
        pulse_us_isr_[ch] = width;
        // Read-modify-write, not ++: volatile compound assignment is
        // deprecated in C++20.
        pulse_seq_[ch] = pulse_seq_[ch] + 1;
    }
}

RcReceiver::RcReceiver() {
    for (int i = 0; i < RC_CHANNELS; i++) {
        channels_[i].pulse_us = 0;
        channels_[i].last_valid_ms = 0;
        channels_[i].lost = true;
        last_seq_[i] = 0;
    }
}

void RcReceiver::begin() {
    for (int i = 0; i < RC_CHANNELS; i++) {
        pinMode(rc_pins[i], INPUT);
        rising_us_[i] = 0;
        pulse_us_isr_[i] = 0;
        pulse_seq_[i] = 0;
        last_seq_[i] = 0;
    }
    // One handler per pin: the core keeps a single slot per GPIO, so RISING and
    // FALLING cannot both be attached. CHANGE covers both edges and the handler
    // sorts them out by reading the level.
    // arg = (pin << 8) | channel: the ISR gets both without touching rc_pins
    // (a flash-resident table) from interrupt context.
    for (int i = 0; i < RC_CHANNELS; i++) {
        uintptr_t packed = ((uintptr_t)rc_pins[i] << 8) | (uintptr_t)i;
        attachInterruptArg(rc_pins[i], RcReceiver::handleEdge,
                           reinterpret_cast<void *>(packed),
                           CHANGE);
    }
}

void RcReceiver::update() {
    uint32_t now = millis();
    for (int i = 0; i < RC_CHANNELS; i++) {
        uint32_t seq = pulse_seq_[i];
        if (seq != last_seq_[i]) {
            uint32_t width = pulse_us_isr_[i];
            // Re-read the counter: a pulse may have landed while we sampled.
            // Drop the stale reading instead of publishing a mixed value; the
            // next frame arrives in 20 ms.
            if (pulse_seq_[i] == seq) {
                channels_[i].pulse_us = (int16_t)width;
                channels_[i].last_valid_ms = now;
                channels_[i].lost = false;
                signal_seen_ = true;
            }
            last_seq_[i] = seq;
        } else if (!channels_[i].lost &&
                   channels_[i].last_valid_ms != 0 &&
                   (now - channels_[i].last_valid_ms) > RC_STALE_MS) {
            channels_[i].lost = true;
        }
    }
}

int16_t RcReceiver::readChannel(int ch) const {
    if (ch < 0 || ch >= RC_CHANNELS) return 0;
    if (channels_[ch].lost) return 0;
    return channels_[ch].pulse_us;
}

bool RcReceiver::isLost(int ch) const {
    if (ch < 0 || ch >= RC_CHANNELS) return true;
    return channels_[ch].lost;
}

bool RcReceiver::anyLost() const {
    for (int i = 0; i < RC_CHANNELS; i++) {
        if (channels_[i].lost) return true;
    }
    return false;
}

bool RcReceiver::allLost() const {
    for (int i = 0; i < RC_CHANNELS; i++) {
        if (!channels_[i].lost) return false;
    }
    return true;
}

int RcReceiver::detectMode() const {
    if (channels_[RC_CH_MODE].lost) return -1;
    int16_t pulse = channels_[RC_CH_MODE].pulse_us;
    if (pulse < RC_MODE_LOW_US)  return 0;  // MANUAL
    if (pulse <= RC_MODE_HIGH_US) return 1; // HEADING_HOLD
    return 2;                               // ANCHOR_HOLD
}

int RcReceiver::pollModeChange() {
    int mode = detectMode();
    if (mode < 0) {
        // No signal: keep the last reported mode so a dropout does not look
        // like a switch action.
        pending_mode_ = -1;
        pending_since_ms_ = 0;
        return -1;
    }

    if (mode == reported_mode_) {
        pending_mode_ = -1;
        pending_since_ms_ = 0;
        return -1;
    }

    uint32_t now = millis();
    if (mode != pending_mode_) {
        pending_mode_ = mode;
        pending_since_ms_ = now;
        return -1;
    }
    if ((now - pending_since_ms_) < RC_MODE_STABLE_MS) return -1;

    reported_mode_ = mode;
    pending_mode_ = -1;
    return mode;
}

float RcReceiver::steerAngleDeg() const {
    int16_t pulse = readChannel(RC_CH_STEERING);
    if (pulse == 0) return 0.0f;

    int16_t span = pulse - RC_NEUTRAL_US;
    if (span > RC_THROTTLE_DEADBAND_US)      span -= RC_THROTTLE_DEADBAND_US;
    else if (span < -RC_THROTTLE_DEADBAND_US) span += RC_THROTTLE_DEADBAND_US;
    else return 0.0f;

    // Same scaling as the throttle: full stick after the deadband = +/-90 deg.
    float angle = (float)span * RC_STEER_RANGE_DEG /
                  (float)(RC_HALF_RANGE_US - RC_THROTTLE_DEADBAND_US);
    if (angle > RC_STEER_RANGE_DEG)  angle = RC_STEER_RANGE_DEG;
    if (angle < -RC_STEER_RANGE_DEG) angle = -RC_STEER_RANGE_DEG;
    return angle;
}

float RcReceiver::throttlePercent() const {
    int16_t pulse = readChannel(RC_CH_THROTTLE);
    if (pulse == 0) return 0.0f;

    int16_t span = pulse - RC_NEUTRAL_US;
    if (span > RC_THROTTLE_DEADBAND_US)      span -= RC_THROTTLE_DEADBAND_US;
    else if (span < -RC_THROTTLE_DEADBAND_US) span += RC_THROTTLE_DEADBAND_US;
    else return 0.0f;

    // Usable stick travel after the deadband: 500 us on either side.
    float percent = (float)span * 100.0f / (float)(RC_HALF_RANGE_US - RC_THROTTLE_DEADBAND_US);
    if (percent > 100.0f)  percent = 100.0f;
    if (percent < -100.0f) percent = -100.0f;
    return percent;
}
