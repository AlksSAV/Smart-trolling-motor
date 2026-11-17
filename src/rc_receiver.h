#pragma once

#include <Arduino.h>

// RC receiver channels: HotRC DS650 with individual PWM per channel.
// Pins per docs/pinmap.md:
// GPIO  4   5   6   7   10  11
// CH   CH1 CH2 CH3 CH4 CH5 CH6
// Only CH1..CH3 are wired for now; CH4..CH6 stay attached but will read
// "lost" when nothing drives them.
//
// Levels: the receiver signal level (3.3/5 V) is NOT verified yet (TODO).
// If it turns out to be 5 V, a level shifter is required before wiring.

#define RC_CHANNELS 6

#define RC_MIN_US        1000
#define RC_MAX_US        2000
#define RC_NEUTRAL_US    1500
#define RC_HALF_RANGE_US 500  // 1000..2000 => +/-500 around neutral

// A PWM receiver frame repeats at ~50 Hz (20 ms). Missing pulses for longer
// than this means the signal is gone.
#define RC_STALE_MS      50

// Acceptable pulse range for the ISR: slightly wider than nominal.
#define RC_PULSE_MIN_ACCEPTED_US 880
#define RC_PULSE_MAX_ACCEPTED_US 2120

// Throttle deadband (us): ignore pulses within this range of neutral.
#define RC_THROTTLE_DEADBAND_US 30

// Full stick maps to this steering angle (matches SERVO_RANGE_DEG in main).
#define RC_STEER_RANGE_DEG 90.0f

// Channel assignment (0-based index into rc_pins).
// Confirm on the transmitter which stick/switch drives which channel.
#define RC_CH_STEERING 0   // CH1
#define RC_CH_THROTTLE 1   // CH2
#define RC_CH_MODE     2   // CH3

// 3-position switch thresholds on the mode channel:
//   < 1300 us  -> 0 (MANUAL)
//   1300-1700  -> 1 (HEADING_HOLD)
//   > 1700 us  -> 2 (ANCHOR_HOLD)
#define RC_MODE_LOW_US  1300
#define RC_MODE_HIGH_US 1700

// A mode must be stable for this long before it is reported as a change,
// so switch bounce and a mid-sweep of a proportional stick do not chatter.
#define RC_MODE_STABLE_MS 100

static const int8_t rc_pins[RC_CHANNELS] = {4, 5, 6, 7, 10, 11};

class RcReceiver {
public:
    RcReceiver();

    // Attach one CHANGE interrupt per channel pin.
    void begin();

    // Call from loop(): marks channels stale when no pulse arrived for
    // RC_STALE_MS and runs the mode-change debouncing.
    void update();

    // Pulse width in us; 0 if the channel is lost or has no pulse yet.
    int16_t readChannel(int ch) const;

    bool isLost(int ch) const;
    bool anyLost() const;
    bool allLost() const;

    // 0/1/2 from the mode channel thresholds, -1 if the channel is lost.
    int detectMode() const;

    // Returns the new mode once per settled change (stable for
    // RC_MODE_STABLE_MS), otherwise -1.
    int pollModeChange();

    // Steering angle from CH1, proportional to the stick: -90..+90 deg with a
    // deadband around neutral. 0 if the channel is lost.
    float steerAngleDeg() const;

    // Throttle percentage from CH2: -100..0..+100.
    float throttlePercent() const;

    // True once at least one valid pulse has been seen on any channel.
    // Until then the receiver is simply not wired/powered, and a caller must
    // not treat the resulting "all lost" state as a failsafe event.
    bool hasSeenSignal() const { return signal_seen_; }

private:
    // One instance is expected; the ISR receives a pointer to the Channel
    // it belongs to, so no global instance pointer is needed.
    struct Channel {
        uint8_t pin;
        volatile uint32_t rising_us;      // micros() at the rising edge
        volatile int16_t pulse_us;        // last measured pulse width
        volatile uint32_t last_valid_ms;  // millis() of the last valid pulse
        volatile bool lost;
    };

    Channel channels_[RC_CHANNELS];

    // ISR-visible state: accessed from IRAM, not part of Channel because
    // static arrays are easier for the ISR to index.
    static uint32_t rising_us_[RC_CHANNELS];
    static volatile uint32_t pulse_us_isr_[RC_CHANNELS];
    static volatile uint32_t pulse_seq_[RC_CHANNELS];
    uint32_t last_seq_[RC_CHANNELS];

    // Mode debouncing state.
    uint32_t pending_since_ms_ = 0;
    int pending_mode_ = -1;
    int reported_mode_ = -1;
    bool signal_seen_ = false;

    // Single ISR per pin: the core keeps one handler slot per pin, so
    // RISING and FALLING cannot be attached separately. One CHANGE handler
    // reads the pin level to tell the edges apart. digitalRead(), millis()
    // and micros() carry ARDUINO_ISR_ATTR in core 3.3.x, so they are safe
    // to call from an IRAM ISR.
    static void IRAM_ATTR handleEdge(void *arg);
};
