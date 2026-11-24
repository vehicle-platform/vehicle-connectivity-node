#ifndef RIDER_INPUT_CORE_H
#define RIDER_INPUT_CORE_H

#include <stddef.h>
#include <stdint.h>

#include "ble_schema.h" // generated: external/vehicle-defs/gen/c/conn/ (D-061)

// Handlebar push-to-talk button (D-047 item 1, D-121 item 5): the pure part. Raw button
// samples plus a millisecond clock go in, debounced press / release / longPress events come
// out as the 8-byte `riderInput` block of the defs BLE schema (ble_schema.json
// `riderInputBlock`, generated ble_schema.h). No ESP-IDF or Arduino include, no dynamic
// memory (invariant 5); the GPIO side is RiderInputModule.
//
// Informational only: nothing safety-related reads it and it never touches the TWAI or UDS
// code (invariant 1). The phone starts its voice session from this event (D-121 item 5).

// Firmware constants, not schema values: how the raw contact is turned into events.
constexpr uint32_t RIDER_PTT_DEBOUNCE_MS = 30u;    // a level must hold this long to count
constexpr uint32_t RIDER_PTT_LONG_PRESS_MS = 800u; // debounced hold time of the one longPress
constexpr size_t RIDER_PTT_QUEUE_DEPTH = 8u;       // events waiting for a BLE pass

// The block is built byte by byte in this order and with these sizes.
constexpr uint8_t RIDER_INPUT_BLOCK_VERSION = BLE_RIDER_INPUT_BLOCK_VERSION;
constexpr size_t RIDER_INPUT_BLOCK_BYTES = BLE_RIDER_INPUT_TOTAL_BYTES;
static_assert(BLE_RIDER_INPUT_VERSION_OFFSET == 0 && BLE_RIDER_INPUT_SEQ_OFFSET == 1 &&
              BLE_RIDER_INPUT_DEVICE_TIME_MS_OFFSET == 2 && BLE_RIDER_INPUT_EVENT_OFFSET == 6 &&
              BLE_RIDER_INPUT_FLAGS_OFFSET == 7,
              "rider input block field order must match the schema");
static_assert(BLE_RIDER_INPUT_VERSION_SIZE == 1 && BLE_RIDER_INPUT_SEQ_SIZE == 1 &&
              BLE_RIDER_INPUT_DEVICE_TIME_MS_SIZE == 4 && BLE_RIDER_INPUT_EVENT_SIZE == 1 &&
              BLE_RIDER_INPUT_FLAGS_SIZE == 1,
              "rider input block field sizes must match the schema");
static_assert(RIDER_INPUT_BLOCK_BYTES == BLE_RIDER_INPUT_FLAGS_OFFSET + BLE_RIDER_INPUT_FLAGS_SIZE,
              "rider input block total must match the schema");

// One event waiting to be sent. timeMs is the node clock when the debounced state changed
// (press, release) or when the hold reached RIDER_PTT_LONG_PRESS_MS (longPress).
struct RiderInputEvent {
    uint32_t timeMs;
    uint8_t event;      // BLE_RIDER_INPUT_EVENT_*
    uint8_t buttonDown; // debounced state right after this event, 1 = held
};

/**
 * @brief Writes one block: version, seq, deviceTimeMs (little-endian), event, flags.
 * @return RIDER_INPUT_BLOCK_BYTES, or 0 if out is too small.
 */
inline size_t packRiderInputBlock(const RiderInputEvent& ev, uint8_t seq, uint8_t* out, size_t outLen) {
    if (out == nullptr || outLen < RIDER_INPUT_BLOCK_BYTES) {
        return 0;
    }
    out[BLE_RIDER_INPUT_VERSION_OFFSET] = RIDER_INPUT_BLOCK_VERSION;
    out[BLE_RIDER_INPUT_SEQ_OFFSET] = seq;
    out[BLE_RIDER_INPUT_DEVICE_TIME_MS_OFFSET + 0u] = (uint8_t)ev.timeMs;
    out[BLE_RIDER_INPUT_DEVICE_TIME_MS_OFFSET + 1u] = (uint8_t)(ev.timeMs >> 8u);
    out[BLE_RIDER_INPUT_DEVICE_TIME_MS_OFFSET + 2u] = (uint8_t)(ev.timeMs >> 16u);
    out[BLE_RIDER_INPUT_DEVICE_TIME_MS_OFFSET + 3u] = (uint8_t)(ev.timeMs >> 24u);
    out[BLE_RIDER_INPUT_EVENT_OFFSET] = ev.event;
    out[BLE_RIDER_INPUT_FLAGS_OFFSET] = (ev.buttonDown != 0u) ? (uint8_t)BLE_RIDER_INPUT_FLAG_BUTTON_DOWN : 0u;
    return RIDER_INPUT_BLOCK_BYTES;
}

/**
 * @brief Debouncer, long-press timer, small event queue and block packer for one button.
 * A disabled core (gpio < 0, the default until the pin is chosen, Q-034) ignores samples and
 * never produces an event.
 */
class RiderInputCore {
public:
    explicit RiderInputCore(int gpio = -1) : _enabled(gpio >= 0) {}

    bool enabled() const { return _enabled; }

    /**
     * @brief Feeds one raw sample (true = button pressed, i.e. the pin is low) taken at nowMs.
     * Call at a fixed period well below RIDER_PTT_DEBOUNCE_MS. Uses unsigned time differences,
     * so the 32-bit clock may wrap.
     */
    void sample(bool rawDown, uint32_t nowMs) {
        if (!_enabled) {
            return;
        }
        if (rawDown != _candidate) {
            _candidate = rawDown; // the level changed: the debounce time starts again
            _candidateSinceMs = nowMs;
        }
        if (_candidate != _down && (uint32_t)(nowMs - _candidateSinceMs) >= RIDER_PTT_DEBOUNCE_MS) {
            _down = _candidate;
            _longSent = false;
            _downSinceMs = nowMs;
            push(nowMs, _down ? (uint8_t)BLE_RIDER_INPUT_EVENT_PRESS : (uint8_t)BLE_RIDER_INPUT_EVENT_RELEASE);
        }
        if (_down && !_longSent && (uint32_t)(nowMs - _downSinceMs) >= RIDER_PTT_LONG_PRESS_MS) {
            _longSent = true; // exactly one per hold
            push(nowMs, (uint8_t)BLE_RIDER_INPUT_EVENT_LONG_PRESS);
        }
    }

    bool pending() const { return _count > 0u; }

    /**
     * @brief Takes the oldest event as a packed block; the sequence number advances once per
     * block handed out. Returns RIDER_INPUT_BLOCK_BYTES, or 0 if nothing is pending or out is
     * too small (the event then stays queued).
     */
    size_t popBlock(uint8_t* out, size_t outLen) {
        if (_count == 0u) {
            return 0;
        }
        size_t len = packRiderInputBlock(_queue[_head], _seq, out, outLen);
        if (len == 0u) {
            return 0;
        }
        _head = (uint8_t)((_head + 1u) % RIDER_PTT_QUEUE_DEPTH);
        _count--;
        _seq++; // uint8_t: wraps 255 -> 0
        return len;
    }

    // Drops queued events (nobody connected); the debounced state and seq are kept.
    void clear() {
        _head = 0u;
        _count = 0u;
    }

    bool buttonDown() const { return _down; }
    uint32_t droppedEvents() const { return _dropped; }

private:
    void push(uint32_t timeMs, uint8_t event) {
        if (_count >= RIDER_PTT_QUEUE_DEPTH) {
            _dropped++; // the phone is not draining: keep the older events
            return;
        }
        RiderInputEvent& slot = _queue[(_head + _count) % RIDER_PTT_QUEUE_DEPTH];
        slot.timeMs = timeMs;
        slot.event = event;
        slot.buttonDown = _down ? 1u : 0u;
        _count++;
    }

    bool _enabled;
    bool _candidate = false; // last raw level
    uint32_t _candidateSinceMs = 0;
    bool _down = false; // debounced state
    uint32_t _downSinceMs = 0;
    bool _longSent = false;
    RiderInputEvent _queue[RIDER_PTT_QUEUE_DEPTH] = {};
    uint8_t _head = 0;
    uint8_t _count = 0;
    uint8_t _seq = 0;
    uint32_t _dropped = 0;
};

#endif // RIDER_INPUT_CORE_H
