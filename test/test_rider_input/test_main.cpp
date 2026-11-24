#include <unity.h>
#include <string.h>

// D-047 item 1 / D-121 item 5 -- native tests of the handlebar push-to-talk core: debounce,
// press / release / one longPress per hold, the sequence counter and the 8-byte block layout
// of the generated ble_schema.h (`riderInput`).
#include "RiderInputCore.h"

namespace {

constexpr uint32_t kStep = 5u; // RiderInputModule's sample period
// Samples after a level change until it is accepted: the first sample starts the timer.
constexpr uint32_t kSettle = RIDER_PTT_DEBOUNCE_MS + kStep;

// Samples `down` every kStep ms from *t for durationMs; advances *t.
void hold(RiderInputCore& c, bool down, uint32_t durationMs, uint32_t* t) {
    for (uint32_t e = 0; e < durationMs; e += kStep) {
        *t += kStep;
        c.sample(down, *t);
    }
}

bool pop(RiderInputCore& c, uint8_t block[RIDER_INPUT_BLOCK_BYTES]) {
    return c.popBlock(block, RIDER_INPUT_BLOCK_BYTES) == RIDER_INPUT_BLOCK_BYTES;
}

uint32_t blockTime(const uint8_t* b) {
    return (uint32_t)b[BLE_RIDER_INPUT_DEVICE_TIME_MS_OFFSET] |
           ((uint32_t)b[BLE_RIDER_INPUT_DEVICE_TIME_MS_OFFSET + 1u] << 8u) |
           ((uint32_t)b[BLE_RIDER_INPUT_DEVICE_TIME_MS_OFFSET + 2u] << 16u) |
           ((uint32_t)b[BLE_RIDER_INPUT_DEVICE_TIME_MS_OFFSET + 3u] << 24u);
}

} // namespace

// The block layout, restated here so a schema change fails at compile time in the test too.
static_assert(BLE_RIDER_INPUT_TOTAL_BYTES == 8u, "riderInput block is 8 bytes");
static_assert(BLE_RIDER_INPUT_VERSION_OFFSET == 0u && BLE_RIDER_INPUT_SEQ_OFFSET == 1u &&
              BLE_RIDER_INPUT_DEVICE_TIME_MS_OFFSET == 2u && BLE_RIDER_INPUT_EVENT_OFFSET == 6u &&
              BLE_RIDER_INPUT_FLAGS_OFFSET == 7u, "riderInput offsets");

void setUp() {}
void tearDown() {}

void test_press_and_release_after_debounce() {
    RiderInputCore c(7);
    uint32_t t = 1000u;
    uint8_t b[RIDER_INPUT_BLOCK_BYTES];
    hold(c, true, RIDER_PTT_DEBOUNCE_MS, &t); // the first sample starts the timer
    TEST_ASSERT_FALSE(c.pending());       // not yet debounced
    hold(c, true, kStep, &t);
    TEST_ASSERT_TRUE(c.buttonDown());
    TEST_ASSERT_TRUE(pop(c, b));
    TEST_ASSERT_EQUAL_UINT8(BLE_RIDER_INPUT_EVENT_PRESS, b[BLE_RIDER_INPUT_EVENT_OFFSET]);
    TEST_ASSERT_EQUAL_UINT8(BLE_RIDER_INPUT_FLAG_BUTTON_DOWN, b[BLE_RIDER_INPUT_FLAGS_OFFSET]);
    TEST_ASSERT_EQUAL_UINT32(t, blockTime(b));
    TEST_ASSERT_FALSE(c.pending());

    hold(c, false, 100u, &t);
    TEST_ASSERT_FALSE(c.buttonDown());
    TEST_ASSERT_TRUE(pop(c, b));
    TEST_ASSERT_EQUAL_UINT8(BLE_RIDER_INPUT_EVENT_RELEASE, b[BLE_RIDER_INPUT_EVENT_OFFSET]);
    TEST_ASSERT_EQUAL_UINT8(0u, b[BLE_RIDER_INPUT_FLAGS_OFFSET]);
    TEST_ASSERT_FALSE(c.pending()); // no longPress for a short hold
}

void test_bouncing_input_gives_one_press_and_one_release() {
    RiderInputCore c(7);
    uint32_t t = 0u;
    // Contact bounce: short alternating levels, each below the debounce time.
    for (int i = 0; i < 4; i++) {
        hold(c, true, 10u, &t);
        hold(c, false, 10u, &t);
    }
    TEST_ASSERT_FALSE(c.pending());
    hold(c, true, 200u, &t); // settles down
    hold(c, true, 0u, &t);
    for (int i = 0; i < 4; i++) { // bounce on release
        hold(c, false, 10u, &t);
        hold(c, true, 10u, &t);
    }
    hold(c, false, 200u, &t);
    uint8_t b[RIDER_INPUT_BLOCK_BYTES];
    TEST_ASSERT_TRUE(pop(c, b));
    TEST_ASSERT_EQUAL_UINT8(BLE_RIDER_INPUT_EVENT_PRESS, b[BLE_RIDER_INPUT_EVENT_OFFSET]);
    TEST_ASSERT_TRUE(pop(c, b));
    TEST_ASSERT_EQUAL_UINT8(BLE_RIDER_INPUT_EVENT_RELEASE, b[BLE_RIDER_INPUT_EVENT_OFFSET]);
    TEST_ASSERT_FALSE(c.pending());
}

void test_exactly_one_long_press_per_hold() {
    RiderInputCore c(7);
    uint32_t t = 0u;
    uint8_t b[RIDER_INPUT_BLOCK_BYTES];
    for (int hold_no = 0; hold_no < 2; hold_no++) {
        hold(c, true, RIDER_PTT_LONG_PRESS_MS + 3000u, &t); // far longer than the long-press time
        hold(c, false, 100u, &t);
        TEST_ASSERT_TRUE(pop(c, b));
        TEST_ASSERT_EQUAL_UINT8(BLE_RIDER_INPUT_EVENT_PRESS, b[BLE_RIDER_INPUT_EVENT_OFFSET]);
        TEST_ASSERT_TRUE(pop(c, b));
        TEST_ASSERT_EQUAL_UINT8(BLE_RIDER_INPUT_EVENT_LONG_PRESS, b[BLE_RIDER_INPUT_EVENT_OFFSET]);
        TEST_ASSERT_EQUAL_UINT8(BLE_RIDER_INPUT_FLAG_BUTTON_DOWN, b[BLE_RIDER_INPUT_FLAGS_OFFSET]);
        TEST_ASSERT_TRUE(pop(c, b));
        TEST_ASSERT_EQUAL_UINT8(BLE_RIDER_INPUT_EVENT_RELEASE, b[BLE_RIDER_INPUT_EVENT_OFFSET]);
        TEST_ASSERT_FALSE(c.pending());
    }
}

void test_long_press_time_counts_from_the_debounced_press() {
    RiderInputCore c(7);
    uint32_t t = 0u;
    uint8_t b[RIDER_INPUT_BLOCK_BYTES];
    hold(c, true, kSettle, &t);
    TEST_ASSERT_TRUE(pop(c, b)); // press
    uint32_t pressAt = t;
    hold(c, true, RIDER_PTT_LONG_PRESS_MS - kStep, &t);
    TEST_ASSERT_FALSE(c.pending());
    hold(c, true, kStep, &t);
    TEST_ASSERT_TRUE(pop(c, b));
    TEST_ASSERT_EQUAL_UINT8(BLE_RIDER_INPUT_EVENT_LONG_PRESS, b[BLE_RIDER_INPUT_EVENT_OFFSET]);
    TEST_ASSERT_EQUAL_UINT32(pressAt + RIDER_PTT_LONG_PRESS_MS, blockTime(b));
}

void test_no_event_while_stable() {
    RiderInputCore c(7);
    uint32_t t = 0u;
    hold(c, false, 60000u, &t);
    TEST_ASSERT_FALSE(c.pending());
    hold(c, true, kSettle, &t);
    uint8_t b[RIDER_INPUT_BLOCK_BYTES];
    TEST_ASSERT_TRUE(pop(c, b));
    hold(c, true, RIDER_PTT_LONG_PRESS_MS, &t);
    TEST_ASSERT_TRUE(pop(c, b)); // the longPress
    hold(c, true, 20000u, &t);   // held on: nothing more
    TEST_ASSERT_FALSE(c.pending());
}

void test_seq_increments_per_block_and_wraps() {
    RiderInputCore c(7);
    uint32_t t = 0u;
    uint8_t b[RIDER_INPUT_BLOCK_BYTES];
    for (uint32_t i = 0; i < 300u; i++) { // 600 events: seq wraps twice
        hold(c, true, kSettle, &t);
        TEST_ASSERT_TRUE(pop(c, b));
        TEST_ASSERT_EQUAL_UINT8((uint8_t)(2u * i), b[BLE_RIDER_INPUT_SEQ_OFFSET]);
        hold(c, false, kSettle, &t);
        TEST_ASSERT_TRUE(pop(c, b));
        TEST_ASSERT_EQUAL_UINT8((uint8_t)(2u * i + 1u), b[BLE_RIDER_INPUT_SEQ_OFFSET]);
    }
}

void test_seq_does_not_advance_when_nothing_is_handed_out() {
    RiderInputCore c(7);
    uint32_t t = 0u;
    uint8_t b[RIDER_INPUT_BLOCK_BYTES];
    TEST_ASSERT_EQUAL_UINT32(0u, c.popBlock(b, sizeof(b))); // empty
    hold(c, true, kSettle, &t);
    TEST_ASSERT_EQUAL_UINT32(0u, c.popBlock(b, RIDER_INPUT_BLOCK_BYTES - 1u)); // buffer too small
    TEST_ASSERT_TRUE(c.pending());
    TEST_ASSERT_TRUE(pop(c, b));
    TEST_ASSERT_EQUAL_UINT8(0u, b[BLE_RIDER_INPUT_SEQ_OFFSET]);
}

void test_block_bytes_match_generated_offsets() {
    RiderInputEvent ev = {0x0A0B0C0Du, (uint8_t)BLE_RIDER_INPUT_EVENT_LONG_PRESS, 1u};
    uint8_t b[RIDER_INPUT_BLOCK_BYTES];
    memset(b, 0xEE, sizeof(b));
    TEST_ASSERT_EQUAL_UINT32(BLE_RIDER_INPUT_TOTAL_BYTES, packRiderInputBlock(ev, 0x42u, b, sizeof(b)));
    const uint8_t expected[8] = {(uint8_t)BLE_RIDER_INPUT_BLOCK_VERSION, 0x42u, 0x0Du, 0x0Cu, 0x0Bu, 0x0Au,
                                 (uint8_t)BLE_RIDER_INPUT_EVENT_LONG_PRESS, (uint8_t)BLE_RIDER_INPUT_FLAG_BUTTON_DOWN};
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, b, 8);
    ev.buttonDown = 0u;
    TEST_ASSERT_EQUAL_UINT32(8u, packRiderInputBlock(ev, 0u, b, sizeof(b)));
    TEST_ASSERT_EQUAL_UINT8(0u, b[BLE_RIDER_INPUT_FLAGS_OFFSET]);
    TEST_ASSERT_EQUAL_UINT32(0u, packRiderInputBlock(ev, 0u, b, 7u));
    TEST_ASSERT_EQUAL_UINT32(0u, packRiderInputBlock(ev, 0u, nullptr, 8u));
}

void test_clock_wrap_does_not_break_debounce() {
    RiderInputCore c(7);
    uint32_t t = 0xFFFFFFFFu - 40u; // wraps during the debounce and the long press
    uint8_t b[RIDER_INPUT_BLOCK_BYTES];
    hold(c, true, kSettle, &t);
    TEST_ASSERT_TRUE(pop(c, b));
    hold(c, true, RIDER_PTT_LONG_PRESS_MS, &t);
    TEST_ASSERT_TRUE(pop(c, b));
    TEST_ASSERT_EQUAL_UINT8(BLE_RIDER_INPUT_EVENT_LONG_PRESS, b[BLE_RIDER_INPUT_EVENT_OFFSET]);
}

void test_full_queue_keeps_older_events_and_counts_drops() {
    RiderInputCore c(7);
    uint32_t t = 0u;
    for (size_t i = 0; i < RIDER_PTT_QUEUE_DEPTH; i++) {
        hold(c, (i % 2u) == 0u, kSettle, &t); // press, release, ...
    }
    TEST_ASSERT_EQUAL_UINT32(0u, c.droppedEvents());
    hold(c, true, kSettle, &t);
    TEST_ASSERT_EQUAL_UINT32(1u, c.droppedEvents());
    uint8_t b[RIDER_INPUT_BLOCK_BYTES];
    size_t n = 0;
    while (pop(c, b)) {
        n++;
    }
    TEST_ASSERT_EQUAL_UINT32(RIDER_PTT_QUEUE_DEPTH, n);
}

void test_clear_drops_pending_events() {
    RiderInputCore c(7);
    uint32_t t = 0u;
    hold(c, true, kSettle, &t);
    TEST_ASSERT_TRUE(c.pending());
    c.clear();
    TEST_ASSERT_FALSE(c.pending());
}

void test_disabled_gpio_never_produces_output() {
    RiderInputCore def; // the default: no pin chosen
    RiderInputCore off(-1);
    TEST_ASSERT_FALSE(def.enabled());
    TEST_ASSERT_FALSE(off.enabled());
    uint32_t t = 0u;
    hold(def, true, 5000u, &t);
    hold(off, true, 5000u, &t);
    hold(off, false, 100u, &t);
    TEST_ASSERT_FALSE(def.pending());
    TEST_ASSERT_FALSE(off.pending());
    TEST_ASSERT_FALSE(off.buttonDown());
    uint8_t b[RIDER_INPUT_BLOCK_BYTES];
    TEST_ASSERT_EQUAL_UINT32(0u, off.popBlock(b, sizeof(b)));
    TEST_ASSERT_TRUE(RiderInputCore(0).enabled()); // GPIO 0 is a valid pin
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_press_and_release_after_debounce);
    RUN_TEST(test_bouncing_input_gives_one_press_and_one_release);
    RUN_TEST(test_exactly_one_long_press_per_hold);
    RUN_TEST(test_long_press_time_counts_from_the_debounced_press);
    RUN_TEST(test_no_event_while_stable);
    RUN_TEST(test_seq_increments_per_block_and_wraps);
    RUN_TEST(test_seq_does_not_advance_when_nothing_is_handed_out);
    RUN_TEST(test_block_bytes_match_generated_offsets);
    RUN_TEST(test_clock_wrap_does_not_break_debounce);
    RUN_TEST(test_full_queue_keeps_older_events_and_counts_drops);
    RUN_TEST(test_clear_drops_pending_events);
    RUN_TEST(test_disabled_gpio_never_produces_output);
    return UNITY_END();
}
