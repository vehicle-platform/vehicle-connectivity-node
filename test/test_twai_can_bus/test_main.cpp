#include <unity.h>

// D-096 (closes ISSUES E-25 (4)): the tester's TWAI transmit never waits for TX queue space
// and never retries. TwaiCanBus.cpp is compiled here against the driver stub in
// test/native_stubs/driver/ ([env:native] builds no src/, so the file is included directly).
#include "../../src/hal/TwaiCanBus.h"
#include "../../src/hal/TwaiCanBus.cpp"

static CanFrame request() {
    CanFrame f{};
    f.id = 0x18DA10F1u;
    f.extended = true;
    f.dlc = 8;
    f.data[0] = 0x03;
    f.data[1] = 0x22;
    return f;
}

void setUp(void) { twaiStub() = TwaiStub(); }
void tearDown(void) {}

void test_transmit_passes_a_zero_wait_to_the_driver(void) {
    static_assert(TwaiCanBus::kTxWaitMs == 0u, "D-096: the tester transmit never waits");
    TwaiCanBus bus(GPIO_NUM_4, GPIO_NUM_5);
    TEST_ASSERT_TRUE(bus.enterNormalMode());
    TEST_ASSERT_TRUE(bus.transmit(request()));
    TEST_ASSERT_EQUAL_INT(1, twaiStub().transmitCalls);
    TEST_ASSERT_EQUAL_UINT32(0u, twaiStub().lastTransmitWait);
    TEST_ASSERT_EQUAL_HEX32(0x18DA10F1u, twaiStub().lastSent.identifier);
    // Only extd is set: ss, self, dlc_non_comp and the reserved bits are 0 (the message is
    // zeroed; a stray `self` would echo the request into RX as a foreign tester).
    TEST_ASSERT_EQUAL_HEX32(0x1u, twaiStub().lastSent.flags);
}

void test_a_standard_frame_carries_no_flag_bits(void) {
    TwaiCanBus bus(GPIO_NUM_4, GPIO_NUM_5);
    TEST_ASSERT_TRUE(bus.enterNormalMode());
    CanFrame f = request();
    f.extended = false;
    f.id = 0x7DFu;
    TEST_ASSERT_TRUE(bus.transmit(f));
    TEST_ASSERT_EQUAL_HEX32(0x0u, twaiStub().lastSent.flags);
}

void test_a_full_queue_drops_the_frame_without_wait_or_retry(void) {
    TwaiCanBus bus(GPIO_NUM_4, GPIO_NUM_5);
    TEST_ASSERT_TRUE(bus.enterNormalMode());
    twaiStub().txQueueFull = true;
    TEST_ASSERT_FALSE(bus.transmit(request()));
    TEST_ASSERT_EQUAL_INT(1, twaiStub().transmitCalls); // one driver call: no retry
    TEST_ASSERT_EQUAL_UINT32(0u, twaiStub().lastTransmitWait);
    TEST_ASSERT_EQUAL_INT(0, twaiStub().sent);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_transmit_passes_a_zero_wait_to_the_driver);
    RUN_TEST(test_a_full_queue_drops_the_frame_without_wait_or_retry);
    RUN_TEST(test_a_standard_frame_carries_no_flag_bits);
    return UNITY_END();
}
