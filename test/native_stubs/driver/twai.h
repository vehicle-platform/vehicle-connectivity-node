#ifndef NATIVE_TEST_DRIVER_TWAI_STUB_H
#define NATIVE_TEST_DRIVER_TWAI_STUB_H

// D-096 -- minimal ESP-IDF 4.4 TWAI driver stand-in so src/hal/TwaiCanBus.cpp compiles and
// runs on the host (test/test_twai_can_bus). It records what the code passes to the driver
// and lets a test refuse transmits like a full TX queue. Only on the native include path.

#include <stdint.h>
#include <string.h>

#include "driver/gpio.h"

typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_TIMEOUT 0x107

typedef uint32_t TickType_t;
#define portTICK_PERIOD_MS 1u
#define pdMS_TO_TICKS(ms) ((TickType_t)((ms) / portTICK_PERIOD_MS))

typedef enum { TWAI_MODE_NORMAL, TWAI_MODE_NO_ACK, TWAI_MODE_LISTEN_ONLY } twai_mode_t;
typedef enum {
    TWAI_STATE_STOPPED,
    TWAI_STATE_RUNNING,
    TWAI_STATE_BUS_OFF,
    TWAI_STATE_RECOVERING
} twai_state_t;

#define TWAI_ALERT_ERR_PASS 0x1u
#define TWAI_ALERT_BUS_RECOVERED 0x2u
#define TWAI_ALERT_ABOVE_ERR_WARN 0x4u
#define TWAI_ALERT_BUS_OFF 0x8u

// Same layout as ESP-IDF 4.4.7 hal/twai_types.h: the flag bits share a union with `flags`.
typedef struct {
    union {
        struct {
            uint32_t extd : 1;
            uint32_t rtr : 1;
            uint32_t ss : 1;
            uint32_t self : 1;
            uint32_t dlc_non_comp : 1;
            uint32_t reserved : 27;
        };
        uint32_t flags;
    };
    uint32_t identifier;
    uint8_t data_length_code;
    uint8_t data[8];
} twai_message_t;

typedef struct { uint32_t brp; } twai_timing_config_t;
typedef struct {
    twai_mode_t mode;
    gpio_num_t tx_io;
    gpio_num_t rx_io;
    uint32_t tx_queue_len;
    uint32_t rx_queue_len;
    uint32_t alerts_enabled;
} twai_general_config_t;
typedef struct { uint32_t acceptance_code; } twai_filter_config_t;
typedef struct {
    twai_state_t state;
    uint32_t tx_error_counter;
    uint32_t rx_error_counter;
    uint32_t rx_missed_count;
    uint32_t rx_overrun_count;
} twai_status_info_t;

#define TWAI_TIMING_CONFIG_500KBITS() {8u}
#define TWAI_GENERAL_CONFIG_DEFAULT(tx, rx, op_mode) {(op_mode), (tx), (rx), 5u, 5u, 0u}
#define TWAI_FILTER_CONFIG_ACCEPT_ALL() {0u}

// What the code under test did, and how the fake driver answers.
struct TwaiStub {
    int transmitCalls = 0;
    TickType_t lastTransmitWait = 0xFFFFFFFFu;
    bool txQueueFull = false; // twai_transmit refuses at once (with a 0 wait) like a full queue
    int sent = 0;
    twai_message_t lastSent{};
};
inline TwaiStub& twaiStub() {
    static TwaiStub stub;
    return stub;
}

inline esp_err_t twai_driver_install(const twai_general_config_t*, const twai_timing_config_t*,
                                     const twai_filter_config_t*) {
    return ESP_OK;
}
inline esp_err_t twai_driver_uninstall(void) { return ESP_OK; }
inline esp_err_t twai_start(void) { return ESP_OK; }
inline esp_err_t twai_stop(void) { return ESP_OK; }
inline esp_err_t twai_initiate_recovery(void) { return ESP_OK; }
inline esp_err_t twai_transmit(const twai_message_t* message, TickType_t ticks_to_wait) {
    TwaiStub& s = twaiStub();
    s.transmitCalls++;
    s.lastTransmitWait = ticks_to_wait;
    if (s.txQueueFull) {
        return ESP_ERR_TIMEOUT;
    }
    s.sent++;
    s.lastSent = *message;
    return ESP_OK;
}
inline esp_err_t twai_receive(twai_message_t*, TickType_t) { return ESP_ERR_TIMEOUT; }
inline esp_err_t twai_get_status_info(twai_status_info_t* status) {
    memset(status, 0, sizeof *status);
    status->state = TWAI_STATE_RUNNING;
    return ESP_OK;
}

#endif // NATIVE_TEST_DRIVER_TWAI_STUB_H
