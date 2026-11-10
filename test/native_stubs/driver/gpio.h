#ifndef NATIVE_TEST_DRIVER_GPIO_STUB_H
#define NATIVE_TEST_DRIVER_GPIO_STUB_H

// D-096 -- minimal ESP-IDF GPIO stand-in for test/test_twai_can_bus (TwaiCanBus::stop()).

#include <stdint.h>

typedef enum { GPIO_NUM_4 = 4, GPIO_NUM_5 = 5 } gpio_num_t;
typedef enum { GPIO_MODE_INPUT = 1, GPIO_MODE_OUTPUT = 2 } gpio_mode_t;
typedef struct {
    uint64_t pin_bit_mask;
    gpio_mode_t mode;
} gpio_config_t;

inline int gpio_set_level(gpio_num_t, uint32_t) { return 0; }
inline int gpio_config(const gpio_config_t*) { return 0; }

#endif // NATIVE_TEST_DRIVER_GPIO_STUB_H
