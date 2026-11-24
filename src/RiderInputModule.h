#ifndef RIDER_INPUT_MODULE_H
#define RIDER_INPUT_MODULE_H

#include "IModule.h"
#include "RiderInputCore.h"

// Handlebar push-to-talk button (D-047 item 1, D-121 item 5), bike ESP32. The GPIO is a
// build constant: -D RIDER_PTT_GPIO=<n> (active-low button to GND, internal pull-up). It
// defaults to -1 (disabled) because the pin is not chosen yet (hardware Q-034 / pin plan);
// disabled, the `riderInput` characteristic still exists but nothing is sampled or sent.
#ifndef RIDER_PTT_GPIO
#define RIDER_PTT_GPIO (-1)
#endif
#if RIDER_PTT_GPIO > 48
#error "RIDER_PTT_GPIO must be -1 (disabled) or an ESP32-S3 GPIO number 0..48."
#endif

// Fixed sampling period (a third of RIDER_PTT_DEBOUNCE_MS at most).
constexpr uint32_t RIDER_PTT_SAMPLE_PERIOD_MS = 5u;
static_assert(RIDER_PTT_SAMPLE_PERIOD_MS * 3u <= RIDER_PTT_DEBOUNCE_MS, "sample often enough to debounce");

/**
 * @brief GPIO adapter for RiderInputCore: samples the pin every RIDER_PTT_SAMPLE_PERIOD_MS from
 * the main loop and queues events in the core, which BLEServerModule drains and notifies.
 * Informational only: it reads one input pin, writes nothing to SystemState and has no link to
 * the TWAI/UDS tester code (invariant 1).
 */
class RiderInputModule : public IConsumerModule {
public:
    RiderInputModule() : _core(RIDER_PTT_GPIO) {}

    bool begin() override;
    void update(const SystemState& state) override;
    bool isHealthy() const override { return true; } // disabled is not a fault
    uint32_t getPeriodMs() const override { return RIDER_PTT_SAMPLE_PERIOD_MS; }

    RiderInputCore& core() { return _core; }

private:
    RiderInputCore _core;
};

#endif // RIDER_INPUT_MODULE_H
