#include "RiderInputModule.h"

#include <Arduino.h>

bool RiderInputModule::begin() {
#if RIDER_PTT_GPIO >= 0
    pinMode(RIDER_PTT_GPIO, INPUT_PULLUP); // button to GND: pressed = low
    Serial.printf("[PTT] push-to-talk button on GPIO %d\n", (int)RIDER_PTT_GPIO);
#else
    Serial.println("[PTT] push-to-talk disabled (RIDER_PTT_GPIO = -1, pin not chosen yet)");
#endif
    return true;
}

void RiderInputModule::update(const SystemState& state) {
    (void)state; // informational input: no SystemState involved
#if RIDER_PTT_GPIO >= 0
    _core.sample(digitalRead(RIDER_PTT_GPIO) == LOW, (uint32_t)millis());
#endif
}
