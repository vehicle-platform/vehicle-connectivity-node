#ifndef TWAI_CAN_BUS_H
#define TWAI_CAN_BUS_H

// The listen-only capture build (D-058 item 4, CONN_CAN_LISTEN_ONLY) must not contain
// code that can transmit: nothing of this file is compiled there.
#if !defined(CONN_CAN_LISTEN_ONLY) || !CONN_CAN_LISTEN_ONLY

#include "ICanBus.h"
#include "driver/twai.h"

/**
 * @brief G3.2 -- Real ESP32-S3 TWAI-backed ICanBus implementation.
 * Wraps the twai_* calls HondaCANModule used to call directly (500 kbps, accept-all
 * filter, bus-off/error-warning alerts). Q-018: the driver starts in listen-only mode
 * and is reinstalled in normal mode only when HondaCANModule's observation window ends;
 * stop() leaves TX held recessive.
 */
class TwaiCanBus : public ICanBus {
private:
    gpio_num_t _txPin;
    gpio_num_t _rxPin;
    bool _installed = false;

    bool install(twai_mode_t mode);
    void uninstall();

public:
    // D-096: the tester's transmit never waits. A frame the full TWAI TX queue refuses is
    // dropped (transmit() returns false) and never retried inside the pass. Requests ignore
    // the result, so a refused one reads as an unanswered request (D-068 timeout path); a
    // refused FC.CTS aborts the reception (HondaCANModule failSegmented, no second FC).
    // With a wait, an unacknowledged bus (ECU off) let one pass block 4 x 5 ms against
    // client_step_max_ms (ISSUES E-25 (4)). Frames already accepted into the queue (up to
    // tx_queue_len) stay there and go out once the bus acknowledges again; they passed the
    // gate when queued (a HIL check, ISSUES E-25).
    static constexpr uint32_t kTxWaitMs = 0;

    TwaiCanBus(gpio_num_t txPin, gpio_num_t rxPin);

    bool beginListenOnly() override;
    bool enterNormalMode() override;
    bool transmit(const CanFrame& frame) override;
    bool receive(CanFrame& frame) override;
    CanBusState getState() override;
    uint32_t rxLostCount() override;
    void getErrorCounters(uint16_t& txErrorCount, uint16_t& rxErrorCount) override;
    bool initiateRecovery() override;
    bool start() override;
    void stop() override;
};

#endif // !CONN_CAN_LISTEN_ONLY

#endif // TWAI_CAN_BUS_H
