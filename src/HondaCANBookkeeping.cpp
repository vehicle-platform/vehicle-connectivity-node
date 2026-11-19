// HondaCANModule, part: stop-epoch / engine-start / warm-idle bookkeeping, stored values, published health and round-trip statistics (T23 split).
#include "HondaCANInternal.h"

uint8_t HondaCANModule::maxTimeoutLogLinesPerWindow() {
    return kMaxTimeoutLogLinesPerWindow;
}

void HondaCANModule::rearmStoppedReads() {
    for (uint8_t i = 0; i < VEHICLE_CL250_DID_COUNT; i++) {
        DidSlot& slot = _dids[i];
        if (slot.verified) {
            continue;
        }
        // D-069 item 3: the stop rule starts again from zero. lastRequestMs is kept, so the
        // DID is due one cadence after its last request (never above its tier rate, D-068
        // budget unchanged). The NRC counts of every unverified DID, stopped or not, are
        // cleared (rt-core#42 safety review round 3 MINOR-1): a count left from before the
        // engine start would stop it on one refusal.
        slot.stopNrcCount = 0;
        slot.sessionNrcCount = 0;
        slot.nrc31Seen = false;
        if (!slot.unsupported) {
            continue;
        }
        slot.unsupported = false;
        slot.stopReason = StopReason::NONE;
        slot.consecutiveTimeouts = 0;
    }
}

void HondaCANModule::watchEngineStart(float rpm) {
    if (rpm <= 0.0f) {
        _rpmZeroSeen = true;
        _rpmRunSamples = 0;
        return;
    }
    if (!_rpmZeroSeen || _engineStartRearmed) {
        return;
    }
    if (++_rpmRunSamples >= kEngineStartSamples) {
        _engineStartRearmed = true; // once per stop epoch, not on every stall/restart
        rearmStoppedReads();
    }
}

// D-075 (provisional) item 1: starts a stop epoch. Called only at the first session
// confirmation after the tester's start or after the ECU was lost, never for a
// re-confirmation after NRC 0x7E/0x7F.
void HondaCANModule::armUnverifiedReads(unsigned long now) {
    for (uint8_t i = 0; i < VEHICLE_CL250_DID_COUNT; i++) {
        DidSlot& slot = _dids[i];
        if (slot.verified) {
            continue; // the D-050..D-053 schedule of a verified DID is unchanged
        }
        slot.answered = false;
        slot.unsupported = false;
        slot.stopReason = StopReason::NONE;
        slot.stopNrcCount = 0;
        slot.sessionNrcCount = 0;
        slot.nrc31Seen = false;
        slot.consecutiveTimeouts = 0;
        slot.skipUntilMs = 0;
        slot.lastRequestMs = now - slot.cadenceMs; // due at once (unsigned, wrap-safe)
    }
    // D-069 item 3: a new stop epoch may re-arm once more at its first engine start
    // (_rpmZeroSeen and the sample run belong to the epoch too, D-075 item 1 (b)).
    _engineStartRearmed = false;
    _rpmZeroSeen = false;
    _rpmRunSamples = 0;
    // D-080 item 1: the key-on run of the epoch (also its "not served" and skip state are
    // forgotten: they hold until the next stop epoch). The warm-idle trigger starts over.
    _dtc.startEpoch();
    _warmIdleSinceMs = 0;
    for (uint8_t i = 0; i < VEHICLE_CL250_DID_COUNT; i++) {
        _didValueMs[i] = 0;
    }
}

bool HondaCANModule::warmIdleConditionsMet(unsigned long now) const {
    for (uint8_t k = 0; k < VEHICLE_CL250_DTC_WARM_IDLE_CONDITION_COUNT; k++) {
        const vehicle_cl250_dtc_condition_t& c = vehicle_cl250_dtc_warm_idle[k];
        if (c.did_index >= VEHICLE_CL250_DID_COUNT || _didValueMs[c.did_index] == 0 ||
            now - _didValueMs[c.did_index] > (unsigned long)vehicle_cl250_dids[c.did_index].stale_after_ms) {
            return false; // no value, or a stale one: not met
        }
        const float v = _didValue[c.did_index];
        bool met = false;
        switch (c.op) {
            case VEHICLE_CL250_DTC_OP_LT: met = v < c.value; break;
            case VEHICLE_CL250_DTC_OP_LE: met = v <= c.value; break;
            case VEHICLE_CL250_DTC_OP_GT: met = v > c.value; break;
            case VEHICLE_CL250_DTC_OP_GE: met = v >= c.value; break;
            case VEHICLE_CL250_DTC_OP_EQ: met = v == c.value; break; // NOLINT: exact, DIDs decode exact steps
            default: met = false; break;
        }
        if (!met) {
            return false;
        }
    }
    return true;
}

void HondaCANModule::watchWarmIdle(unsigned long now) {
    if (!warmIdleConditionsMet(now)) {
        _warmIdleSinceMs = 0;
        return;
    }
    if (_warmIdleSinceMs == 0) {
        _warmIdleSinceMs = (now == 0) ? 1 : now; // 0 means "not holding"
        return;
    }
    if (now - _warmIdleSinceMs >= (unsigned long)VEHICLE_CL250_DTC_WARM_IDLE_MIN_DURATION_MS) {
        _dtc.armWarmIdle(); // once per stop epoch (DtcSnapshot keeps the flag)
    }
}

void HondaCANModule::storeValue(SystemState& state, uint8_t didIndex, float value, unsigned long now) {
    switch (didIndex) {
        case VEHICLE_CL250_IDX_ENGINE_SPEED:
            state.engine.rpm = value;
            state.engine.rpmUpdatedMs = now;
            break;
        case VEHICLE_CL250_IDX_VEHICLE_SPEED:
            state.engine.speed = (uint8_t)value;
            state.engine.speedUpdatedMs = now;
            break;
        case VEHICLE_CL250_IDX_COOLANT_TEMP:
            state.engine.coolantTemp = (int16_t)value;
            state.engine.coolantTempUpdatedMs = now;
            break;
        case VEHICLE_CL250_IDX_THROTTLE_POS:
            state.engine.throttlePos = value;
            state.engine.throttlePosUpdatedMs = now;
            break;
        case VEHICLE_CL250_IDX_BATTERY_VOLTAGE:
            state.engine.batteryVoltage = value;
            state.engine.batteryVoltageUpdatedMs = now;
            break;
        default:
            break;
    }
}

// Read-only: mirrors bus/tester counters into SystemState for BLE telemetry v3. Never
// transmits and never changes the poller state; once latched off the driver is gone,
// so the counters stay frozen at their last values.
void HondaCANModule::publishHealth(SystemState& state) {
    state.tester = _tester.stats();
    state.tester.available = true;
    CanHealth& h = state.can;
    h.flags = CAN_HEALTH_FLAG_POLLER_ENABLED;
    if (_latchReason == TesterLatchReason::FOREIGN_TESTER || _latchReason == TesterLatchReason::UNKNOWN) {
        h.flags |= CAN_HEALTH_FLAG_LATCHED_FOREIGN_TESTER;
    }
    if (_latchReason == TesterLatchReason::BUS_OFF || _latchReason == TesterLatchReason::UNKNOWN) {
        h.flags |= CAN_HEALTH_FLAG_LATCHED_BUS_OFF; // both bits = latched, cause unknown
    }
    h.busOffCount = _busOffEventCount;
    h.unansweredDidCount = _unansweredDidCount;
    // D-066 item 4: which DIDs sit in their timeout cool-down (read-only mirror). Once
    // latched off nothing is read any more: no DID is skipped and no NRC is current, so the
    // values only age into STALE (safety-reviewer M1).
    const unsigned long now = millis();
    for (uint8_t i = 0; i < VEHICLE_CL250_DID_COUNT; i++) {
        VehicleSignalRecord& r = state.vehicleSignals.dids[i];
        // D-068: a DID stopped after timeouts shows as skipped; one stopped by NRC 0x31 keeps
        // showing that NRC (its last answer).
        r.skipped = !_latchedOff && (beforeDeadline(now, _dids[i].skipUntilMs) ||
                                     (_dids[i].unsupported && !r.lastWasNrc));
        if (_latchedOff) {
            r.lastWasNrc = false;
        }
    }
    if (_latchedOff) {
        h.busState = CanHealthState::STOPPED;
        return;
    }
    uint16_t txErr = 0, rxErr = 0;
    _bus.getErrorCounters(txErr, rxErr);
    h.txErrorCount = txErr;
    h.rxErrorCount = rxErr;
    switch (_bus.getState()) {
        case CanBusState::BUS_OFF: h.busState = CanHealthState::BUS_OFF; break;
        case CanBusState::STOPPED: h.busState = CanHealthState::STOPPED; break;
        case CanBusState::ERROR_WARNING: h.busState = CanHealthState::ERROR_WARNING; break;
        case CanBusState::RUNNING:
        default:
            h.busState = (txErr >= CAN_ERROR_WARNING_LIMIT || rxErr >= CAN_ERROR_WARNING_LIMIT)
                ? CanHealthState::ERROR_WARNING : CanHealthState::RUNNING;
            break;
    }
}

// D-058 item 1: the pending request was resolved at `now` by a positive response or an NRC
// other than 0x78. Time from its (first) send to this drain; none after a 0x78.
// None either for a request sent within kResponsePendingMaxMs after a timeout: the answer
// to the timed-out request may arrive late and resolve it early, which would record a
// too-short sample (a positive answer echoes its DID, so for the next request of the same
// DID; an NRC carries no DID, so for any DID read). An ECU answer later than the generated
// maximum wait is outside the ECU's own P2* limit and not covered.
void HondaCANModule::recordRoundTrip(unsigned long now) {
    if (_pendingSawNrc78 || _pendingRttSkip || _pendingDidIndex < 0) {
        return;
    }
    _tester.recordRoundTrip((uint8_t)_pendingDidIndex, (uint32_t)(now - _requestFirstSentMs));
}
