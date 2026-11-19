#include "HondaCANInternal.h"

HondaCANModule::HondaCANModule(ICanBus& bus, ITesterLatchStore& latchStore)
    : _bus(bus), _latchStore(latchStore), _dtc(reportSerialLine)
#if CONN_DISCOVERY_PROBE
    , _scan(reportSerialLine)
#endif
{
    _timeoutLogBudget = kMaxTimeoutLogLinesPerWindow; // the first window is open from boot
    for (uint8_t i = 0; i < VEHICLE_CL250_DID_COUNT; i++) {
        _dids[i].did = vehicle_cl250_dids[i].did;
        _dids[i].cadenceMs = vehicle_cl250_dids[i].poll_period_ms;
        _dids[i].lastRequestMs = 0;
        _dids[i].consecutiveTimeouts = 0;
        _dids[i].skipUntilMs = 0;
        _dids[i].verified = vehicle_cl250_dids[i].verified;
        _dids[i].answered = false;
        _dids[i].unsupported = false;
        _dids[i].stopReason = StopReason::NONE;
        _dids[i].stopNrcCount = 0;
        _dids[i].sessionNrcCount = 0;
        _dids[i].nrc31Seen = false;
    }
}

bool HondaCANModule::begin() {
    // Q-018: a latch from before the last reset stays in force (an invalid record after a
    // non-power-on reset counts as one), and so does the bus-off budget. A latched node
    // never installs the driver: it neither transmits nor acknowledges on the vehicle bus.
    TesterLatchStatus restored = _latchStore.load();
    _busOffEventCount = restored.busOffCount;
    if (restored.reason == TesterLatchReason::NONE && _busOffEventCount >= kMaxBusOffEvents) {
        restored.reason = TesterLatchReason::BUS_OFF; // defensive: the budget is already spent
        _latchStore.save(restored.reason, _busOffEventCount);
    }
    if (restored.reason != TesterLatchReason::NONE) {
        _latchedOff = true;
        _latchReason = restored.reason;
        _bus.stop();
        Serial.printf("[UDS STOP] Vehicle-bus poller stays latched off from before the last reset (%s). Power-cycle the node to re-enable it.\n",
            latchReasonText(_latchReason));
        _initialized = true; // healthy: update() keeps publishing the latch in CAN health
        return true;
    }

    if (_bus.beginListenOnly()) {
        // Nothing is sent yet: the window starts at the first update() (listenStep()).
        _listening = true;
        Serial.printf("[CAN SUCCESS] CAN Bus Active (500 kbps), listen-only for %lu ms before the first request (bus-off events so far: %lu)...\n",
            kListenOnlyWindowMs, (unsigned long)_busOffEventCount);
        _initialized = true;
        return true;
    }
    _bus.stop();
    Serial.println("[CAN ERROR] Failed to initialize CAN bus driver! Check TX/RX pins.");
    _initialized = false;
    return false;
}

bool HondaCANModule::sendFrame(uint32_t id, const uint8_t* request, uint8_t len) {
    CanFrame frame;
    frame.extended = isotp::isExtendedId(id);
    frame.id = id;
    frame.dlc = VEHICLE_CL250_FRAME_DLC;
    frame.data[0] = len; // ISO-TP single-frame PCI
    for (uint8_t i = 1; i < VEHICLE_CL250_FRAME_DLC; i++) {
        frame.data[i] = (i - 1 < len) ? request[i - 1] : VEHICLE_CL250_PADDING_BYTE;
    }

    // D-021: only the ECU's own request IDs; D-020: only allow-listed services, Single
    // Frames only. The generated guard is the last word on what may reach the vehicle bus.
    bool idAllowed = (id == VEHICLE_CL250_REQUEST_ID) || (id == VEHICLE_CL250_FALLBACK_REQUEST_ID);
    if (!idAllowed || len == 0 || len > isotp::kSingleFrameMaxLen ||
        !vehicle_cl250_frame_allowed(frame.data, frame.dlc)) {
        _blockedFrames++;
        Serial.printf("[UDS BLOCKED] Frame 0x%08lX [%02X %02X %02X] refused by the D-020 guard (total=%lu).\n",
            (unsigned long)frame.id, frame.data[0], frame.data[1], frame.data[2], (unsigned long)_blockedFrames);
        return false;
    }
    return _bus.transmit(frame);
}

void HondaCANModule::sendRequest(const uint8_t* request, uint8_t len) {
    sendFrame(VEHICLE_CL250_REQUEST_ID, request, len);
    sendFrame(VEHICLE_CL250_FALLBACK_REQUEST_ID, request, len);
}

void HondaCANModule::requestDID(uint16_t did) {
    const uint8_t request[] = {UDS_SID_READ_DATA_BY_IDENTIFIER, (uint8_t)((did >> 8) & 0xFF), (uint8_t)(did & 0xFF)};
    sendRequest(request, sizeof(request));
}

void HondaCANModule::update(SystemState& state) {
    // D-058 item 2: every entry counts, also in the listen window and when latched (the step
    // still runs). micros() wraps at 2^32 us; TesterStatsTracker subtracts wrap-safely.
    _tester.onStepEntry((uint32_t)micros());
    poll(state);
    publishHealth(state);
}

void HondaCANModule::poll(SystemState& state) {
    unsigned long now = millis();

    // Latched off (foreign tester seen or too many bus-offs): transmit nothing ever
    // again until power-on, and stop feeding SystemState so consumers show "stale".
    if (_latchedOff) {
        state.engine.ecuPresent = false;
        return;
    }

    if (_listening) {
        listenStep(state, now);
        return;
    }

    // Q-018: drain RX first -- before anything below transmits (a foreign tester in the
    // queue latches us off first) and before the response-timeout check (a response that
    // is already queued resolves its request instead of counting as a timeout).
    DrainResult rx = drainRx(state, now);
    if (rx == DrainResult::LATCHED) {
        return;
    }
    if (rx == DrainResult::BACKLOG) {
        // More queued than one pass handles: a foreign request may still be in there, so
        // transmit nothing this pass (and count no timeout).
        return;
    }
    // D-059/D-080: while a segmented answer is received nothing else is sent and no response
    // timeout runs. Lost frames and N_Cr are checked only now, after the drain, against
    // the drain time of the last CF, so CFs already queued are never "late".
    if (_isoRx.active()) {
        if (!_isoRx.checkAfterDrain(now, _bus.rxLostCount())) {
            return;
        }
        endReception(now);
    }
    // The FC of an accepted First Frame goes out only here, after the whole queue was
    // drained without a foreign tester (Q-018), never in the middle of a drain.
    if (_fcPending) {
        sendPendingFlowControl(now);
        if (_isoRx.active()) {
            return; // the reception started: nothing else this pass
        }
    }
    // D-080: at most one line of a 0x19 answer's hex dump per pass (Serial is slow).
    _dtc.flushOne();
    watchWarmIdle(now);

    if (_sessionStartPending) {
        // First pass in normal mode, after its drain: start the extended diagnostic
        // session. G2.3: not assumed to succeed -- retried below until 0x50 confirms it.
        _sessionStartPending = false;
        Serial.println("[CAN] Starting the diagnostic session.");
        sendRequest(kSessionRequest, sizeof(kSessionRequest));
        _lastSessionAttemptMs = now;
        _lastKeepAlive = now;
#if CONN_DISCOVERY_PROBE
        _firstSessionRequestMs = now;
#endif
        for (uint8_t i = 0; i < VEHICLE_CL250_DID_COUNT; i++) {
            _dids[i].lastRequestMs = now; // the DID schedule starts together with the session
        }
        return;
    }

    // 1. Bus-Off Auto Recovery Check (G1.3 -- exponential backoff, event logging)
    CanBusState busState = _bus.getState();
    if (busState == CanBusState::BUS_OFF) {
        if (!_busOff) {
            // Just entered bus-off: log once, reset backoff, attempt immediately.
            _busOff = true;
            _busOffEventCount++;
            if (_busOffEventCount >= kMaxBusOffEvents) {
                latchOff(TesterLatchReason::BUS_OFF, "too many bus-off events");
                state.engine.ecuPresent = false;
                return;
            }
            // Q-018: the budget counts from power-on, not from the last reset.
            _latchStore.save(TesterLatchReason::NONE, _busOffEventCount);
            _recoveryBackoffMs = kBackoffMinMs;
            _lastRecoveryAttempt = 0;
            uint16_t txErr = 0, rxErr = 0;
            _bus.getErrorCounters(txErr, rxErr);
            Serial.printf("[CAN WARNING] Bus-Off detected (event #%lu, tx_err=%u, rx_err=%u). Starting recovery...\n",
                (unsigned long)_busOffEventCount, txErr, rxErr);
        }

        if (now - _lastRecoveryAttempt >= _recoveryBackoffMs) {
            _lastRecoveryAttempt = now;
            Serial.printf("[CAN WARNING] Bus-Off recovery attempt (next retry in %lu ms if this fails)...\n",
                _recoveryBackoffMs);
            _bus.initiateRecovery();
            _recoveryBackoffMs = min(_recoveryBackoffMs * 2, kBackoffMaxMs);
        }
    } else if (busState == CanBusState::STOPPED) {
        // initiateRecovery() lands the driver here once recovery completes.
        if (_busOff) {
            Serial.println("[CAN SUCCESS] Bus-Off recovery complete, restarting driver.");
            _busOff = false;
            _recoveryBackoffMs = kBackoffMinMs;
        }
        _bus.start();
    } else if (busState == CanBusState::RUNNING && _busOff) {
        _busOff = false;
        _recoveryBackoffMs = kBackoffMinMs;
    }

    // txHold = the quiet time after a First Frame that got no FC or an aborted reception,
    // in which no request at all is sent (yaml flow_control rule). D-059 probe only (true in
    // every other build): during the scan, tester present and the session retry go out only
    // between requests, never while one is in flight.
    const bool txHold = beforeDeadline(now, _txQuietUntilMs);
    if (!txHold) {
        _ffHoldRun = false; // D-116: the run of withheld First Frames ends at this send slot
    }
    bool auxAllowed = true;
#if CONN_DISCOVERY_PROBE
    if (_scanPhase != ScanPhase::DONE) {
        auxAllowed = (_udsState == UdsRequestState::IDLE);
    }
#endif
    // D-080 (safety-reviewer MINOR-2): no tester present or session retry while a 0x19 is in
    // flight, so no other answer lands in its (segmented) reception. At most one slot plus a
    // reception (<= RESPONSE_TIMEOUT_MAX_MS), well inside S3 with the 1 s tester present.
    if (_pendingDtc && _udsState == UdsRequestState::WAITING) {
        auxAllowed = false;
    }
    bool auxSent = false; // tester present or session retry sent this pass

    // 2. Tester present -- only meaningful once a session is confirmed, but harmless to
    // send regardless (an ECU in default session simply ignores/NAKs it).
    if (!txHold && auxAllowed && now - _lastKeepAlive >= kTesterPresentPeriodMs) {
        _lastKeepAlive = now;
        sendRequest(kTesterPresentRequest, sizeof(kTesterPresentRequest));
        auxSent = true;
    }

    // 2b. G2.3 -- Retry the extended diagnostic session until a positive 0x50 confirms it.
    if (!txHold && auxAllowed && !_sessionConfirmed &&
        (_sessionRetryNow || now - _lastSessionAttemptMs >= kSessionRetryMs)) {
        _lastSessionAttemptMs = now;
        _sessionRetryNow = false;
        Serial.println("[UDS] Extended session (0x10 0x03) not yet confirmed -- retrying...");
        sendRequest(kSessionRequest, sizeof(kSessionRequest));
        auxSent = true;
    }
    (void)auxSent; // read by the 0x19 step below and the probe's scan step

    // 2c. G2.3 -- Derive ECU presence from recency of any UDS response and publish it
    // for consumers to show an explicit "ECU not found" state.
    bool ecuPresentNow = (_lastGoodResponseMs != 0) && (now - _lastGoodResponseMs < kEcuAbsentTimeoutMs);
    if (ecuPresentNow != _ecuPresent) {
        if (!ecuPresentNow) {
            _sessionConfirmed = false; // ECU lost (e.g. reset): request the session again
            _newStopEpoch = true;      // D-075 item 1: its confirmation starts a new stop epoch
        }
        _ecuPresent = ecuPresentNow;
        Serial.printf("[UDS] ECU is now %s.\n", _ecuPresent ? "PRESENT" : "NOT DETECTED");
    }
    state.engine.ecuPresent = _ecuPresent;

    // 3. G2.2 -- UDS single-request state machine: IDLE -> REQUEST_SENT -> WAITING -> COMPLETE/TIMEOUT
    if (_udsState == UdsRequestState::IDLE && !txHold) {
#if CONN_DISCOVERY_PROBE
        // D-059: the scan runs first, once; DID polling starts when it is done. After a
        // tester present or session retry, the next scan request waits for the base
        // response timeout, so the aux request is answered (or not) before it (one request
        // in flight, safety-reviewer MAJOR-2).
        if (_scanPhase != ScanPhase::DONE) {
            if (auxSent) {
                unsigned long hold = now + kResponseTimeoutMs;
                if (!beforeDeadline(hold, _scanHoldUntilMs)) {
                    _scanHoldUntilMs = hold; // never shortens a longer hold
                }
            } else {
                scanStep(now);
            }
        } else
#endif
        {
        bool didSent = false;
        for (uint8_t i = 0; i < VEHICLE_CL250_DID_COUNT; i++) {
            DidSlot& slot = _dids[i];
            if (beforeDeadline(now, slot.skipUntilMs)) {
                continue; // temporarily skipped after repeated timeouts
            }
            if (slot.unsupported || (!slot.verified && !_sessionConfirmed)) {
                // D-068 item 2: an unverified DID is asked only in a confirmed session (an
                // ECU that serves it only there would refuse it before), and not at all
                // once stopped.
                continue;
            }
            if (now - slot.lastRequestMs >= slot.cadenceMs) {
                slot.lastRequestMs = now;
                _pendingDidIndex = i;
                _pendingDid = slot.did;
                _pendingSid = UDS_SID_READ_DATA_BY_IDENTIFIER;
                _udsState = UdsRequestState::REQUEST_SENT;

                requestDID(slot.did);

                // Transmission above is synchronous, so the request is immediately
                // outstanding -- move straight into WAITING for its response.
                _udsState = UdsRequestState::WAITING;
                _requestSentMs = now;
                _requestFirstSentMs = now;
                _pendingSawNrc78 = false;
                _pendingRttSkip = beforeDeadline(now, _rttQuietUntilMs);
                _responseTimeoutMs = kResponseTimeoutMs;
                didSent = true;
                break; // exactly one request in flight at a time
            }
        }
        // D-080 item 4: a 0x19 request only in a step where no DID was due (after every
        // verified and unverified DID), and only in a confirmed session, like an unverified read.
        // Not in a pass that sent tester present or the session retry: its answer could land in
        // the 0x19 reception (round-2 MINOR-3).
        if (!didSent && !auxSent && _sessionConfirmed && _dtc.due()) {
            sendDtcRequest(now);
        }
        }
    } else if (_udsState == UdsRequestState::WAITING) {
        if (now - _requestSentMs > _responseTimeoutMs) {
            _udsState = UdsRequestState::TIMEOUT;
        }
    }

#if CONN_DISCOVERY_PROBE
    if (_udsState == UdsRequestState::TIMEOUT && _pendingScanIndex >= 0) {
        // A scan entry is not a DID: no DID timeout count. Its answer may still arrive late
        // and be taken for the next entry's, so the next scan request waits.
        _scan.recordNoAnswer((uint8_t)_pendingScanIndex);
        _pendingScanIndex = -1;
        _scanHoldUntilMs = now + kResponsePendingMaxMs;
        _udsState = UdsRequestState::IDLE;
    }
#endif
    if (_udsState == UdsRequestState::TIMEOUT && _pendingDtc) {
        // Not a DID: no DID timeout count. A late answer no longer matches (the run ended).
        _pendingDtc = false;
        _dtc.onTimeout();
        _udsState = UdsRequestState::IDLE;
    }
    bool timeoutThisPass = false;
    if (_udsState == UdsRequestState::TIMEOUT) {
        DidSlot& slot = _dids[_pendingDidIndex];
        slot.consecutiveTimeouts++;
        _unansweredDidCount++;
        // D-066 item 4: silence is now this DID's latest answer, so an earlier NRC is no
        // longer shown (safety-reviewer M1); the record falls back to its value's age.
        state.vehicleSignals.dids[_pendingDidIndex].lastWasNrc = false;
        // D-058: the timed-out request's answer may still come and resolve a later request
        // early (see recordRoundTrip()); statistics only, the schedule is unchanged.
        _rttQuietUntilMs = now + kResponsePendingMaxMs;
        // Log only (the counters and the stop rule here do not depend on it): a bounded
        // number of lines per window. What was left out is folded into the first line of
        // the next window, so a timeout never prints more than this one line.
        timeoutThisPass = true;
        uint16_t foldedSuppressed = 0;
        if (now - _timeoutLogWindowStartMs >= kTimeoutLogWindowMs) {
            foldedSuppressed = _timeoutLogsSuppressed;
            _timeoutLogWindowStartMs = now;
            _timeoutLogBudget = kMaxTimeoutLogLinesPerWindow;
            _timeoutLogsSuppressed = 0;
        }
        if (takeLogSlot(_timeoutLogBudget)) {
            if (foldedSuppressed > 0) {
                Serial.printf("[UDS WARNING] Timeout waiting for DID 0x%04X (consecutive=%u/%u) (+%u suppressed in the last window)\n",
                    _pendingDid, slot.consecutiveTimeouts, kMaxConsecutiveTimeouts, (unsigned)foldedSuppressed);
            } else {
                Serial.printf("[UDS WARNING] Timeout waiting for DID 0x%04X (consecutive=%u/%u)\n",
                    _pendingDid, slot.consecutiveTimeouts, kMaxConsecutiveTimeouts);
            }
        } else if (_timeoutLogsSuppressed < 0xFFFFu) {
            _timeoutLogsSuppressed++; // saturating, never wraps
        }

        if (_pendingDidIndex == VEHICLE_CL250_IDX_ENGINE_SPEED) {
            _rpmRunSamples = 0; // D-069 item 3: the RPM samples must be consecutive
        }
        if (slot.consecutiveTimeouts >= kMaxConsecutiveTimeouts) {
            slot.consecutiveTimeouts = 0;
            if (!slot.verified && !slot.answered) {
                // D-068 item 2: never answered in this stop epoch: probably not served at all.
                slot.unsupported = true;
                slot.stopReason = StopReason::TIMEOUTS;
                Serial.printf("[UDS] DID 0x%04X never answered -- not asked again until the next stop epoch (D-068, D-075).\n",
                    _pendingDid);
            } else {
                slot.skipUntilMs = now + kDidSkipCooldownMs;
                Serial.printf("[UDS WARNING] DID 0x%04X unresponsive -- skipping requests for %lu ms.\n",
                    _pendingDid, kDidSkipCooldownMs);
            }
        }
        _udsState = UdsRequestState::IDLE;
    }

    if (_udsState == UdsRequestState::COMPLETE) {
        if (_pendingDidIndex >= 0) {
            _dids[_pendingDidIndex].consecutiveTimeouts = 0;
        }
        _pendingDtc = false;
#if CONN_DISCOVERY_PROBE
        _pendingScanIndex = -1;
#endif
        _udsState = UdsRequestState::IDLE;
    }

    // Log only: when timeouts stop, the last window's suppressed count is reported once on a
    // later pass (never in a pass that already printed a timeout line).
    if (!timeoutThisPass && _timeoutLogsSuppressed > 0 &&
        now - _timeoutLogWindowStartMs >= kTimeoutLogWindowMs) {
        Serial.printf("[UDS WARNING] %u DID timeout warnings suppressed in the last window\n",
            (unsigned)_timeoutLogsSuppressed);
        _timeoutLogsSuppressed = 0;
    }
}

void HondaCANModule::listenStep(SystemState& state, unsigned long now) {
    state.engine.ecuPresent = false;
    if (!_listenStarted) {
        // The window starts here, not in begin(): setup() still runs the other modules'
        // begin() in between, and the queue may overflow meanwhile (caught below).
        _listenStarted = true;
        _listenStartMs = now;
    }
    DrainResult rx = drainRx(state, now);
    if (rx == DrainResult::LATCHED) {
        return; // latched off before this node ever transmitted
    }
    uint32_t lost = _bus.rxLostCount();
    if (rx == DrainResult::BACKLOG || lost != _rxLostBaseline) {
        // Frames went unseen (queue full) or are still queued: this window proves nothing.
        _rxLostBaseline = lost;
        _listenStartMs = now;
        if (_listenRestarts++ == 0) {
            Serial.println("[CAN] Listen-only window restarted: frames were lost or still queued.");
        }
        return;
    }
    if (now - _listenStartMs < kListenOnlyWindowMs) {
        return;
    }
    _listening = false;
    if (!_bus.enterNormalMode()) {
        Serial.println("[CAN ERROR] Could not switch the CAN driver to normal mode -- poller disabled.");
        _bus.stop();
        _initialized = false;
        return;
    }
    Serial.printf("[CAN] No other tester seen for %lu ms (window restarts: %lu). Normal mode.\n",
        kListenOnlyWindowMs, (unsigned long)_listenRestarts);
    // The session request goes out on the next pass, after its RX drain, so frames that
    // arrived while the driver was reinstalled are checked first.
    _sessionStartPending = true;
}

void HondaCANModule::latchOff(TesterLatchReason reason, const char* why) {
    if (_latchedOff) {
        return;
    }
    _latchedOff = true;
    _latchReason = reason;
    _latchStore.save(reason, _busOffEventCount); // Q-018: first, so it holds even if a reset follows
    _bus.stop(); // no more transmission, no ACKs, no error frames from this node
    Serial.printf("[UDS STOP] Vehicle-bus poller latched off until power-on: %s.\n", why);
}
