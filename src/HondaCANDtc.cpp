// HondaCANModule, part: D-080 DTC freeze-frame requests and answers (0x19) and the transmit hold (T23 split).
#include "HondaCANInternal.h"

// ---------------------------------------------------------------------------
// D-080 -- DTC freeze frames; segmented reception of a 0x19 (or D-059 scan) answer.
// ---------------------------------------------------------------------------

void HondaCANModule::holdTx(unsigned long now) {
    // yaml flow_control: after a First Frame that got no FC (or an aborted reception) no
    // request for response_timeout_max_ms; the ECU's remaining frames drain meanwhile.
    _txQuietUntilMs = now + kResponsePendingMaxMs;
    if (_txQuietUntilMs == 0) {
        _txQuietUntilMs = 1; // 0 means "none"
    }
}

// D-116 (amends D-059): a First Frame withheld an FC opens one quiet window per run, counted
// from the first one. Later ones before the window closes only count. The flag is cleared at
// the poll's send slot (update(), after the drain), not by a time comparison, so a request slot
// follows every run and a 2^32 ms wrap cannot alias it. Mirrors rt-core uds_client_core.
void HondaCANModule::holdTxForWithheldFf(unsigned long now) {
    if (_ffHoldRun) {
        if (_fcHoldNotRenewed < 0xFFFFFFFFu) {
            _fcHoldNotRenewed++;
        }
        return;
    }
    _ffHoldRun = true;
    holdTx(now);
}

void HondaCANModule::sendDtcRequest(unsigned long now) {
    uint8_t request[DtcSnapshot::kRequestMax];
    const uint8_t len = _dtc.buildRequest(request);
    if (len == 0) {
        return;
    }
    _pendingDtc = true;
    _pendingDidIndex = -1; // not a DID: no round-trip sample, no DID timeout count
    _pendingDid = 0;
    _pendingSid = UDS_SID_READ_DTC_INFORMATION;
    _fcSentForPending = false;
    _requestLostBaseline = _bus.rxLostCount(); // a frame lost after this may be part of the answer
    // A padded Single Frame like every request; sendFrame() passes it through the D-020 gate.
    // Round-2 MAJOR-A/B: the run's first request (the count) goes to both request IDs; the
    // first response ID that answers it locks the run, and the rest goes to its pair only,
    // so the other addressing never answers a segmented 0x19 in between.
    if (_dtc.step() == DtcSnapshot::Step::COUNT) {
        _dtcLocked = false;
    }
    if (_dtcLocked) {
        sendFrame(_dtcRequestId, request, len);
    } else {
        sendRequest(request, len);
    }
    _udsState = UdsRequestState::WAITING;
    _requestSentMs = now;
    _requestFirstSentMs = now;
    _pendingSawNrc78 = false;
    _pendingRttSkip = true;
    _responseTimeoutMs = kResponseTimeoutMs; // D-080 item 4: never extended by NRC 0x78
}

bool HondaCANModule::onDtcSingleFrame(const CanFrame& f, unsigned long now) {
    if (!_pendingDtc || _udsState != UdsRequestState::WAITING) {
        return false;
    }
    const uint8_t len = (uint8_t)(f.data[0] & 0x0F);
    if (len == 0 || len > isotp::kSingleFrameMaxLen || len + 1u > f.dlc) {
        return false;
    }
    const uint8_t* msg = &f.data[1];
    const bool positive = _dtc.matchesPositive(msg, len);
    if (!positive && !_dtc.matchesNegative(msg, len)) {
        return false;
    }
    if (!positive && !_dtcLocked && msg[2] == UDS_NRC_RESPONSE_PENDING) {
        // E-21 MINOR-1: NRC 0x78 is not an answer. It must not lock the run (a fallback-ID
        // node's 7F 19 78 would otherwise shut out the real ECU's count); the run stays
        // unlocked and waits for the real answer within the base timeout (no extension).
        return true;
    }
    if (!acceptDtcAnswerFrom(f)) {
        return true; // the other addressing (or after a First Frame of ours): ignored
    }
    if (positive) {
        checkDtcSlot(now);
        _dtc.onPositive(msg, len, f.id);
        _lastGoodResponseMs = now;
        _udsState = UdsRequestState::COMPLETE;
        return true;
    }
    _nrcCount++;
    _lastGoodResponseMs = now; // G2.3: an NRC still proves the ECU is alive and answering
    switch (_dtc.onNrc(msg[2])) {
        case DtcSnapshot::NrcAction::WAIT:
            return true; // 0x78: no extension (D-080 item 4), it times out at the base timeout
        case DtcSnapshot::NrcAction::SESSION_LOST:
            // D-080 item 2 / D-075 item 1: the ECU fell back to the default session; ask for
            // it again at the next step. Its confirmation stays in the same stop epoch.
            if (_sessionConfirmed) {
                _sessionConfirmed = false;
                Serial.println("[UDS] Extended session lost (NRC 0x7E/0x7F to 0x19) -- requesting it again.");
            }
            _sessionRetryNow = true;
            break;
        default:
            break;
    }
    _udsState = UdsRequestState::COMPLETE;
    return true;
}

bool HondaCANModule::acceptDtcAnswerFrom(const CanFrame& f) {
    uint32_t requestId = 0;
    if (!pairedRequestId(f, requestId)) {
        return false;
    }
    if (_fcPending || _isoRx.active()) {
        // Our First Frame is being answered: a Single Frame now (from either ID) must not
        // resolve the 0x19 under the reception (round-2 MAJOR-B). If it is the other ID,
        // 0x19 is off for the epoch; the reception finishes and is checked and logged.
        if (!(_fcPending ? (f.id == _fcPendingFrame.id && f.extended == _fcPendingFrame.extended)
                         : _isoRx.fromReceptionId(f))) {
            _dtc.haltAfterAnswer(DtcSnapshot::Halt::BOTH_IDS);
        }
        return false;
    }
    if (_dtcLocked) {
        return f.id == _dtcResponseId && f.extended == _dtcResponseExtended;
    }
    lockDtcTo(f, requestId);
    return true;
}

void HondaCANModule::lockDtcTo(const CanFrame& f, uint32_t requestId) {
    _dtcLocked = true;
    _dtcResponseId = f.id;
    _dtcResponseExtended = f.extended;
    _dtcRequestId = requestId;
}

void HondaCANModule::checkDtcSlot(unsigned long now) {
    if (_pendingDtc && now - _requestFirstSentMs > kDtcSlotMaxMs) {
        _dtc.haltAfterAnswer(DtcSnapshot::Halt::SLOW);
    }
}
