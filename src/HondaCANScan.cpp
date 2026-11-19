// HondaCANModule, part: D-059 discovery scan (probe env only; empty otherwise) (T23 split).
#include "HondaCANInternal.h"

#if CONN_DISCOVERY_PROBE
// ---------------------------------------------------------------------------
// D-059 item 3 -- one-shot discovery scan (probe env only).
// ---------------------------------------------------------------------------

void HondaCANModule::scanStep(unsigned long now) {
    if (_scanPhase == ScanPhase::WAIT_SESSION) {
        // After the listen window and the session start: when 0x50 confirms the session, or
        // one retry interval after the first session request (then NRCs and timeouts are
        // the findings). A late 0x50 during the scan does not restart it.
        if (!_sessionConfirmed && now - _firstSessionRequestMs < kSessionRetryMs) {
            return;
        }
        _scanPhase = ScanPhase::RUNNING;
        _scan.begin(_sessionConfirmed);
    }
    if (beforeDeadline(now, _scanHoldUntilMs)) {
        return;
    }
    int next = _scan.next();
    if (next < 0) {
        _scanPhase = ScanPhase::DONE;
        for (uint8_t i = 0; i < VEHICLE_CL250_DID_COUNT; i++) {
            _dids[i].lastRequestMs = now; // the DID schedule starts after the scan
        }
        // D-075 item 1: the scan-end arm is the epoch's deferred first arm (the scan ran
        // before the DID polling). Without a confirmed session the next confirmation is
        // that first arm, also if the session was lost meanwhile.
        if (_sessionConfirmed) {
            _newStopEpoch = false;
            armUnverifiedReads(now); // D-068: their first read is due at once, as without a scan
        } else {
            _newStopEpoch = true;
        }
        return;
    }
    const vehicle_cl250_scan_t& e = vehicle_cl250_discovery_scan[next];
    _pendingScanIndex = (int8_t)next;
    _pendingDidIndex = -1; // not a DID: no round-trip sample, no DID timeout count
    _pendingDid = 0;
    _pendingSid = e.request[0];
    _fcSentForPending = false;
    _requestLostBaseline = _bus.rxLostCount(); // a frame lost after this may be part of the answer
    // A Single Frame like every request; sendFrame() passes it through the D-020 gate.
    sendRequest(e.request, e.size);
    _udsState = UdsRequestState::WAITING;
    _requestSentMs = now;
    _requestFirstSentMs = now;
    _pendingSawNrc78 = false;
    _pendingRttSkip = true;
    _responseTimeoutMs = kResponseTimeoutMs;
}

bool HondaCANModule::onScanSingleFrame(const CanFrame& f, unsigned long now) {
    if (_pendingScanIndex < 0 || _udsState != UdsRequestState::WAITING) {
        return false;
    }
    uint8_t len;
    if (!singleFrameLength(f, len)) {
        return false;
    }
    uint8_t i = (uint8_t)_pendingScanIndex;
    const uint8_t* msg = &f.data[1];
    if (DiscoveryScan::matchesPositive(i, msg, len)) {
        _scan.recordPositive(i, msg, len);
        _lastGoodResponseMs = now;
        _udsState = UdsRequestState::COMPLETE;
        return true;
    }
    if (DiscoveryScan::matchesNegative(i, msg, len) && msg[2] != UDS_NRC_RESPONSE_PENDING) {
        // 0x78 is left to the normal NRC handling, which extends the wait (capped).
        _nrcCount++;
        _scan.recordNrc(i, msg[2]);
        _lastGoodResponseMs = now;
        _udsState = UdsRequestState::COMPLETE;
        return true;
    }
    return false;
}
#endif // CONN_DISCOVERY_PROBE
