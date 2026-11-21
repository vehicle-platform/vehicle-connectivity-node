// HondaCANModule, part: RX drain, foreign-tester detection and ISO-TP segmented reception with the one flow control (T23 split).
#include "HondaCANInternal.h"

bool HondaCANModule::isForeignTesterFrame(const CanFrame& f) {
    // Our ECU's physical request ID from any source address (29-bit normal fixed
    // addressing 0x18DA<TA><SA>), or exactly the ID when it is an 11-bit one.
    const bool primaryExtended = isotp::isExtendedId(VEHICLE_CL250_REQUEST_ID);
    const uint32_t primaryMask = primaryExtended ? ~isotp::kNormalFixedSourceAddressMask : 0xFFFFFFFFu;
    if (f.extended == primaryExtended && (f.id & primaryMask) == (VEHICLE_CL250_REQUEST_ID & primaryMask)) {
        return true;
    }
    if (f.extended == isotp::isExtendedId(VEHICLE_CL250_FALLBACK_REQUEST_ID) && f.id == VEHICLE_CL250_FALLBACK_REQUEST_ID) {
        return true;
    }
    // Q-021/D-040: the watch-only OBD functional (broadcast) request IDs of a generic OBD
    // tester. A 29-bit entry (0x18DB<TA><SA>) matches from any source address, like the
    // physical ID above; an 11-bit entry matches exactly.
    for (uint8_t i = 0; i < VEHICLE_CL250_FUNCTIONAL_WATCH_COUNT; i++) {
        const vehicle_cl250_watch_id_t& w = vehicle_cl250_functional_watch[i];
        const uint32_t mask = w.extended ? ~isotp::kNormalFixedSourceAddressMask : 0xFFFFFFFFu;
        if (f.extended == w.extended && (f.id & mask) == (w.id & mask)) {
            return true;
        }
    }
    return false;
}

// Reads incoming CAN frames. Bounded: at most kMaxRxFramesPerPass per call.
HondaCANModule::DrainResult HondaCANModule::drainRx(SystemState& state, unsigned long now) {
    uint8_t logBudget = kMaxRxLogLinesPerPass;
    CanFrame rxMsg;
    for (uint8_t n = 0; n < kMaxRxFramesPerPass; n++) {
        if (!_bus.receive(rxMsg)) {
            return DrainResult::DRAINED;
        }
        // TWAI does not receive its own frames, so a diagnostic request on the bus is
        // another tester (e.g. rt-core already polls). Two testers are never allowed
        // (D-021): latch this poller off, across resets until power-on (Q-018).
        if (isForeignTesterFrame(rxMsg)) {
            latchOff(TesterLatchReason::FOREIGN_TESTER, "another tester is on the vehicle bus (D-021)");
            state.engine.ecuPresent = false;
            return DrainResult::LATCHED;
        }
        bool isUDSResponse =
            (rxMsg.id == VEHICLE_CL250_RESPONSE_ID && rxMsg.extended == isotp::isExtendedId(VEHICLE_CL250_RESPONSE_ID)) ||
            (rxMsg.id == VEHICLE_CL250_FALLBACK_RESPONSE_ID &&
             rxMsg.extended == isotp::isExtendedId(VEHICLE_CL250_FALLBACK_RESPONSE_ID));
        if (!isUDSResponse || rxMsg.dlc < 2) {
            continue;
        }
        if (_listening) {
            // Nothing has been sent yet, so an ECU answer means someone else asked (their
            // request may have been lost or sent before the window began). Known false
            // positive, on the safe side: after a non-power-on reset the ECU may still
            // answer (or 0x78) a request this node sent before the reset; a power cycle
            // clears that latch.
            latchOff(TesterLatchReason::FOREIGN_TESTER, "the ECU answered a request this node did not send (D-021)");
            state.engine.ecuPresent = false;
            return DrainResult::LATCHED;
        }

        // G2.4 -- ISO-TP (ISO 15765-2) PCI byte check. data[0] high nibble is the frame
        // type: 0x0X = Single Frame (X = payload length), 0x1X = First Frame, 0x2X =
        // Consecutive Frame, 0x3X = Flow Control. A First Frame's data[1] is part of the
        // multi-frame length, not a SID, so parsing it as one would silently corrupt
        // SystemState. Multi-frame reassembly is only for the 0x19 answers (D-080) and the
        // D-059 probe's scan (below); vehicle-rt-core owns the long-term client (D-023), and
        // every DID used today fits in a Single Frame.
        uint8_t isoTpFrameType = (rxMsg.data[0] >> 4) & 0x0F;
        // D-059/D-080: segmented answers (an FC only for the 0x19 or scan request in flight).
        if (isoTpFrameType == IsoTpReceiver::kPciFirstFrame) {
            onFirstFrame(rxMsg, now, logBudget);
            continue;
        }
        if (isoTpFrameType == IsoTpReceiver::kPciConsecutiveFrame) {
            onConsecutiveFrame(rxMsg, now);
            continue;
        }
        if (isoTpFrameType == IsoTpReceiver::kPciSingleFrame &&
            (_isoRx.fromReceptionId(rxMsg) ||
             (_fcPending && rxMsg.id == _fcPendingFrame.id && rxMsg.extended == _fcPendingFrame.extended))) {
            _isoRx.abort(IsoTpReceiver::AbortReason::UNEXPECTED_FRAME); // then handled as usual
            endReception(now);
        }
        if (isoTpFrameType == IsoTpReceiver::kPciSingleFrame && onDtcSingleFrame(rxMsg, now)) {
            continue;
        }
#if CONN_DISCOVERY_PROBE
        if (isoTpFrameType == IsoTpReceiver::kPciSingleFrame && onScanSingleFrame(rxMsg, now)) {
            continue;
        }
#endif
        if (isoTpFrameType != 0x0) {
            if (takeLogSlot(logBudget)) {
                Serial.printf("[UDS WARNING] Unexpected ISO-TP frame type 0x%X (PCI=0x%02X) -- dropping frame.\n",
                    isoTpFrameType, rxMsg.data[0]);
            }
            continue;
        }
        // A ReadDataByIdentifier answer: the generated parser checks the Single Frame
        // PCI, the positive SID, the DID echo, the length and the physical range.
        uint16_t echoedDid = (uint16_t)((rxMsg.data[2] << 8) | rxMsg.data[3]);
        const vehicle_cl250_did_t* entry = vehicle_cl250_find(echoedDid);
        float value = 0.0f;
        if (entry != nullptr && vehicle_cl250_parse_response(echoedDid, rxMsg.data, rxMsg.dlc, &value)) {
            const uint8_t didIndex = (uint8_t)(entry - vehicle_cl250_dids);
            storeValue(state, didIndex, value, now);
            _didValue[didIndex] = value; // D-080 warm-idle trigger (bookkeeping only)
            _didValueMs[didIndex] = (now == 0) ? 1 : now;
            // D-066 item 4: the same validated answer, as raw data bytes for the BLE
            // vehicle-signal block. Bookkeeping only; nothing here changes what is sent.
            vsigRecordPositive(state.vehicleSignals.dids[didIndex], &rxMsg.data[4], entry->length, (uint32_t)now);
            // D-068: the DID echo proves the ECU serves this DID, also for a late answer, so
            // an earlier stop (e.g. an NRC 0x31 from the other response ID) is undone.
            _dids[didIndex].answered = true;
            // D-075 item 1 (f), review MINOR-1: a late answer (the other response ID) does not
            // undo a stop by the (f) counter; it undoes a 0x31 or timeout stop as before.
            if (_dids[didIndex].sessionNrcCount < kStopNrcReads) {
                _dids[didIndex].unsupported = false;
                _dids[didIndex].stopReason = StopReason::NONE;
            }
            _dids[didIndex].stopNrcCount = 0;
            _dids[didIndex].nrc31Seen = false; // review MINOR-2: with stopNrcCount, as in rt-core
            // G2.2 -- resolve the state machine only if this is the DID we're waiting on.
            if (_udsState == UdsRequestState::WAITING && echoedDid == _pendingDid) {
                if (didIndex == VEHICLE_CL250_IDX_ENGINE_SPEED) {
                    // D-069 item 3: only the answer to the current request is a fresh sample.
                    // In the quiet window after a timeout it may be the timed-out read's late
                    // answer (same DID echo), so it breaks the run instead (safety-reviewer
                    // MINOR-1).
                    if (_pendingRttSkip) {
                        _rpmRunSamples = 0;
                    } else {
                        watchEngineStart(value);
                    }
                }
                recordRoundTrip(now);
                _udsState = UdsRequestState::COMPLETE;
            }
            _lastGoodResponseMs = now; // G2.3: any DID response proves the ECU is present
        } else if ((rxMsg.data[0] & 0xF0) == 0 && (rxMsg.data[0] & 0x0F) >= 2 && rxMsg.dlc >= 3 &&
                   rxMsg.data[1] == kSidSessionResponse &&
                   (rxMsg.data[2] & 0x7F) == VEHICLE_CL250_SESSION_SUBFUNCTION) {
            // G2.3 -- Positive response to DiagnosticSessionControl (0x10).
            if (!_sessionConfirmed) {
                _sessionConfirmed = true;
                _sessionRetryNow = false;
                // D-075 (provisional) item 1: only the first confirmation of a stop epoch
                // (tester start, ECU lost) arms everything at once. A re-confirmation after
                // NRC 0x7E/0x7F stays in the epoch: stops, answered flags, the engine-start
                // flag and every lastRequestMs are kept.
                if (_newStopEpoch) {
                    _newStopEpoch = false;
                    armUnverifiedReads(now); // D-068 item 2: read every unverified DID once now
                }
                Serial.println("[UDS SUCCESS] Extended diagnostic session (0x10 0x03) confirmed by ECU.");
            }
            _lastGoodResponseMs = now;
        } else if (rxMsg.data[1] == UDS_SID_NEGATIVE_RESPONSE && rxMsg.dlc >= 4) {
            // G2.1 -- Negative response: [PCI][0x7F][echoed SID][NRC].
            uint8_t echoedSid = rxMsg.data[2];
            uint8_t nrc = rxMsg.data[3];
            _nrcCount++;
            _lastGoodResponseMs = now; // G2.3: a NRC still proves the ECU is alive and answering
            // Only an NRC for ReadDataByIdentifier can resolve or extend the pending DID
            // request; NRCs for 0x10/0x3E are logged and otherwise ignored.
            // A 0x19 is resolved only by onDtcSingleFrame() (its own response-ID rule, round-2
            // MINOR-4): a malformed or other-ID 7F 19 never resolves it here.
            bool forPendingRead = (_udsState == UdsRequestState::WAITING) &&
                                  (echoedSid == _pendingSid) && !_pendingDtc;

            if (nrc == UDS_NRC_RESPONSE_PENDING) {
                if (forPendingRead && !_pendingSawNrc78 && _pendingDidIndex >= 0) {
                    _pendingSawNrc78 = true; // D-058: once per request, and no round-trip sample
                    if (!_pendingRttSkip) {
                        // In the quiet window after a timeout this 0x78 (no DID) may be the
                        // timed-out read's: not counted against the pending DID.
                        _tester.recordNrc78((uint8_t)_pendingDidIndex);
                    }
                }
                // responsePending: the ECU is still working on the DID we're WAITING on.
                // Reset and double the timeout, but never beyond the generated maximum
                // total wait since the request was sent, so a 0x78 storm still times out.
                // D-068 (safety-reviewer MAJOR-1): never for an unverified DID, like a D-050
                // faulty read: its read holds the slot for at most the base timeout + one
                // step, the premise of the verified DIDs' gap bounds. Scan entries (-1) keep it.
                unsigned long waited = now - _requestFirstSentMs;
                const bool extendable = _pendingDidIndex < 0 || _dids[_pendingDidIndex].verified;
                if (forPendingRead && extendable && waited < kResponsePendingMaxMs) {
                    unsigned long left = kResponsePendingMaxMs - waited;
                    unsigned long doubled = min(_responseTimeoutMs * 2, kResponsePendingMaxMs);
                    _requestSentMs = now;
                    _responseTimeoutMs = min(doubled, left);
                }
                if (takeLogSlot(logBudget)) {
                    Serial.printf("[UDS] NRC 0x78 responsePending for SID 0x%02X -- extending wait to %lu ms.\n",
                        echoedSid, _responseTimeoutMs);
                }
            } else {
                if (takeLogSlot(logBudget)) {
                    Serial.printf("[UDS WARNING] Negative response: SID=0x%02X NRC=0x%02X (%s) [total NRCs=%lu]\n",
                        echoedSid, nrc, nrcName(nrc), (unsigned long)_nrcCount);
                }
                // D-068 (safety-reviewer MINOR-1): a read or tester present refused as not
                // supported in the active session means the ECU fell back to the default
                // session (a short reset, S3 expiry) without 3 s of silence. Ask for the
                // session again (retried below, D-020). D-075 item 1: its confirmation stays
                // in the same stop epoch and re-arms nothing; the refusal of a never-answered
                // unverified DID counts toward its stop (below).
                if (_sessionConfirmed &&
                    (nrc == UDS_NRC_SERVICE_NOT_SUPPORTED_IN_ACTIVE_SESSION ||
                     nrc == UDS_NRC_SUBFUNCTION_NOT_SUPPORTED_IN_ACTIVE_SESSION) &&
                    (echoedSid == UDS_SID_READ_DATA_BY_IDENTIFIER || echoedSid == VEHICLE_CL250_TESTER_PRESENT_SID)) {
                    _sessionConfirmed = false; // same stop epoch (D-075 item 1): no re-arm on confirm
                    Serial.println("[UDS] Extended session lost (NRC 0x7E/0x7F) -- requesting it again.");
                }
                // Any other NRC definitively resolves this request (not a timeout, not
                // success) -- don't leave the state machine WAITING on a rejected DID.
                if (forPendingRead) {
                    // D-066 item 4: shown as the pending DID's answer, except in the quiet
                    // window after a timeout, where the NRC (it carries no DID) may belong to
                    // the timed-out read (see recordRoundTrip()). A scan entry is no DID
                    // (_pendingDidIndex -1). Bookkeeping only, like the round-trip record.
                    if (_pendingDidIndex >= 0 && !_pendingRttSkip) {
                        vsigRecordNrc(state.vehicleSignals.dids[_pendingDidIndex], nrc);
                        // D-068 item 2: requestOutOfRange = the ECU does not serve this DID.
                        // Same quiet-window rule: an NRC carries no DID.
                        // Two separate reads (safety-reviewer MAJOR-2): requests go to both
                        // IDs, so one 0x31 may be the other ID's, or trail an earlier answer.
                        DidSlot& slot = _dids[_pendingDidIndex];
                        // D-075 (provisional) item 1: NRC 0x7E/0x7F to a read of an unverified
                        // DID without a positive answer in this epoch counts like 0x31.
                        const bool sessionNrc = nrc == UDS_NRC_SERVICE_NOT_SUPPORTED_IN_ACTIVE_SESSION ||
                            nrc == UDS_NRC_SUBFUNCTION_NOT_SUPPORTED_IN_ACTIVE_SESSION;
                        const bool nrc31 = nrc == UDS_NRC_REQUEST_OUT_OF_RANGE;
                        const bool counted = (nrc31 || sessionNrc) && !slot.verified && !slot.answered;
                        // D-075 item 1 (f), provisional: a session NRC is also counted, in its
                        // own counter, for an unverified DID that answered in this epoch
                        // (a positive answer does not reset it). Verified DIDs: never.
                        const bool countedF = sessionNrc && !slot.verified;
                        if (countedF) {
                            // D-075 item 1 (amended, provisional) (a), (f): only a counted
                            // refusal asks for the session at the next step instead of after
                            // kSessionRetryMs.
                            _sessionRetryNow = true;
                        }
                        if (counted && nrc31) {
                            slot.nrc31Seen = true;
                        }
                        bool stop = false;
                        if (counted && ++slot.stopNrcCount >= kStopNrcReads) {
                            stop = true;
                        }
                        if (countedF) {
                            if (slot.sessionNrcCount < UINT8_MAX) {
                                slot.sessionNrcCount++; // saturates, never wraps (review MINOR-3)
                            }
                        }
                        if (countedF && slot.sessionNrcCount >= kStopNrcReads) {
                            stop = true;
                        }
                        if (stop) {
                            slot.unsupported = true;
                            // Precedence: once a 0x31 was counted, the reason stays NRC_31.
                            slot.stopReason = slot.nrc31Seen ? StopReason::NRC_31 : StopReason::NRC_7E_7F;
                            Serial.printf("[UDS] DID 0x%04X not supported (NRC 0x%02X) -- not asked again in this stop epoch (D-075).\n",
                                _pendingDid, nrc);
                        }
                    }
                    if (_pendingDidIndex == VEHICLE_CL250_IDX_ENGINE_SPEED) {
                        _rpmRunSamples = 0; // D-069 item 3: an NRC breaks the run (MINOR-2)
                    }
                    recordRoundTrip(now);
                    _udsState = UdsRequestState::COMPLETE;
                }
            }
        }
    }
    return DrainResult::BACKLOG;
}

bool HondaCANModule::segmentedRequestPending() const {
    if (_udsState != UdsRequestState::WAITING) {
        return false;
    }
#if CONN_DISCOVERY_PROBE
    if (_pendingScanIndex >= 0) {
        return true;
    }
#endif
    return _pendingDtc;
}

void HondaCANModule::failSegmented(const char* why, uint16_t received, uint16_t announced, unsigned long now) {
    holdTx(now);
    if (_pendingDtc) {
        _dtc.onSegmentedFailure(why, received, announced); // D-080 item 3: 0x19 off for the epoch
    }
#if CONN_DISCOVERY_PROBE
    if (_pendingScanIndex >= 0) {
        _scan.recordAborted((uint8_t)_pendingScanIndex, why, received, announced);
    }
#endif
    _udsState = UdsRequestState::COMPLETE;
}

void HondaCANModule::onFirstFrame(const CanFrame& f, unsigned long now, uint8_t& logBudget) {
    // D-116 item 3 (mirrors rt-core isotp_core rx_first_frame): an invalid First Frame (shorter
    // than 8 bytes, FF_DL 0 = escape, FF_DL 1..7 = must be a Single Frame) is dropped as if it
    // had not been received: no hold, no log, no effect on a pending read or a reception.
    if (f.dlc < VEHICLE_CL250_FRAME_DLC || IsoTpReceiver::firstFrameLength(f) < IsoTpReceiver::kMinFirstFrameLength) {
        return;
    }
    if (_isoRx.active() || _fcPending) {
        const CanFrame& current = _isoRx.active() ? f : _fcPendingFrame;
        bool sameId = _isoRx.active() ? _isoRx.fromReceptionId(f)
                                      : (current.id == f.id && current.extended == f.extended);
        if (sameId) {
            // A second First Frame on the reception's ID: no new FC, the reception ends.
            _fcPending = false;
            _isoRx.abort(IsoTpReceiver::AbortReason::UNEXPECTED_FRAME);
            endReception(now);
        } else {
            // The other response ID answered too (requests go to both IDs): that one gets
            // no FC, and its ECU times out on N_Bs (safety-reviewer MINOR-2).
            holdTxForWithheldFf(now);
            if (_pendingDtc) {
                // D-080 MAJOR-1 (a): this would repeat for every segmented 0x19 answer; once
                // per epoch, then 0x19 is off (the reception in progress still completes).
                _dtc.haltAfterAnswer(DtcSnapshot::Halt::BOTH_IDS);
            }
        }
        return;
    }
    const uint16_t announced = IsoTpReceiver::firstFrameLength(f);
    // The data after the 2-byte PCI, only from an 8-byte frame (a shorter FF is invalid).
    const uint16_t inFrame = (f.dlc != VEHICLE_CL250_FRAME_DLC) ? 0u
        : (announced < IsoTpReceiver::kFirstFramePayload) ? announced : (uint16_t)IsoTpReceiver::kFirstFramePayload;
    // The FF must carry the in-flight request's positive SID and echo (for 0x19 0x04/0x06
    // also the DTC), so an FC only ever goes to the answer of our own request; never within
    // a quiet time (safety-reviewer MINOR-1).
    bool ours = false;
    if (segmentedRequestPending() && inFrame > 0) {
        uint32_t requestId = 0;
        ours = _pendingDtc && _dtc.matchesPositive(&f.data[2], inFrame) && pairedRequestId(f, requestId) &&
               (!_dtcLocked || (f.id == _dtcResponseId && f.extended == _dtcResponseExtended));
        if (ours && !_dtcLocked) {
            lockDtcTo(f, requestId); // the count's answer, segmented
        }
#if CONN_DISCOVERY_PROBE
        ours = ours || (_pendingScanIndex >= 0 &&
                        DiscoveryScan::matchesPositive((uint8_t)_pendingScanIndex, &f.data[2], inFrame));
#endif
    }
    if (ours && !_fcSentForPending && !beforeDeadline(now, _txQuietUntilMs) && IsoTpReceiver::acceptableLength(f) != 0) {
        _fcSentForPending = true; // one FC per request (BS 0), also if it is never sent
        _fcPending = true;        // sent by poll() once the drain is complete
        _fcPendingFrame = f;
        _fcPendingMs = now;
        return;
    }
    // No FC: too long, not an answer to the request in flight, in a quiet time, or a second FF.
    holdTxForWithheldFf(now);
    if (takeLogSlot(logBudget)) {
        Serial.printf("[UDS WARNING] First Frame (FF_DL=%u) not answered with an FC; no request for %lu ms.\n",
            announced, kResponsePendingMaxMs);
    }
    if (!ours && inFrame > 0 && f.data[2] == (uint8_t)(UDS_SID_READ_DTC_INFORMATION + UDS_POSITIVE_RESPONSE_OFFSET)) {
        // Round-2 MAJOR-A: any quiet time a 0x19 answer causes (a late copy, the other
        // addressing, another DTC's echo) is the epoch's one; then 0x19 is off. The request in
        // flight, if any, still resolves (or times out).
        _dtc.haltAfterAnswer(DtcSnapshot::Halt::BOTH_IDS);
    }
    if (ours) {
#if CONN_DISCOVERY_PROBE
        if (_pendingScanIndex >= 0) {
            _scan.recordFfRefused((uint8_t)_pendingScanIndex, announced);
        }
#endif
        if (_pendingDtc) {
            // D-080 item 3: above max_ff_dl (or no FC possible) = refused, 0x19 off for the epoch.
            _dtc.onSegmentedFailure("First Frame refused", 0, announced);
        }
        _udsState = UdsRequestState::COMPLETE;
    } else if (_udsState == UdsRequestState::WAITING && _pendingDidIndex >= 0 && inFrame >= 3 &&
               f.data[2] == (uint8_t)(UDS_SID_READ_DATA_BY_IDENTIFIER + UDS_POSITIVE_RESPONSE_OFFSET) &&
               (uint16_t)((f.data[3] << 8) | f.data[4]) == _pendingDid) {
        // yaml: a segmented DID answer that got no FC is not a DID timeout (no sample either).
        // D-068 (safety-reviewer MINOR-2): every table DID fits a Single Frame, so an
        // unverified DID answered this way is never decoded; stop it, or its read would hold
        // every request for the quiet time each period.
        DidSlot& slot = _dids[_pendingDidIndex];
        if (!slot.verified && !slot.answered) {
            slot.unsupported = true;
            slot.stopReason = StopReason::FIRST_FRAME;
        } else {
            // A verified or answered DID is never stopped: the D-050..D-053 skip, so the
            // quiet time does not repeat every period (mirrors rt-core's stop_or_skip).
            slot.skipUntilMs = now + kDidSkipCooldownMs;
        }
        _udsState = UdsRequestState::COMPLETE;
    } else if (_udsState == UdsRequestState::WAITING && _pendingDidIndex >= 0 && inFrame >= 1 &&
               f.data[2] != (uint8_t)(UDS_SID_READ_DATA_BY_IDENTIFIER + UDS_POSITIVE_RESPONSE_OFFSET)) {
        // D-116 item 2 (mirrors rt-core uds_client_core F4): a withheld First Frame that is not a
        // positive 0x62 answer is not this read's. The read ends without a DID consequence: no
        // timeout count, no skip or stop, no sample; the DID is due again as before it was sent.
        // The 0x19 branch above may also have run (a late 0x19 answer). Only what happens to the
        // pending read changes: no request, service or gate, the quiet window is the one held above.
        _dids[_pendingDidIndex].lastRequestMs = _prevDidRequestMs;
        if (_pendingDidIndex == VEHICLE_CL250_IDX_ENGINE_SPEED) {
            _rpmRunSamples = 0; // D-069 item 3: the RPM samples must be consecutive
        }
        _udsState = UdsRequestState::IDLE; // not COMPLETE: that would clear consecutiveTimeouts
    }
}

// A quiet time started meanwhile by an FF on the other response ID does not stop this FC: it
// answers our own request, and the yaml quiet-time rule covers requests only (intended).
void HondaCANModule::sendPendingFlowControl(unsigned long now) {
    _fcPending = false;
    if (!segmentedRequestPending()) {
        // The request was resolved meanwhile: no FC, so the yaml quiet time (round-2 MAJOR-B).
        // E-21 NIT: no trigger is known; if one shows up it is an FF we did not answer, so
        // 0x19 is off for the epoch like every other quiet time caused by a 0x19 answer.
        holdTx(now);
        if (_fcPendingFrame.dlc > 2 &&
            _fcPendingFrame.data[2] == (uint8_t)(UDS_SID_READ_DTC_INFORMATION + UDS_POSITIVE_RESPONSE_OFFSET)) {
            _dtc.haltAfterAnswer(DtcSnapshot::Halt::BOTH_IDS);
        }
        return;
    }
    const uint16_t announced = IsoTpReceiver::firstFrameLength(_fcPendingFrame);
    const char* why = nullptr;
    const uint32_t lost = _bus.rxLostCount();
    if (lost != _requestLostBaseline) {
        why = "lost frame before the FC"; // part of the answer may be missing
    } else if (now - _fcPendingMs > kResponseTimeoutMs) {
        why = "FC too late"; // the drain stayed backlogged; well inside the ECU's N_Bs
    } else if (!sendFlowControl(_fcPendingFrame)) {
        why = "FC not sent";
    }
    if (why != nullptr) {
        failSegmented(why, 0, announced, now);
        return;
    }
    // Times count from this drain; the baseline is the count the request went out with.
    _isoRx.start(_fcPendingFrame, now, lost);
}

void HondaCANModule::onConsecutiveFrame(const CanFrame& f, unsigned long now) {
    if (!_isoRx.fromReceptionId(f)) {
        return; // no reception (or after an abort): a stray CF is dropped, never printed
    }
    if (_isoRx.onConsecutive(f, now) == IsoTpReceiver::Result::IN_PROGRESS) {
        return;
    }
    endReception(now);
}

void HondaCANModule::endReception(unsigned long now) {
    _fcPending = false;
    if (segmentedRequestPending()) {
        if (_isoRx.complete()) {
            // The FF matched the request's SID and echo; the payload starts at the SID.
            if (_pendingDtc) {
                checkDtcSlot(now);
                _dtc.onPositive(_isoRx.payload(), _isoRx.length(), _isoRx.receptionId());
            }
#if CONN_DISCOVERY_PROBE
            if (_pendingScanIndex >= 0) {
                _scan.recordPositive((uint8_t)_pendingScanIndex, _isoRx.payload(), _isoRx.length());
            }
#endif
            _lastGoodResponseMs = now;
            _udsState = UdsRequestState::COMPLETE;
        } else {
            // Nothing of a partial answer is printed (in the probe it may be the VIN).
            failSegmented(IsoTpReceiver::reasonText(_isoRx.abortReason()), _isoRx.received(), _isoRx.length(), now);
        }
    } else if (!_isoRx.complete()) {
        holdTx(now);
    }
    _isoRx.reset();
}

bool HondaCANModule::sendFlowControl(const CanFrame& firstFrame) {
    // The request ID paired with the response ID the First Frame came on (gen/ IDs only).
    uint32_t id = 0;
    const bool paired = pairedRequestId(firstFrame, id);
    CanFrame fc;
    fc.id = id;
    fc.extended = isotp::isExtendedId(id);
    fc.dlc = VEHICLE_CL250_FRAME_DLC;
    for (uint8_t i = 0; i < VEHICLE_CL250_FRAME_DLC; i++) {
        fc.data[i] = vehicle_cl250_fc_cts[i]; // byte for byte from gen/ (D-059 item 1)
    }
    // Same last word as every request: the latches, the listen window, and the D-020 gate.
    if (!paired || _latchedOff || _listening || !vehicle_cl250_frame_allowed(fc.data, fc.dlc)) {
        _blockedFrames++;
        Serial.printf("[UDS BLOCKED] Flow Control to 0x%08lX refused (total=%lu).\n",
            (unsigned long)fc.id, (unsigned long)_blockedFrames);
        return false;
    }
    if (!_bus.transmit(fc)) {
        return false;
    }
    Serial.printf("[ISOTP] FC.CTS sent to 0x%08lX for a %u-byte answer.\n",
        (unsigned long)fc.id, IsoTpReceiver::firstFrameLength(firstFrame));
    return true;
}
