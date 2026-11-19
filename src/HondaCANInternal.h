// Internal to the HondaCANModule translation units (T23 split of HondaCANModule.cpp by
// concern): the typed copies of the generated timing macros, the local policy constants and
// the small helpers every part shares. Not a public API; include HondaCANModule.h instead.
#ifndef HONDA_CAN_INTERNAL_H
#define HONDA_CAN_INTERNAL_H

#include "HondaCANModule.h"
#include "IsoTpCan.h"
#include "uds_iso14229.h" // generated: D-040 client subset (gen/c/conn/)

namespace {
// Typed copies of the generated timing macros, so min()/comparisons see one type.
const unsigned long kTesterPresentPeriodMs = VEHICLE_CL250_TESTER_PRESENT_PERIOD_MS;
const unsigned long kSessionRetryMs = VEHICLE_CL250_SESSION_RETRY_INTERVAL_MS;
const unsigned long kResponseTimeoutMs = VEHICLE_CL250_RESPONSE_TIMEOUT_BASE_MS;
const unsigned long kResponsePendingMaxMs = VEHICLE_CL250_RESPONSE_TIMEOUT_MAX_MS;
const uint8_t kMaxConsecutiveTimeouts = VEHICLE_CL250_MAX_CONSECUTIVE_TIMEOUTS;
const unsigned long kDidSkipCooldownMs = VEHICLE_CL250_DID_SKIP_COOLDOWN_MS;
const unsigned long kEcuAbsentTimeoutMs = VEHICLE_CL250_ECU_ABSENT_TIMEOUT_MS;
const unsigned long kBackoffMinMs = VEHICLE_CL250_BUS_OFF_BACKOFF_INITIAL_MS;
const unsigned long kBackoffMaxMs = VEHICLE_CL250_BUS_OFF_BACKOFF_MAX_MS;
// Response IDs of the ECU; `out` = the paired request ID. False for any other frame.
inline bool pairedRequestId(const CanFrame& f, uint32_t& out) {
    if (f.id == VEHICLE_CL250_RESPONSE_ID && f.extended == isotp::isExtendedId(VEHICLE_CL250_RESPONSE_ID)) {
        out = VEHICLE_CL250_REQUEST_ID;
        return true;
    }
    if (f.id == VEHICLE_CL250_FALLBACK_RESPONSE_ID &&
        f.extended == isotp::isExtendedId(VEHICLE_CL250_FALLBACK_RESPONSE_ID)) {
        out = VEHICLE_CL250_FALLBACK_REQUEST_ID;
        return true;
    }
    return false;
}

// D-080 item 4 / D-068 item 1: a 0x19 transaction may hold the slot as long as an unverified
// DID read (base timeout + one step); longer = the epoch's one 0x19 fault (MAJOR-2).
const unsigned long kDtcSlotMaxMs = VEHICLE_CL250_RESPONSE_TIMEOUT_BASE_MS + VEHICLE_CL250_CLIENT_STEP_MAX_MS;

// Local policy of this TEMPORARY tester, not a vehicle definition: after this many
// bus-off events the driver is stopped for good (until reboot) instead of rejoining the
// OEM bus forever and putting error frames on it. Candidate for vehicle-defs.
const uint32_t kMaxBusOffEvents = 5;

// D-068 item 2 (safety-reviewer MAJOR-2): NRC 0x31 on this many separate reads, with no
// positive answer between them, stops an unverified DID. Local policy of this tester.
const uint8_t kStopNrcReads = 2;

// Local policy of this TEMPORARY tester (Q-018): listen-only for this long after begin()
// before the first request, so a tester that is already polling the ECU is seen before
// this node ever transmits (D-021). Candidate for vehicle-defs, like the one above.
const unsigned long kListenOnlyWindowMs = 2000;

// Bounded RX drain: at most this many frames per pass. Twice the TWAI RX queue depth
// (hal/CanRxQueue.h), so one pass empties the queue (also a whole CF burst after the FC),
// while a flooded bus still cannot stall the loop.
static_assert(2u * kCanRxQueueLen <= 255u, "the drain bound must fit uint8_t");
const uint8_t kMaxRxFramesPerPass = (uint8_t)(2u * kCanRxQueueLen);
// At most this many per-frame log lines per pass (Serial output is slow; see TWDT).
const uint8_t kMaxRxLogLinesPerPass = 4;
// DID timeout warnings: at most one timeout happens per pass, so they are bounded per time
// window instead. A log policy of this temporary node, not a vehicle value (Serial is slow,
// see TWDT).
const unsigned long kTimeoutLogWindowMs = 1000;
const uint8_t kMaxTimeoutLogLinesPerWindow = kMaxRxLogLinesPerPass;

// Per-pass log budget: true (and one slot used) while the budget lasts.
inline bool takeLogSlot(uint8_t& budget) {
    if (budget == 0) {
        return false;
    }
    budget--;
    return true;
}

inline const char* latchReasonText(TesterLatchReason reason) {
    switch (reason) {
        case TesterLatchReason::FOREIGN_TESTER: return "another tester on the bus";
        case TesterLatchReason::BUS_OFF: return "too many bus-off events";
        case TesterLatchReason::UNKNOWN: return "latch record invalid after a reset";
        default: return "not latched";
    }
}

// NRC name for logs only. Only the NRCs the generated client subset names (D-040);
// the log line always carries the raw code as well.
inline const char* nrcName(uint8_t nrc) {
    switch (nrc) {
        case UDS_NRC_RESPONSE_PENDING: return "responsePending";
        case UDS_NRC_SUBFUNCTION_NOT_SUPPORTED_IN_ACTIVE_SESSION: return "subFunctionNotSupportedInActiveSession";
        case UDS_NRC_SERVICE_NOT_SUPPORTED_IN_ACTIVE_SESSION: return "serviceNotSupportedInActiveSession";
        case UDS_NRC_REQUEST_OUT_OF_RANGE: return "requestOutOfRange";
        default: return "other";
    }
}

// millis()-wrap-safe "deadline not reached yet" (deadline 0 = none).
inline bool beforeDeadline(unsigned long now, unsigned long deadline) {
    return deadline != 0 && (long)(deadline - now) > 0;
}

// Single Frame PCI check shared by the 0x19 and scan answer paths: the payload length `len`
// (low nibble of byte 0) must be 1..7 and fit in the received DLC. False = not a valid SF.
inline bool singleFrameLength(const CanFrame& f, uint8_t& len) {
    len = (uint8_t)(f.data[0] & 0x0F);
    return !(len == 0 || len > isotp::kSingleFrameMaxLen || len + 1u > f.dlc);
}

const uint8_t kSidSessionResponse = VEHICLE_CL250_SESSION_POSITIVE_SID;

const uint8_t kSessionRequest[] = {VEHICLE_CL250_SESSION_SID, VEHICLE_CL250_SESSION_SUBFUNCTION};
const uint8_t kTesterPresentRequest[] = {VEHICLE_CL250_TESTER_PRESENT_SID,
                                         VEHICLE_CL250_TESTER_PRESENT_SUBFUNCTION};

// D-059 scan and D-080 DTC report lines go to USB serial.
inline void reportSerialLine(const char* line) {
    Serial.println(line);
}
} // namespace
#endif // HONDA_CAN_INTERNAL_H
