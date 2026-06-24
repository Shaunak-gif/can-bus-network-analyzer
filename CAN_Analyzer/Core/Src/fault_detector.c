/*
 * fault_detector.c
 * Timeout watchdog and value-range checker for CAN signals.
 */

#include "fault_detector.h"
#include <string.h>
#include <stdio.h>

/* ─── Per-signal watchdog ───────────────────────────────────────────── */
typedef struct {
    uint32_t can_id;
    uint32_t timeout_ms;
    uint32_t last_seen;       /* tick of last received message */
    uint8_t  ever_seen;       /* 0 until first message arrives */
} Watchdog;

static Watchdog _watchdogs[] = {
    { CAN_ID_SPEED,    FAULT_TIMEOUT_SPEED,    0, 0 },
    { CAN_ID_RPM,      FAULT_TIMEOUT_RPM,      0, 0 },
    { CAN_ID_FUEL,     FAULT_TIMEOUT_FUEL,     0, 0 },
    { CAN_ID_TEMP,     FAULT_TIMEOUT_TEMP,     0, 0 },
    { CAN_ID_BATTERY,  FAULT_TIMEOUT_BATTERY,  0, 0 },
    { CAN_ID_THROTTLE, FAULT_TIMEOUT_THROTTLE, 0, 0 },
};
#define NUM_WATCHDOGS (sizeof(_watchdogs) / sizeof(_watchdogs[0]))

static char _last_fault_msg[64] = "No faults";

static void set_fault(const char *msg)
{
    strncpy(_last_fault_msg, msg, sizeof(_last_fault_msg) - 1);
}

/* ─── Public functions ──────────────────────────────────────────────── */

void FaultDetector_Init(void)
{
    for (uint8_t i = 0; i < NUM_WATCHDOGS; i++) {
        _watchdogs[i].last_seen  = 0;
        _watchdogs[i].ever_seen  = 0;
    }
    strncpy(_last_fault_msg, "No faults", sizeof(_last_fault_msg));
}

uint8_t FaultDetector_Update(const DecodedFrame *decoded,
                             const CAN_Frame    *raw,
                             uint32_t            tick)
{
    uint8_t flags = 0;

    /* 1. Update watchdog for this ID */
    for (uint8_t i = 0; i < NUM_WATCHDOGS; i++) {
        if (_watchdogs[i].can_id == decoded->can_id) {
            _watchdogs[i].last_seen = tick;
            _watchdogs[i].ever_seen = 1;
            break;
        }
    }

    /* 2. Check for explicit ECU fault frame */
    if (decoded->is_fault) {
        char msg[64];
        snprintf(msg, sizeof(msg), "ECU FAULT: %s", decoded->value_str);
        set_fault(msg);
        flags |= FAULT_FLAG_ECU_FRAME;
    }

    /* 3. Value range checks */
    switch (decoded->can_id) {
        case CAN_ID_SPEED:
            if (raw->data[0] > FAULT_SPEED_MAX_KMH) {
                set_fault("Speed out of range");
                flags |= FAULT_FLAG_RANGE;
            }
            break;

        case CAN_ID_RPM: {
            uint16_t rpm = ((uint16_t)raw->data[0] << 8) | raw->data[1];
            if (rpm > FAULT_RPM_MAX) {
                set_fault("RPM exceeds limit");
                flags |= FAULT_FLAG_RANGE;
            }
            break;
        }

        case CAN_ID_TEMP:
            if (raw->data[0] > FAULT_TEMP_MAX_C) {
                set_fault("Coolant temp critical");
                flags |= FAULT_FLAG_RANGE;
            }
            break;

        case CAN_ID_BATTERY: {
            uint16_t mv = ((uint16_t)raw->data[0] << 8) | raw->data[1];
            if (mv < FAULT_BATT_MIN_MV) {
                set_fault("Battery voltage low");
                flags |= FAULT_FLAG_RANGE;
            } else if (mv > FAULT_BATT_MAX_MV) {
                set_fault("Battery voltage high");
                flags |= FAULT_FLAG_RANGE;
            }
            break;
        }

        default:
            break;
    }

    return flags;
}

uint8_t FaultDetector_Check(uint32_t tick)
{
    uint8_t flags = 0;

    for (uint8_t i = 0; i < NUM_WATCHDOGS; i++) {
        /* Only flag timeout if we've seen at least one message */
        if (!_watchdogs[i].ever_seen) continue;

        if ((tick - _watchdogs[i].last_seen) > _watchdogs[i].timeout_ms) {
            char msg[48];
            snprintf(msg, sizeof(msg),
                     "TIMEOUT: ID 0x%03lX missing",
                     (unsigned long)_watchdogs[i].can_id);
            set_fault(msg);
            flags |= FAULT_FLAG_TIMEOUT;
        }
    }

    return flags;
}

const char *FaultDetector_GetLastMsg(void)
{
    return _last_fault_msg;
}

void FaultDetector_Clear(void)
{
    FaultDetector_Init();
}
