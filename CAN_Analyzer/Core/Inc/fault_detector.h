/*
 * fault_detector.h
 * Level 4: detect abnormal CAN conditions.
 *
 * Detects:
 *  - Message timeout (expected ID not seen within threshold)
 *  - Value out of range
 *  - Explicit fault frames (ID 0x7DF)
 */

#ifndef FAULT_DETECTOR_H
#define FAULT_DETECTOR_H

#include "mcp2515.h"
#include "can_decoder.h"
#include <stdint.h>

/* ─── Timeout thresholds (ms) ───────────────────────────────────────── */
#define FAULT_TIMEOUT_SPEED     500
#define FAULT_TIMEOUT_RPM       500
#define FAULT_TIMEOUT_FUEL      2000
#define FAULT_TIMEOUT_TEMP      2000
#define FAULT_TIMEOUT_BATTERY   1000
#define FAULT_TIMEOUT_THROTTLE  500

/* ─── Value out-of-range limits ─────────────────────────────────────── */
#define FAULT_SPEED_MAX_KMH     200
#define FAULT_RPM_MAX           7500
#define FAULT_TEMP_MAX_C        120
#define FAULT_BATT_MIN_MV       11000
#define FAULT_BATT_MAX_MV       14800

/* ─── Fault flag bits ───────────────────────────────────────────────── */
#define FAULT_FLAG_ECU_FRAME    (1 << 0)   /* ECU sent a fault code     */
#define FAULT_FLAG_TIMEOUT      (1 << 1)   /* Message missing too long  */
#define FAULT_FLAG_RANGE        (1 << 2)   /* Value out of expected range */

/* ─── Public API ────────────────────────────────────────────────────── */

/** @brief  Initialise timeout watchdogs. */
void FaultDetector_Init(void);

/**
 * @brief  Feed a decoded frame into the fault detector.
 *         Call this every time a frame is received.
 * @param  decoded  Pointer to the decoded frame.
 * @param  tick     Current HAL_GetTick() timestamp.
 * @retval Bitfield of FAULT_FLAG_* bits (0 = all clear).
 */
uint8_t FaultDetector_Update(const DecodedFrame *decoded,
                             const CAN_Frame    *raw,
                             uint32_t            tick);

/**
 * @brief  Call periodically to check for message timeouts.
 * @param  tick  Current HAL_GetTick().
 * @retval Bitfield of FAULT_FLAG_* bits (0 = all clear).
 */
uint8_t FaultDetector_Check(uint32_t tick);

/** @brief  Return a description of the last fault detected. */
const char *FaultDetector_GetLastMsg(void);

/** @brief  Clear all fault state. */
void FaultDetector_Clear(void);

#endif /* FAULT_DETECTOR_H */
