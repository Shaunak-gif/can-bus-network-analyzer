/*
 * can_logger.h
 * Circular in-RAM log of received CAN frames with timestamps.
 * (Level 3 feature)
 *
 * Stored entries can be dumped to UART on demand (press 'L' in terminal).
 */

#ifndef CAN_LOGGER_H
#define CAN_LOGGER_H

#include "mcp2515.h"
#include "can_decoder.h"
#include <stdint.h>

#define LOG_MAX_ENTRIES   64    /* Number of frames to keep in RAM */

/* ─── Log entry ─────────────────────────────────────────────────────── */
typedef struct {
    uint32_t    timestamp_ms;   /* HAL_GetTick() at reception */
    CAN_Frame   raw;
    DecodedFrame decoded;
} LogEntry;

/* ─── Public API ────────────────────────────────────────────────────── */

/** @brief  Initialise the log buffer. */
void CAN_Logger_Init(void);

/** @brief  Append one frame to the circular log. */
void CAN_Logger_Add(uint32_t timestamp_ms,
                    const CAN_Frame   *raw,
                    const DecodedFrame *decoded);

/** @brief  Print all stored entries to UART (most recent last). */
void CAN_Logger_Dump(void);

/** @brief  Return total number of frames logged (wraps at LOG_MAX_ENTRIES). */
uint32_t CAN_Logger_Count(void);

/** @brief  Clear the log buffer. */
void CAN_Logger_Clear(void);

#endif /* CAN_LOGGER_H */
