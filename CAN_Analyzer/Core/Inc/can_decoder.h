/*
 * can_decoder.h
 * Decodes raw CAN frames into human-readable signal values.
 *
 * This is the "Level 2" feature described in the project spec.
 * The decoder knows about the specific IDs transmitted by the ECU Simulator.
 */

#ifndef CAN_DECODER_H
#define CAN_DECODER_H

#include "mcp2515.h"
#include <stdint.h>

/* ─── Signal IDs (same as ECU_Transmitter) ──────────────────────────── */
#define CAN_ID_SPEED        0x100
#define CAN_ID_RPM          0x101
#define CAN_ID_FUEL         0x102
#define CAN_ID_TEMP         0x103
#define CAN_ID_BATTERY      0x104
#define CAN_ID_THROTTLE     0x105
#define CAN_ID_FAULT        0x7DF

/* ─── Decoded frame result ──────────────────────────────────────────── */
typedef struct {
    uint32_t can_id;
    char     signal_name[20];   /* e.g. "Speed"     */
    char     value_str[20];     /* e.g. "80 km/h"   */
    uint8_t  is_fault;          /* 1 if this is a fault frame */
} DecodedFrame;

/* ─── Public API ────────────────────────────────────────────────────── */

/**
 * @brief  Decode a raw CAN_Frame into a human-readable DecodedFrame.
 * @param  raw     Pointer to the received CAN_Frame.
 * @param  out     Pointer to DecodedFrame to populate.
 * @retval 1 if decoded successfully, 0 if ID is unknown (raw dump shown).
 */
uint8_t CAN_Decode(const CAN_Frame *raw, DecodedFrame *out);

/**
 * @brief  Format a raw frame as a hex dump string.
 *         Used for Level 1 (raw display) and unknown IDs.
 * @param  raw  Pointer to CAN_Frame.
 * @param  buf  Output string buffer (at least 40 chars).
 */
void CAN_FormatRaw(const CAN_Frame *raw, char *buf, uint8_t buf_len);

#endif /* CAN_DECODER_H */
