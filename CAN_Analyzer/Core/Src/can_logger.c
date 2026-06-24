/*
 * can_logger.c
 * Circular buffer log for received CAN frames.
 */

#include "can_logger.h"
#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <string.h>

/* ─── Private state ─────────────────────────────────────────────────── */
static LogEntry  _log[LOG_MAX_ENTRIES];
static uint32_t  _head       = 0;   /* next write index (circular) */
static uint32_t  _total      = 0;   /* total frames ever logged     */

/* ─── UART handle (extern from main.c) ─────────────────────────────── */
extern UART_HandleTypeDef huart1;

static void uart_print(const char *s)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)s, strlen(s), HAL_MAX_DELAY);
}

/* ─── Public functions ──────────────────────────────────────────────── */

void CAN_Logger_Init(void)
{
    memset(_log, 0, sizeof(_log));
    _head  = 0;
    _total = 0;
}

void CAN_Logger_Add(uint32_t timestamp_ms,
                    const CAN_Frame   *raw,
                    const DecodedFrame *decoded)
{
    _log[_head].timestamp_ms = timestamp_ms;
    _log[_head].raw          = *raw;
    _log[_head].decoded      = *decoded;

    _head = (_head + 1) % LOG_MAX_ENTRIES;
    _total++;
}

void CAN_Logger_Dump(void)
{
    uart_print("\r\n========== CAN LOG DUMP ==========\r\n");

    uint32_t count  = (_total < LOG_MAX_ENTRIES) ? _total : LOG_MAX_ENTRIES;
    /* Start from oldest entry */
    uint32_t start  = (_total < LOG_MAX_ENTRIES) ? 0 : _head;

    char buf[80];
    for (uint32_t i = 0; i < count; i++)
    {
        uint32_t idx   = (start + i) % LOG_MAX_ENTRIES;
        LogEntry *e    = &_log[idx];
        uint32_t  ms   = e->timestamp_ms;
        uint32_t  sec  = ms / 1000;
        uint32_t  msec = ms % 1000;

        snprintf(buf, sizeof(buf),
                 "[%5lu.%03lu] ID:0x%03lX  %-14s = %s\r\n",
                 (unsigned long)sec,
                 (unsigned long)msec,
                 (unsigned long)e->decoded.can_id,
                 e->decoded.signal_name,
                 e->decoded.value_str);
        uart_print(buf);
    }

    snprintf(buf, sizeof(buf),
             "===  Total frames logged: %lu  ===\r\n\r\n",
             (unsigned long)_total);
    uart_print(buf);
}

void CAN_Logger_Clear(void)
{
    memset(_log, 0, sizeof(_log));
    _head  = 0;
    _total = 0;
    uart_print("[LOG] Cleared.\r\n");
}

uint32_t CAN_Logger_Count(void)
{
    return _total;
}
