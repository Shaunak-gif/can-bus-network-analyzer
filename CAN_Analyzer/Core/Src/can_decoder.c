/*
 * can_decoder.c
 * Decodes CAN frames from the ECU Simulator into readable strings.
 */

#include "can_decoder.h"
#include <stdio.h>
#include <string.h>

/* ─── Fault code look-up table ──────────────────────────────────────── */
typedef struct { uint8_t code; const char *desc; } FaultEntry;

static const FaultEntry fault_table[] = {
    { 0x01, "Coolant Temp High" },
    { 0x02, "Low Fuel"          },
    { 0x03, "Battery Low"       },
    { 0x04, "RPM Limit"         },
    { 0x00, NULL }              /* sentinel */
};

static const char *lookup_fault(uint8_t code)
{
    for (uint8_t i = 0; fault_table[i].desc != NULL; i++)
        if (fault_table[i].code == code)
            return fault_table[i].desc;
    return "Unknown Fault";
}

/* ─── CAN_Decode ────────────────────────────────────────────────────── */
uint8_t CAN_Decode(const CAN_Frame *raw, DecodedFrame *out)
{
    out->can_id   = raw->id;
    out->is_fault = 0;

    switch (raw->id)
    {
        /* ── 0x100 : Speed (1 byte, km/h) ─────────────────────────── */
        case CAN_ID_SPEED:
            strncpy(out->signal_name, "Speed", sizeof(out->signal_name));
            snprintf(out->value_str, sizeof(out->value_str),
                     "%u km/h", raw->data[0]);
            return 1;

        /* ── 0x101 : RPM (2 bytes, big-endian) ─────────────────────── */
        case CAN_ID_RPM: {
            uint16_t rpm = ((uint16_t)raw->data[0] << 8) | raw->data[1];
            strncpy(out->signal_name, "RPM", sizeof(out->signal_name));
            snprintf(out->value_str, sizeof(out->value_str),
                     "%u RPM", rpm);
            return 1;
        }

        /* ── 0x102 : Fuel (1 byte, %) ───────────────────────────────── */
        case CAN_ID_FUEL:
            strncpy(out->signal_name, "Fuel", sizeof(out->signal_name));
            snprintf(out->value_str, sizeof(out->value_str),
                     "%u%%", raw->data[0]);
            return 1;

        /* ── 0x103 : Coolant Temperature (1 byte, °C) ──────────────── */
        case CAN_ID_TEMP:
            strncpy(out->signal_name, "Coolant Temp", sizeof(out->signal_name));
            snprintf(out->value_str, sizeof(out->value_str),
                     "%u °C", raw->data[0]);
            return 1;

        /* ── 0x104 : Battery Voltage (2 bytes, mV big-endian) ─────── */
        case CAN_ID_BATTERY: {
            uint16_t mv = ((uint16_t)raw->data[0] << 8) | raw->data[1];
            strncpy(out->signal_name, "Battery", sizeof(out->signal_name));
            snprintf(out->value_str, sizeof(out->value_str),
                     "%u.%01u V", mv / 1000, (mv % 1000) / 100);
            return 1;
        }

        /* ── 0x105 : Throttle (1 byte, %) ──────────────────────────── */
        case CAN_ID_THROTTLE:
            strncpy(out->signal_name, "Throttle", sizeof(out->signal_name));
            snprintf(out->value_str, sizeof(out->value_str),
                     "%u%%", raw->data[0]);
            return 1;

        /* ── 0x7DF : Fault Frame ────────────────────────────────────── */
        case CAN_ID_FAULT:
            strncpy(out->signal_name, "FAULT", sizeof(out->signal_name));
            snprintf(out->value_str, sizeof(out->value_str),
                     "[%02X] %s", raw->data[1], lookup_fault(raw->data[1]));
            out->is_fault = 1;
            return 1;

        default:
            /* Unknown ID – caller should use CAN_FormatRaw */
            strncpy(out->signal_name, "Unknown", sizeof(out->signal_name));
            CAN_FormatRaw(raw, out->value_str, sizeof(out->value_str));
            return 0;
    }
}

/* ─── CAN_FormatRaw ─────────────────────────────────────────────────── */
void CAN_FormatRaw(const CAN_Frame *raw, char *buf, uint8_t buf_len)
{
    int pos = 0;
    for (uint8_t i = 0; i < raw->dlc && i < 8; i++) {
        int written = snprintf(buf + pos, buf_len - pos,
                               "%02X ", raw->data[i]);
        if (written < 0 || pos + written >= buf_len) break;
        pos += written;
    }
    if (pos > 0 && buf[pos - 1] == ' ')
        buf[pos - 1] = '\0';   /* trim trailing space */
}
