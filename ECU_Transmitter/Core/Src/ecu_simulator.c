/*
 * ecu_simulator.c
 * Simulates a vehicle ECU by generating realistic-looking CAN data.
 *
 * The simulation models a simple "drive cycle":
 *   0–10 s   : idle (RPM ~800, speed 0)
 *  10–30 s   : accelerating to 80 km/h
 *  30–60 s   : cruising at 80 km/h
 *  60–75 s   : braking back to idle
 *  (then loops)
 */

#include "ecu_simulator.h"
#include "mcp2515.h"
#include <string.h>

/* ─── Internal helpers ──────────────────────────────────────────────── */

static uint8_t clamp8(int v, int lo, int hi)
{
    if (v < lo) return (uint8_t)lo;
    if (v > hi) return (uint8_t)hi;
    return (uint8_t)v;
}

static uint16_t clamp16(int v, int lo, int hi)
{
    if (v < lo) return (uint16_t)lo;
    if (v > hi) return (uint16_t)hi;
    return (uint16_t)v;
}

/* Simple pseudo-random jitter (no stdlib rand needed) */
static uint32_t _seed = 1;
static int8_t jitter(int8_t range)
{
    _seed = _seed * 1664525u + 1013904223u;
    return (int8_t)((_seed >> 16) % (2 * range + 1)) - range;
}

/* ─── ECU_UpdateState ────────────────────────────────────────────────── */
void ECU_UpdateState(ECU_State *ecu, uint32_t tick)
{
    /* Drive cycle period = 75 seconds, repeat */
    uint32_t t = (tick / 1000) % 75;   /* seconds within cycle */

    int speed = 0;
    int rpm   = 800;

    if (t < 10) {
        /* Idle */
        speed = 0;
        rpm   = 800 + jitter(30);
    } else if (t < 30) {
        /* Accelerating: 0 → 80 km/h over 20 s */
        speed = (int)((t - 10) * 4);
        rpm   = 800 + (int)((t - 10) * 160) + jitter(50);
    } else if (t < 60) {
        /* Cruising at 80 km/h */
        speed = 80 + jitter(2);
        rpm   = 2500 + jitter(80);
    } else {
        /* Braking: 80 → 0 over 15 s */
        speed = (int)(80 - ((t - 60) * 80 / 15));
        rpm   = 2500 - (int)((t - 60) * 110) + jitter(40);
        if (rpm < 800) rpm = 800;
    }

    ecu->speed_kmh      = clamp8(speed, 0, 255);
    ecu->rpm            = clamp16(rpm,  0, 8000);

    /* Fuel drains very slowly */
    uint8_t fuel_base   = (uint8_t)(100 - (tick / 60000) % 100);
    ecu->fuel_pct       = clamp8(fuel_base + jitter(1), 0, 100);

    /* Coolant warms up to 90 °C within first 60 s, stays there */
    uint32_t warm_s     = tick / 1000;
    int temp            = (warm_s < 60) ? (int)(40 + warm_s * 50 / 60)
                                        : 90;
    ecu->coolant_temp_c = clamp8(temp + jitter(2), 0, 150);

    /* Battery sags slightly under load */
    int batt            = 12600 - (ecu->rpm / 10) + jitter(20);
    ecu->battery_mv     = clamp16(batt, 10000, 15000);

    /* Throttle proportional to acceleration, zero when cruising/idle */
    if (t >= 10 && t < 30)
        ecu->throttle_pct = clamp8((int)((t - 10) * 3) + jitter(3), 0, 100);
    else if (t >= 30 && t < 60)
        ecu->throttle_pct = clamp8(15 + jitter(3), 0, 100);
    else
        ecu->throttle_pct = 0;

    /* Inject a demo fault at t=45 s */
    ecu->fault_code = (t == 45) ? 0x01 : 0x00;   /* 0x01 = "Coolant Temp High" */
}

/* ─── ECU_TransmitAll ────────────────────────────────────────────────── */
void ECU_TransmitAll(const ECU_State *ecu)
{
    CAN_Frame f;
    memset(&f, 0, sizeof(f));

    /* 0x100 – Speed (1 byte: km/h) */
    f.id = CAN_ID_SPEED;  f.dlc = 1;
    f.data[0] = ecu->speed_kmh;
    MCP2515_SendFrame(&f);

    /* 0x101 – RPM (2 bytes: big-endian) */
    f.id = CAN_ID_RPM;    f.dlc = 2;
    f.data[0] = (uint8_t)(ecu->rpm >> 8);
    f.data[1] = (uint8_t)(ecu->rpm & 0xFF);
    MCP2515_SendFrame(&f);

    /* 0x102 – Fuel level (1 byte: %) */
    f.id = CAN_ID_FUEL;   f.dlc = 1;
    f.data[0] = ecu->fuel_pct;
    MCP2515_SendFrame(&f);

    /* 0x103 – Coolant temperature (1 byte: °C) */
    f.id = CAN_ID_TEMP;   f.dlc = 1;
    f.data[0] = ecu->coolant_temp_c;
    MCP2515_SendFrame(&f);

    /* 0x104 – Battery voltage (2 bytes: mV, big-endian) */
    f.id = CAN_ID_BATTERY; f.dlc = 2;
    f.data[0] = (uint8_t)(ecu->battery_mv >> 8);
    f.data[1] = (uint8_t)(ecu->battery_mv & 0xFF);
    MCP2515_SendFrame(&f);

    /* 0x105 – Throttle position (1 byte: %) */
    f.id = CAN_ID_THROTTLE; f.dlc = 1;
    f.data[0] = ecu->throttle_pct;
    MCP2515_SendFrame(&f);

    /* 0x7DF – Fault (only send when active) */
    if (ecu->fault_code != 0x00) {
        f.id = CAN_ID_FAULT; f.dlc = 2;
        f.data[0] = 0x01;              /* Number of fault codes */
        f.data[1] = ecu->fault_code;
        MCP2515_SendFrame(&f);
    }
}
