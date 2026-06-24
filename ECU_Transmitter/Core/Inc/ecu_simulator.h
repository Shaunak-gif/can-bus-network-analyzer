/*
 * ecu_simulator.h
 * Fake ECU data definitions for the CAN Bus Transmitter (STM32 #1)
 *
 * Each "signal" is sent on its own CAN ID so the Analyzer
 * can decode them independently.
 */

#ifndef ECU_SIMULATOR_H
#define ECU_SIMULATOR_H

#include <stdint.h>

/* ─── CAN Message IDs ───────────────────────────────────────────────── */
#define CAN_ID_SPEED        0x100   /* Vehicle speed   (km/h)  1 byte  */
#define CAN_ID_RPM          0x101   /* Engine RPM             2 bytes  */
#define CAN_ID_FUEL         0x102   /* Fuel level      (%)    1 byte   */
#define CAN_ID_TEMP         0x103   /* Coolant temp    (°C)   1 byte   */
#define CAN_ID_BATTERY      0x104   /* Battery voltage (mV)   2 bytes  */
#define CAN_ID_THROTTLE     0x105   /* Throttle pos    (%)    1 byte   */
#define CAN_ID_FAULT        0x7DF   /* OBD-II-style fault     2 bytes  */

/* ─── ECU State ─────────────────────────────────────────────────────── */
typedef struct {
    uint8_t  speed_kmh;        /*  0–255 km/h          */
    uint16_t rpm;              /*  0–8000 RPM           */
    uint8_t  fuel_pct;         /*  0–100 %              */
    uint8_t  coolant_temp_c;   /*  0–150 °C             */
    uint16_t battery_mv;       /* 10000–15000 mV        */
    uint8_t  throttle_pct;     /*  0–100 %              */
    uint8_t  fault_code;       /*  0 = no fault         */
} ECU_State;

/* ─── Public API ────────────────────────────────────────────────────── */

/**
 * @brief Fill ECU_State with a simulated driving scenario.
 *        Call periodically to "drive" the simulated vehicle.
 * @param ecu   Pointer to ECU_State to update.
 * @param tick  System tick (ms) used to animate values.
 */
void ECU_UpdateState(ECU_State *ecu, uint32_t tick);

/**
 * @brief Transmit all ECU signals as individual CAN frames.
 * @param ecu  Pointer to current ECU_State.
 */
void ECU_TransmitAll(const ECU_State *ecu);

#endif /* ECU_SIMULATOR_H */
