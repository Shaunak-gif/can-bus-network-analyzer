/*
 * mcp2515.h
 * MCP2515 CAN Controller Driver
 * Communicates via SPI with the STM32
 */

#ifndef MCP2515_H
#define MCP2515_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* ─── MCP2515 SPI Instructions ─────────────────────────────────────── */
#define MCP_RESET       0xC0
#define MCP_READ        0x03
#define MCP_WRITE       0x02
#define MCP_RTS_TX0     0x81   /* Request-to-Send TX Buffer 0 */
#define MCP_READ_RX0    0x90   /* Read RX Buffer 0 */
#define MCP_READ_STATUS 0xA0
#define MCP_BIT_MODIFY  0x05

/* ─── Register Addresses ────────────────────────────────────────────── */
#define MCP_CANSTAT     0x0E
#define MCP_CANCTRL     0x0F

/* Baud Rate Config (CNF1/2/3) */
#define MCP_CNF1        0x2A
#define MCP_CNF2        0x29
#define MCP_CNF3        0x28

/* Interrupt */
#define MCP_CANINTE     0x2B
#define MCP_CANINTF     0x2C

/* TX Buffer 0 */
#define MCP_TXB0CTRL    0x30
#define MCP_TXB0SIDH    0x31
#define MCP_TXB0SIDL    0x32
#define MCP_TXB0DLC     0x35
#define MCP_TXB0D0      0x36

/* RX Buffer 0 */
#define MCP_RXB0CTRL    0x60
#define MCP_RXB0SIDH    0x61
#define MCP_RXB0SIDL    0x62
#define MCP_RXB0DLC     0x65
#define MCP_RXB0D0      0x66

/* ─── CANCTRL Mode Bits ──────────────────────────────────────────────── */
#define MCP_MODE_NORMAL     0x00
#define MCP_MODE_CONFIG     0x80
#define MCP_MODE_LOOPBACK   0x40
#define MCP_MODE_LISTEN     0x60

/* ─── Baud Rate Settings (16 MHz crystal, 500 kbps) ─────────────────── */
/*
 * Fosc = 16 MHz
 * TQ   = 2 * (BRP+1) / Fosc
 * Target: 500 kbps  →  CNF1=0x00, CNF2=0x90, CNF3=0x02
 *
 * For 250 kbps use CNF1=0x01, CNF2=0x90, CNF3=0x02
 */
#define MCP_CNF1_500K   0x00
#define MCP_CNF2_500K   0x90
#define MCP_CNF3_500K   0x02

/* ─── Status / Error bits ───────────────────────────────────────────── */
#define MCP_STAT_RX0IF  0x01   /* RX Buffer 0 Full */
#define MCP_STAT_RX1IF  0x02   /* RX Buffer 1 Full */

/* ─── CAN Frame Structure ───────────────────────────────────────────── */
typedef struct {
    uint32_t id;          /* 11-bit Standard ID (0x000–0x7FF) */
    uint8_t  dlc;         /* Data Length Code (0–8 bytes)       */
    uint8_t  data[8];     /* Payload                            */
} CAN_Frame;

/* ─── Return Codes ──────────────────────────────────────────────────── */
typedef enum {
    MCP_OK    = 0,
    MCP_ERROR = 1,
    MCP_TIMEOUT = 2
} MCP_Status;

/* ─── Public API ────────────────────────────────────────────────────── */

/**
 * @brief  Initialise the MCP2515 at 500 kbps.
 * @param  hspi  Pointer to the HAL SPI handle used for this module.
 * @param  cs_port  GPIO port for the chip-select pin.
 * @param  cs_pin   GPIO pin number for chip-select.
 * @retval MCP_OK on success.
 */
MCP_Status MCP2515_Init(SPI_HandleTypeDef *hspi,
                        GPIO_TypeDef      *cs_port,
                        uint16_t           cs_pin);

/**
 * @brief  Send one CAN frame via TX Buffer 0.
 * @param  frame  Pointer to a populated CAN_Frame struct.
 * @retval MCP_OK on success.
 */
MCP_Status MCP2515_SendFrame(const CAN_Frame *frame);

/**
 * @brief  Check if a frame is waiting in RX Buffer 0.
 * @retval 1 if data available, 0 otherwise.
 */
uint8_t MCP2515_DataAvailable(void);

/**
 * @brief  Read one CAN frame from RX Buffer 0.
 * @param  frame  Pointer to a CAN_Frame to fill.
 * @retval MCP_OK on success.
 */
MCP_Status MCP2515_ReadFrame(CAN_Frame *frame);

/**
 * @brief  Set the operating mode (Normal / Config / Loopback / Listen).
 * @param  mode  One of MCP_MODE_* constants.
 */
MCP_Status MCP2515_SetMode(uint8_t mode);

/* Low-level helpers (also useful for debugging) */
void    MCP2515_Reset(void);
uint8_t MCP2515_ReadReg(uint8_t addr);
void    MCP2515_WriteReg(uint8_t addr, uint8_t val);
void    MCP2515_BitModify(uint8_t addr, uint8_t mask, uint8_t val);

#endif /* MCP2515_H */
