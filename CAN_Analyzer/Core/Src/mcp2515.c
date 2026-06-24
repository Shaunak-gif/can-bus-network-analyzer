/*
 * mcp2515.c
 * MCP2515 CAN Controller Driver Implementation
 *
 * Wiring (SPI1 default pins on STM32F103C8T6 "Blue Pill"):
 *   MCP2515  ←→  STM32
 *   VCC      ←→  3.3V
 *   GND      ←→  GND
 *   SCK      ←→  PA5
 *   SI (MOSI)←→  PA7
 *   SO (MISO)←→  PA6
 *   CS       ←→  PA4  (configurable)
 *   INT      ←→  PB0  (configurable, optional polling mode)
 */

#include "mcp2515.h"
#include <string.h>

/* ─── Private state ─────────────────────────────────────────────────── */
static SPI_HandleTypeDef *_hspi     = NULL;
static GPIO_TypeDef      *_cs_port  = NULL;
static uint16_t           _cs_pin   = 0;

/* ─── CS helpers ────────────────────────────────────────────────────── */
static inline void CS_Low(void)  { HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_RESET); }
static inline void CS_High(void) { HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_SET);   }

/* ─── Low-level SPI byte exchange ───────────────────────────────────── */
static uint8_t SPI_Transfer(uint8_t byte)
{
    uint8_t rx = 0;
    HAL_SPI_TransmitReceive(_hspi, &byte, &rx, 1, HAL_MAX_DELAY);
    return rx;
}

/* ─── Register read/write ───────────────────────────────────────────── */
uint8_t MCP2515_ReadReg(uint8_t addr)
{
    uint8_t val;
    CS_Low();
    SPI_Transfer(MCP_READ);
    SPI_Transfer(addr);
    val = SPI_Transfer(0x00);
    CS_High();
    return val;
}

void MCP2515_WriteReg(uint8_t addr, uint8_t val)
{
    CS_Low();
    SPI_Transfer(MCP_WRITE);
    SPI_Transfer(addr);
    SPI_Transfer(val);
    CS_High();
}

void MCP2515_BitModify(uint8_t addr, uint8_t mask, uint8_t val)
{
    CS_Low();
    SPI_Transfer(MCP_BIT_MODIFY);
    SPI_Transfer(addr);
    SPI_Transfer(mask);
    SPI_Transfer(val);
    CS_High();
}

/* ─── Reset ─────────────────────────────────────────────────────────── */
void MCP2515_Reset(void)
{
    CS_Low();
    SPI_Transfer(MCP_RESET);
    CS_High();
    HAL_Delay(10);   /* allow internal reset to complete */
}

/* ─── Mode ──────────────────────────────────────────────────────────── */
MCP_Status MCP2515_SetMode(uint8_t mode)
{
    MCP2515_BitModify(MCP_CANCTRL, 0xE0, mode);
    HAL_Delay(10);

    uint8_t status = MCP2515_ReadReg(MCP_CANSTAT) & 0xE0;
    return (status == mode) ? MCP_OK : MCP_ERROR;
}

/* ─── Initialisation ────────────────────────────────────────────────── */
MCP_Status MCP2515_Init(SPI_HandleTypeDef *hspi,
                        GPIO_TypeDef      *cs_port,
                        uint16_t           cs_pin)
{
    _hspi    = hspi;
    _cs_port = cs_port;
    _cs_pin  = cs_pin;

    CS_High();   /* deselect before reset */

    /* 1. Hardware reset */
    MCP2515_Reset();

    /* 2. Enter Configuration mode */
    if (MCP2515_SetMode(MCP_MODE_CONFIG) != MCP_OK)
        return MCP_ERROR;

    /* 3. Baud rate: 500 kbps with 16 MHz crystal
     *
     *  CNF1: SJW=1TQ, BRP=0  → TQ = 125 ns
     *  CNF2: BTLMODE=1, SAM=0, PHSEG1=7TQ, PRSEG=1TQ
     *  CNF3: PHSEG2=2TQ
     *  Total bit time = 1+1+7+2 = 11 TQ? No:
     *  Sync=1, Prop=1, PS1=7, PS2=2 → 11 TQ × 125 ns ≈ 1/727 kbps
     *  Tuned values below give clean 500 kbps:
     */
    MCP2515_WriteReg(MCP_CNF1, 0x00);   /* BRP=0, SJW=1TQ          */
    MCP2515_WriteReg(MCP_CNF2, 0xF0);   /* BTLMODE, PS1=8TQ,PROP=1 */
    MCP2515_WriteReg(MCP_CNF3, 0x86);   /* PS2=7TQ, WAKFIL enabled  */

    /*
     * Practical note: these values target 500 kbps with a 16 MHz crystal.
     * If your MCP2515 module has an 8 MHz crystal (common on cheap boards),
     * change to: CNF1=0x00, CNF2=0xB1, CNF3=0x85 for 500 kbps,
     * or          CNF1=0x01, CNF2=0xB1, CNF3=0x85 for 250 kbps.
     */

    /* 4. RX Buffer 0: accept all messages (masks/filters disabled) */
    MCP2515_WriteReg(MCP_RXB0CTRL, 0x60);  /* RXM=11: receive any */

    /* 5. Enable RX interrupt on INT pin (optional if polling) */
    MCP2515_WriteReg(MCP_CANINTE, 0x01);   /* RX0IE */

    /* 6. Clear interrupt flags */
    MCP2515_WriteReg(MCP_CANINTF, 0x00);

    /* 7. Normal operating mode */
    return MCP2515_SetMode(MCP_MODE_NORMAL);
}

/* ─── Send Frame ────────────────────────────────────────────────────── */
MCP_Status MCP2515_SendFrame(const CAN_Frame *frame)
{
    if (!frame || frame->dlc > 8) return MCP_ERROR;

    /* Standard 11-bit ID → SIDH / SIDL */
    uint8_t sidh = (uint8_t)((frame->id >> 3) & 0xFF);
    uint8_t sidl = (uint8_t)((frame->id & 0x07) << 5);

    /* Load TX Buffer 0 */
    CS_Low();
    SPI_Transfer(MCP_WRITE);
    SPI_Transfer(MCP_TXB0SIDH);
    SPI_Transfer(sidh);
    SPI_Transfer(sidl);
    SPI_Transfer(0x00);   /* TXB0EID8 (not used for standard frames) */
    SPI_Transfer(0x00);   /* TXB0EID0 */
    SPI_Transfer(frame->dlc & 0x0F);
    for (uint8_t i = 0; i < frame->dlc; i++)
        SPI_Transfer(frame->data[i]);
    CS_High();

    /* Request transmission */
    CS_Low();
    SPI_Transfer(MCP_RTS_TX0);
    CS_High();

    /* Wait for TX complete (TXREQ bit clears) – with timeout */
    uint32_t t = HAL_GetTick();
    while (MCP2515_ReadReg(MCP_TXB0CTRL) & 0x08) {
        if (HAL_GetTick() - t > 100) return MCP_TIMEOUT;
    }
    return MCP_OK;
}

/* ─── Check data available ──────────────────────────────────────────── */
uint8_t MCP2515_DataAvailable(void)
{
    CS_Low();
    SPI_Transfer(MCP_READ_STATUS);
    uint8_t status = SPI_Transfer(0x00);
    CS_High();
    return (status & MCP_STAT_RX0IF) ? 1 : 0;
}

/* ─── Read Frame ────────────────────────────────────────────────────── */
MCP_Status MCP2515_ReadFrame(CAN_Frame *frame)
{
    if (!frame) return MCP_ERROR;

    /* Read RX Buffer 0 starting at SIDH */
    CS_Low();
    SPI_Transfer(MCP_READ_RX0);    /* auto-increments from SIDH */
    uint8_t sidh = SPI_Transfer(0x00);
    uint8_t sidl = SPI_Transfer(0x00);
    SPI_Transfer(0x00);            /* EID8 – ignore */
    SPI_Transfer(0x00);            /* EID0 – ignore */
    uint8_t dlc  = SPI_Transfer(0x00) & 0x0F;

    frame->id  = ((uint32_t)sidh << 3) | ((sidl >> 5) & 0x07);
    frame->dlc = dlc;

    for (uint8_t i = 0; i < dlc && i < 8; i++)
        frame->data[i] = SPI_Transfer(0x00);

    CS_High();

    /* Clear RX0 interrupt flag */
    MCP2515_BitModify(MCP_CANINTF, 0x01, 0x00);

    return MCP_OK;
}
