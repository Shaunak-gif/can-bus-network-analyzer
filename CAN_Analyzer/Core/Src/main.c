/*
 * main.c  –  STM32 #2  :  CAN Bus Network Analyzer
 *
 * Receives CAN frames from the ECU Simulator (or any real ECU),
 * decodes them, logs them, and streams results over UART to a PC.
 *
 * Terminal commands (type in serial monitor, 115200 baud):
 *   'L'  – dump the full in-RAM log
 *   'C'  – clear the log
 *   'R'  – toggle raw hex / decoded display
 *   'F'  – show current fault status
 *   'S'  – show statistics (frame count, IDs seen)
 *
 * ─── Pin connections (same as Transmitter board) ────────────────────
 *   MCP2515 VCC → 3.3V      SCK  → PA5
 *   MCP2515 GND → GND       MOSI → PA7
 *   CS  → PA4                MISO → PA6
 *   INT → PB0  (polled via status register, interrupt optional)
 *   UART TX → PA9   (connect to USB-TTL adapter)
 *   UART RX → PA10
 */

#include "main.h"
#include "mcp2515.h"
#include "can_decoder.h"
#include "can_logger.h"
#include "fault_detector.h"
#include <string.h>
#include <stdio.h>

/* ─── Peripheral handles ────────────────────────────────────────────── */
SPI_HandleTypeDef  hspi1;
UART_HandleTypeDef huart1;

/* ─── Private prototypes ────────────────────────────────────────────── */
static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_Init(void);
static void Print(const char *s);
static void PrintFrame(const CAN_Frame *raw, const DecodedFrame *dec,
                       uint32_t tick, uint8_t fault_flags);
static void PrintStats(void);
static void PrintHelp(void);
static void HandleCommand(uint8_t cmd);

/* ─── Runtime state ─────────────────────────────────────────────────── */
static uint8_t  raw_mode       = 0;     /* 0 = decoded, 1 = raw hex  */
static uint32_t frame_count    = 0;
static uint32_t fault_count    = 0;
static uint32_t last_fault_chk = 0;

/* Simple frame-rate counter: IDs and how many times each was seen */
#define MAX_UNIQUE_IDS 16
static struct { uint32_t id; uint32_t count; } id_stats[MAX_UNIQUE_IDS];
static uint8_t id_count = 0;

static void id_stats_add(uint32_t id)
{
    for (uint8_t i = 0; i < id_count; i++) {
        if (id_stats[i].id == id) { id_stats[i].count++; return; }
    }
    if (id_count < MAX_UNIQUE_IDS) {
        id_stats[id_count].id    = id;
        id_stats[id_count].count = 1;
        id_count++;
    }
}

/* ─── Main ──────────────────────────────────────────────────────────── */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI1_Init();
    MX_USART1_Init();

    CAN_Logger_Init();
    FaultDetector_Init();

    Print("\r\n");
    Print("╔══════════════════════════════════════╗\r\n");
    Print("║   CAN Bus Network Analyzer v1.0      ║\r\n");
    Print("║   STM32F103 + MCP2515  @500 kbps     ║\r\n");
    Print("╚══════════════════════════════════════╝\r\n");

    if (MCP2515_Init(&hspi1, GPIOA, GPIO_PIN_4) != MCP_OK) {
        Print("[ERROR] MCP2515 init failed! Check wiring.\r\n");
        while (1) {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            HAL_Delay(100);
        }
    }

    Print("[OK]   MCP2515 initialised at 500 kbps\r\n");
    PrintHelp();

    uint8_t uart_rx = 0;   /* non-blocking UART receive byte */

    while (1)
    {
        uint32_t now = HAL_GetTick();

        /* ── Receive CAN frames ─────────────────────────────────────── */
        if (MCP2515_DataAvailable())
        {
            CAN_Frame   raw;
            DecodedFrame dec;

            if (MCP2515_ReadFrame(&raw) == MCP_OK)
            {
                CAN_Decode(&raw, &dec);

                uint8_t fault_flags = FaultDetector_Update(&dec, &raw, now);
                CAN_Logger_Add(now, &raw, &dec);
                id_stats_add(raw.id);
                frame_count++;

                if (fault_flags) fault_count++;

                PrintFrame(&raw, &dec, now, fault_flags);

                /* Heartbeat LED on every received frame */
                HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            }
        }

        /* ── Periodic timeout fault check (every 200 ms) ────────────── */
        if (now - last_fault_chk >= 200)
        {
            last_fault_chk = now;
            uint8_t tf = FaultDetector_Check(now);
            if (tf) {
                char buf[64];
                snprintf(buf, sizeof(buf),
                         "[!FAULT] %s\r\n", FaultDetector_GetLastMsg());
                Print(buf);
                fault_count++;
            }
        }

        /* ── Non-blocking UART command handling ─────────────────────── */
        if (HAL_UART_Receive(&huart1, &uart_rx, 1, 0) == HAL_OK)
            HandleCommand(uart_rx);
    }
}

/* ─── Print frame to UART ───────────────────────────────────────────── */
static void PrintFrame(const CAN_Frame *raw, const DecodedFrame *dec,
                       uint32_t tick, uint8_t fault_flags)
{
    char buf[120];
    uint32_t sec  = tick / 1000;
    uint32_t msec = tick % 1000;

    if (raw_mode) {
        /* Level 1: raw hex dump */
        char hex[30];
        CAN_FormatRaw(raw, hex, sizeof(hex));
        snprintf(buf, sizeof(buf),
                 "[%4lu.%03lu] ID:0x%03lX DLC:%u DATA: %s\r\n",
                 (unsigned long)sec,
                 (unsigned long)msec,
                 (unsigned long)raw->id,
                 raw->dlc, hex);
    } else {
        /* Level 2: decoded */
        if (fault_flags & FAULT_FLAG_ECU_FRAME) {
            snprintf(buf, sizeof(buf),
                     "[%4lu.%03lu] *** %s ***\r\n",
                     (unsigned long)sec,
                     (unsigned long)msec,
                     dec->value_str);
        } else if (fault_flags) {
            snprintf(buf, sizeof(buf),
                     "[%4lu.%03lu] [RANGE ERR] %-14s = %s\r\n",
                     (unsigned long)sec,
                     (unsigned long)msec,
                     dec->signal_name, dec->value_str);
        } else {
            snprintf(buf, sizeof(buf),
                     "[%4lu.%03lu] %-14s = %s\r\n",
                     (unsigned long)sec,
                     (unsigned long)msec,
                     dec->signal_name, dec->value_str);
        }
    }
    Print(buf);
}

/* ─── Command handler ───────────────────────────────────────────────── */
static void HandleCommand(uint8_t cmd)
{
    switch (cmd) {
        case 'L': case 'l':
            CAN_Logger_Dump();
            break;
        case 'C': case 'c':
            CAN_Logger_Clear();
            frame_count = 0; fault_count = 0;
            id_count = 0;
            Print("[LOG] Cleared. Stats reset.\r\n");
            break;
        case 'R': case 'r':
            raw_mode = !raw_mode;
            Print(raw_mode ? "[MODE] Raw hex\r\n" : "[MODE] Decoded\r\n");
            break;
        case 'F': case 'f': {
            char buf[80];
            snprintf(buf, sizeof(buf), "[FAULT] %s\r\n",
                     FaultDetector_GetLastMsg());
            Print(buf);
            break;
        }
        case 'S': case 's':
            PrintStats();
            break;
        case 'H': case 'h': case '?':
            PrintHelp();
            break;
        default:
            break;
    }
}

static void PrintStats(void)
{
    char buf[80];
    Print("\r\n--- Statistics ---\r\n");
    snprintf(buf, sizeof(buf), "Total frames : %lu\r\n",
             (unsigned long)frame_count);
    Print(buf);
    snprintf(buf, sizeof(buf), "Fault events : %lu\r\n",
             (unsigned long)fault_count);
    Print(buf);
    Print("IDs seen:\r\n");
    for (uint8_t i = 0; i < id_count; i++) {
        snprintf(buf, sizeof(buf), "  0x%03lX  ×%lu\r\n",
                 (unsigned long)id_stats[i].id,
                 (unsigned long)id_stats[i].count);
        Print(buf);
    }
    Print("------------------\r\n\r\n");
}

static void PrintHelp(void)
{
    Print("\r\nCommands: L=Log dump  C=Clear  R=Toggle raw  "
          "F=Fault  S=Stats  H=Help\r\n\r\n");
}

/* ─── UART helper ───────────────────────────────────────────────────── */
static void Print(const char *s)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)s, strlen(s), HAL_MAX_DELAY);
}

/* ─── Peripheral inits (same as Transmitter) ────────────────────────── */

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState       = RCC_HSE_ON;
    osc.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    osc.PLL.PLLState   = RCC_PLL_ON;
    osc.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLMUL     = RCC_PLL_MUL9;
    HAL_RCC_OscConfig(&osc);
    clk.ClockType      = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                         RCC_CLOCKTYPE_PCLK1  | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2);
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    gpio.Pin = GPIO_PIN_13; gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &gpio);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    gpio.Pin = GPIO_PIN_4; gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);
}

static void MX_SPI1_Init(void)
{
    __HAL_RCC_SPI1_CLK_ENABLE();
    hspi1.Instance               = SPI1;
    hspi1.Init.Mode              = SPI_MODE_MASTER;
    hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase          = SPI_PHASE_1EDGE;
    hspi1.Init.NSS               = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    HAL_SPI_Init(&hspi1);

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_5 | GPIO_PIN_7; gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_6; gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);
}

static void MX_USART1_Init(void)
{
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_9; gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_10; gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);

    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 115200;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);
}

void Error_Handler(void)
{
    while (1) {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        HAL_Delay(100);
    }
}
