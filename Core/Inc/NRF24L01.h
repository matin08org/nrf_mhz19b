
#ifndef INC_NRF24L01_H_
#define INC_NRF24L01_H_

#include "main.h"
#include <stdint.h>

/* ================= CONFIGURATION ================= */

#define NRF24_CHANNEL          10U
#define NRF24_ADDRESS_WIDTH    5U
#define NRF24_PAYLOAD_SIZE     32U
#define NRF24_SPI_TIMEOUT      100U
#define NRF24_TX_TIMEOUT       100U

/* ================= REGISTER MAP ================= */

#define NRF_REG_CONFIG       0x00
#define NRF_REG_EN_AA        0x01
#define NRF_REG_EN_RXADDR    0x02
#define NRF_REG_SETUP_AW     0x03
#define NRF_REG_SETUP_RETR   0x04
#define NRF_REG_RF_CH        0x05
#define NRF_REG_RF_SETUP     0x06
#define NRF_REG_STATUS       0x07
#define NRF_REG_OBSERVE_TX   0x08
#define NRF_REG_FIFO_STATUS  0x17
#define NRF_REG_DYNPD        0x1C
#define NRF_REG_FEATURE      0x1D

#define NRF_REG_RX_ADDR_P0   0x0A
#define NRF_REG_TX_ADDR      0x10

#define NRF_REG_RX_PW_P0     0x11

/* ================= COMMANDS ================= */

#define NRF_CMD_R_REGISTER    0x00
#define NRF_CMD_W_REGISTER    0x20
#define NRF_CMD_REGISTER_MASK 0x1F

#define NRF_CMD_R_RX_PAYLOAD  0x61
#define NRF_CMD_W_TX_PAYLOAD  0xA0

#define NRF_CMD_FLUSH_TX      0xE1
#define NRF_CMD_FLUSH_RX      0xE2
#define NRF_CMD_NOP           0xFF

/* ================= STATUS BITS ================= */

#define NRF_STATUS_RX_DR      (1U << 6)
#define NRF_STATUS_TX_DS      (1U << 5)
#define NRF_STATUS_MAX_RT     (1U << 4)

#define NRF_CONFIG_EN_CRC     (1U << 3)
#define NRF_CONFIG_CRCO       (1U << 2)
#define NRF_CONFIG_PWR_UP     (1U << 1)
#define NRF_CONFIG_PRIM_RX    (1U << 0)

/* ================= DEBUG ================= */

extern volatile uint32_t nrf_spi_error_count;
extern volatile uint8_t nrf_last_status;
extern volatile uint8_t nrf_last_fifo;

/* ================= API ================= */

uint8_t NRF24_Init(void);

uint8_t NRF24_CheckConnection(void);

uint8_t NRF24_TxMode(uint8_t *Address, uint8_t channel);

uint8_t NRF24_Transmit(uint8_t *data);

void NRF24_CE_Enable(void);
void NRF24_CE_Disable(void);

uint8_t NRF24_ReadRegister(uint8_t reg);

void NRF24_FlushTX(void);

#endif
