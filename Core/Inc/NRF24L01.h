#ifndef INC_NRF24L01_H_
#define INC_NRF24L01_H_

#include "main.h"
#include <stdint.h>

/*
 * nRF24L01+ driver for STM32F030F4P6 / STM32F0 HAL.
 * Fixed 32-byte payload, 5-byte address, 1 Mbps, channel 10,
 * 2-byte CRC and Enhanced ShockBurst Auto-ACK.
 */

#define NRF24_CHANNEL              10U
#define NRF24_ADDRESS_WIDTH        5U
#define NRF24_PAYLOAD_SIZE         32U
#define NRF24_SPI_TIMEOUT_MS       20U
#define NRF24_TX_TIMEOUT_MS        100U

/* Register map */
#define NRF_REG_CONFIG             0x00U
#define NRF_REG_EN_AA              0x01U
#define NRF_REG_EN_RXADDR          0x02U
#define NRF_REG_SETUP_AW           0x03U
#define NRF_REG_SETUP_RETR         0x04U
#define NRF_REG_RF_CH              0x05U
#define NRF_REG_RF_SETUP           0x06U
#define NRF_REG_STATUS             0x07U
#define NRF_REG_OBSERVE_TX         0x08U
#define NRF_REG_RX_ADDR_P0         0x0AU
#define NRF_REG_TX_ADDR            0x10U
#define NRF_REG_RX_PW_P0            0x11U
#define NRF_REG_FIFO_STATUS        0x17U
#define NRF_REG_DYNPD              0x1CU
#define NRF_REG_FEATURE            0x1DU

/* SPI commands */
#define NRF_CMD_R_REGISTER         0x00U
#define NRF_CMD_W_REGISTER         0x20U
#define NRF_CMD_REGISTER_MASK      0x1FU
#define NRF_CMD_W_TX_PAYLOAD       0xA0U
#define NRF_CMD_FLUSH_TX           0xE1U
#define NRF_CMD_FLUSH_RX           0xE2U
#define NRF_CMD_NOP                0xFFU

/* STATUS register bits */
#define NRF_STATUS_RX_DR           (1U << 6)
#define NRF_STATUS_TX_DS           (1U << 5)
#define NRF_STATUS_MAX_RT          (1U << 4)

/* CONFIG register bits */
#define NRF_CONFIG_EN_CRC          (1U << 3)
#define NRF_CONFIG_CRCO            (1U << 2)
#define NRF_CONFIG_PWR_UP          (1U << 1)
#define NRF_CONFIG_PRIM_RX         (1U << 0)

/* Public result codes: 1 means the remote radio returned an ACK. */
#define NRF24_TX_NONE              0U
#define NRF24_TX_ACK               1U
#define NRF24_TX_MAX_RT            2U
#define NRF24_TX_SPI_ERROR         3U
#define NRF24_TX_TIMEOUT           4U
#define NRF24_TX_NOT_READY         5U
#define NRF24_TX_BAD_ARGUMENT      6U

/* Initialization progress / failure codes for debugger Watch. */
#define NRF24_INIT_IDLE            0U
#define NRF24_INIT_SPI_PROBE       1U
#define NRF24_INIT_CHIP_PROBE      2U
#define NRF24_INIT_REG_CONFIG      3U
#define NRF24_INIT_TX_CONFIG       4U
#define NRF24_INIT_READY           5U
#define NRF24_INIT_ERROR           0xFFU

#define NRF24_INIT_ERR_NONE        0U
#define NRF24_INIT_ERR_SPI         1U
#define NRF24_INIT_ERR_NO_DEVICE   2U
#define NRF24_INIT_ERR_REGISTER    3U
#define NRF24_INIT_ERR_ADDRESS     4U
#define NRF24_INIT_ERR_POWER_UP    5U

/* Live diagnostic values. All are debugger-Watch friendly. */
extern volatile uint32_t nrf_spi_transfer_count;
extern volatile uint32_t nrf_spi_error_count;
extern volatile uint32_t nrf_spi_consecutive_errors;
extern volatile uint32_t nrf_status_error_count;
extern volatile uint32_t nrf_status_consecutive_errors;
extern volatile uint32_t nrf_tx_attempt_count;
extern volatile uint32_t nrf_tx_ack_count;
extern volatile uint32_t nrf_tx_max_rt_count;
extern volatile uint32_t nrf_tx_timeout_count;

extern volatile uint8_t nrf_spi_last_transfer_ok;
extern volatile uint8_t nrf_spi_last_hal_status;
extern volatile uint32_t nrf_spi_last_hal_error;

extern volatile uint8_t nrf_detected;
extern volatile uint8_t nrf_config_ok;
extern volatile uint8_t nrf_init_stage;
extern volatile uint8_t nrf_init_error;

extern volatile uint8_t nrf_last_status;
extern volatile uint8_t nrf_last_fifo;
extern volatile uint8_t nrf_last_observe_tx;
extern volatile uint8_t nrf_last_tx_result;

/* API */
uint8_t NRF24_Init(void);
uint8_t NRF24_CheckConnection(void);
uint8_t NRF24_TxMode(const uint8_t address[NRF24_ADDRESS_WIDTH],
                     uint8_t channel);
uint8_t NRF24_Transmit(const uint8_t data[NRF24_PAYLOAD_SIZE]);

uint8_t NRF24_ReadRegisterChecked(uint8_t reg, uint8_t *value);
uint8_t NRF24_ReadRegister(uint8_t reg);
void NRF24_FlushTX(void);

void NRF24_CE_Enable(void);
void NRF24_CE_Disable(void);

#endif /* INC_NRF24L01_H_ */
