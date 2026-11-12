#include "main.h"
#include "NRF24L01.h"

#include <string.h>

/* Hardware mapping is defined by main.h / CubeMX:
 * CE  = PA3, CSN = PA4, SCK = PA5, MISO = PA6, MOSI = PA7.
 */
extern SPI_HandleTypeDef hspi1;

#define NRF_SPI                 (&hspi1)
#define NRF_CE_PORT             CE_GPIO_Port
#define NRF_CE_PIN              CE_Pin
#define NRF_CSN_PORT            CSN_GPIO_Port
#define NRF_CSN_PIN             CSN_Pin

#define NRF_CONFIG_POWERDOWN    (NRF_CONFIG_EN_CRC | NRF_CONFIG_CRCO)
#define NRF_CONFIG_TX_READY     (NRF_CONFIG_EN_CRC | NRF_CONFIG_CRCO | NRF_CONFIG_PWR_UP)
#define NRF_EN_AA_PIPE0         0x01U
#define NRF_EN_RX_PIPE0         0x01U
#define NRF_SETUP_RETR_VALUE    0x2FU /* 750 us retry delay, 15 retries */
#define NRF_RF_SETUP_1MBPS_0DBM 0x06U

volatile uint32_t nrf_spi_transfer_count = 0U;
volatile uint32_t nrf_spi_error_count = 0U;
volatile uint32_t nrf_spi_consecutive_errors = 0U;
volatile uint32_t nrf_status_error_count = 0U;
volatile uint32_t nrf_status_consecutive_errors = 0U;
volatile uint32_t nrf_tx_attempt_count = 0U;
volatile uint32_t nrf_tx_ack_count = 0U;
volatile uint32_t nrf_tx_max_rt_count = 0U;
volatile uint32_t nrf_tx_timeout_count = 0U;

volatile uint8_t nrf_spi_last_transfer_ok = 0U;
volatile uint8_t nrf_spi_last_hal_status = (uint8_t)HAL_OK;
volatile uint32_t nrf_spi_last_hal_error = HAL_SPI_ERROR_NONE;

volatile uint8_t nrf_detected = 0U;
volatile uint8_t nrf_config_ok = 0U;
volatile uint8_t nrf_init_stage = NRF24_INIT_IDLE;
volatile uint8_t nrf_init_error = NRF24_INIT_ERR_NONE;

volatile uint8_t nrf_last_status = 0U;
volatile uint8_t nrf_last_fifo = 0U;
volatile uint8_t nrf_last_observe_tx = 0U;
volatile uint8_t nrf_last_tx_result = NRF24_TX_NONE;

static uint8_t NRF_StatusValid(uint8_t status)
{
    /* STATUS bit 7 is reserved and must always read as zero.
     * A floating/high MISO line commonly returns 0xFF.
     */
    if ((status & 0x80U) != 0U)
    {
        nrf_status_error_count++;
        nrf_status_consecutive_errors++;
        return 0U;
    }

    nrf_status_consecutive_errors = 0U;
    return 1U;
}

static void NRF_CS_Low(void)
{
    HAL_GPIO_WritePin(NRF_CSN_PORT, NRF_CSN_PIN, GPIO_PIN_RESET);
}

static void NRF_CS_High(void)
{
    HAL_GPIO_WritePin(NRF_CSN_PORT, NRF_CSN_PIN, GPIO_PIN_SET);
}

void NRF24_CE_Enable(void)
{
    HAL_GPIO_WritePin(NRF_CE_PORT, NRF_CE_PIN, GPIO_PIN_SET);
}

void NRF24_CE_Disable(void)
{
    HAL_GPIO_WritePin(NRF_CE_PORT, NRF_CE_PIN, GPIO_PIN_RESET);
}

/* One HAL call equals one uninterrupted SPI byte stream. CSN is managed
 * manually and stays low across all bytes belonging to a single command.
 */
static uint8_t NRF_SPI_Transfer(uint8_t *tx,
                                uint8_t *rx,
                                uint16_t size,
                                uint32_t timeout_ms)
{
    HAL_StatusTypeDef hal_status;

    if ((tx == (uint8_t *)0) || (rx == (uint8_t *)0) || (size == 0U))
    {
        nrf_spi_last_transfer_ok = 0U;
        return 0U;
    }

    hal_status = HAL_SPI_TransmitReceive(NRF_SPI, tx, rx, size, timeout_ms);

    nrf_spi_transfer_count++;
    nrf_spi_last_hal_status = (uint8_t)hal_status;
    nrf_spi_last_hal_error = NRF_SPI->ErrorCode;

    if (hal_status != HAL_OK)
    {
        nrf_spi_error_count++;
        nrf_spi_consecutive_errors++;
        nrf_spi_last_transfer_ok = 0U;
        return 0U;
    }

    nrf_spi_consecutive_errors = 0U;
    nrf_spi_last_transfer_ok = 1U;
    return 1U;
}

static uint8_t NRF_ReadReg(uint8_t reg, uint8_t *value)
{
    uint8_t tx[2] = { (uint8_t)(NRF_CMD_R_REGISTER |
                                (reg & NRF_CMD_REGISTER_MASK)), NRF_CMD_NOP };
    uint8_t rx[2] = { 0U, 0U };
    uint8_t ok;

    if (value == (uint8_t *)0)
    {
        return 0U;
    }

    NRF_CS_Low();
    ok = NRF_SPI_Transfer(tx, rx, 2U, NRF24_SPI_TIMEOUT_MS);
    NRF_CS_High();

    if (ok != 0U)
    {
        if (NRF_StatusValid(rx[0]) == 0U)
        {
            return 0U;
        }
        *value = rx[1];
        nrf_last_status = rx[0];
    }

    return ok;
}

static uint8_t NRF_WriteReg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = { (uint8_t)(NRF_CMD_W_REGISTER |
                                (reg & NRF_CMD_REGISTER_MASK)), value };
    uint8_t rx[2] = { 0U, 0U };
    uint8_t ok;

    NRF_CS_Low();
    ok = NRF_SPI_Transfer(tx, rx, 2U, NRF24_SPI_TIMEOUT_MS);
    NRF_CS_High();

    if (ok != 0U)
    {
        if (NRF_StatusValid(rx[0]) == 0U)
        {
            return 0U;
        }
        nrf_last_status = rx[0];
    }

    return ok;
}

static uint8_t NRF_WriteMulti(uint8_t reg,
                              const uint8_t *data,
                              uint8_t size)
{
    uint8_t command;
    uint8_t status_rx = 0U;
    uint8_t rx[NRF24_ADDRESS_WIDTH] = { 0U, 0U, 0U, 0U, 0U };
    uint8_t ok;

    if ((data == (const uint8_t *)0) ||
        (size == 0U) ||
        (size > NRF24_ADDRESS_WIDTH))
    {
        return 0U;
    }

    command = (uint8_t)(NRF_CMD_W_REGISTER |
                        (reg & NRF_CMD_REGISTER_MASK));

    NRF_CS_Low();
    ok = NRF_SPI_Transfer(&command, &status_rx, 1U, NRF24_SPI_TIMEOUT_MS);

    if (ok != 0U)
    {
        if (NRF_StatusValid(status_rx) == 0U)
        {
            ok = 0U;
        }
        else
        {
            nrf_last_status = status_rx;
        }
    }

    if (ok != 0U)
    {
        /* HAL_SPI_TransmitReceive needs a writable TX buffer. The address
         * passed here is read-only, so copy it to a bounded local buffer.
         */
        {
            uint8_t tx_copy[NRF24_ADDRESS_WIDTH];
            uint8_t i;

            for (i = 0U; i < size; i++)
            {
                tx_copy[i] = data[i];
            }

            ok = NRF_SPI_Transfer(tx_copy, rx, size, NRF24_SPI_TIMEOUT_MS);
        }
    }

    NRF_CS_High();
    return ok;
}

static uint8_t NRF_ReadMulti(uint8_t reg, uint8_t *data, uint8_t size)
{
    uint8_t command;
    uint8_t status_rx = 0U;
    uint8_t dummy[NRF24_ADDRESS_WIDTH] = { NRF_CMD_NOP, NRF_CMD_NOP,
                                          NRF_CMD_NOP, NRF_CMD_NOP,
                                          NRF_CMD_NOP };
    uint8_t ok;

    if ((data == (uint8_t *)0) ||
        (size == 0U) ||
        (size > NRF24_ADDRESS_WIDTH))
    {
        return 0U;
    }

    command = (uint8_t)(NRF_CMD_R_REGISTER |
                        (reg & NRF_CMD_REGISTER_MASK));

    NRF_CS_Low();
    ok = NRF_SPI_Transfer(&command, &status_rx, 1U, NRF24_SPI_TIMEOUT_MS);

    if (ok != 0U)
    {
        if (NRF_StatusValid(status_rx) == 0U)
        {
            ok = 0U;
        }
        else
        {
            nrf_last_status = status_rx;
            ok = NRF_SPI_Transfer(dummy, data, size, NRF24_SPI_TIMEOUT_MS);
        }
    }

    NRF_CS_High();
    return ok;
}

static uint8_t NRF_Command(uint8_t command)
{
    uint8_t rx = 0U;
    uint8_t ok;

    NRF_CS_Low();
    ok = NRF_SPI_Transfer(&command, &rx, 1U, NRF24_SPI_TIMEOUT_MS);
    NRF_CS_High();

    if (ok != 0U)
    {
        if (NRF_StatusValid(rx) == 0U)
        {
            return 0U;
        }
        nrf_last_status = rx;
    }

    return ok;
}

static uint8_t NRF_VerifyRegister(uint8_t reg, uint8_t expected)
{
    uint8_t actual = 0U;

    if (NRF_ReadReg(reg, &actual) == 0U)
    {
        return 0U;
    }

    return (actual == expected) ? 1U : 0U;
}

uint8_t NRF24_ReadRegisterChecked(uint8_t reg, uint8_t *value)
{
    return NRF_ReadReg(reg, value);
}

uint8_t NRF24_ReadRegister(uint8_t reg)
{
    uint8_t value = 0xFFU;

    if (NRF_ReadReg(reg, &value) == 0U)
    {
        return 0xFFU;
    }

    return value;
}

void NRF24_FlushTX(void)
{
    (void)NRF_Command(NRF_CMD_FLUSH_TX);
}

/* A writable register test is used because nRF24L01+ has no WHO_AM_I
 * register. RF_CH is tested with two different values and restored.
 */
uint8_t NRF24_CheckConnection(void)
{
    uint8_t original = 0U;
    uint8_t value_a = 0U;
    uint8_t value_b = 0U;
    uint8_t restore_ok;

    if (NRF_ReadReg(NRF_REG_RF_CH, &original) == 0U)
    {
        return 0U;
    }

    if (NRF_WriteReg(NRF_REG_RF_CH, 0x2AU) == 0U)
    {
        goto restore;
    }

    if (NRF_ReadReg(NRF_REG_RF_CH, &value_a) == 0U)
    {
        goto restore;
    }

    if (NRF_WriteReg(NRF_REG_RF_CH, 0x55U) == 0U)
    {
        goto restore;
    }

    if (NRF_ReadReg(NRF_REG_RF_CH, &value_b) == 0U)
    {
        goto restore;
    }

    if ((value_a == 0x2AU) && (value_b == 0x55U))
    {
        restore_ok = NRF_WriteReg(NRF_REG_RF_CH,
                                  (uint8_t)(original & 0x7FU));
        return restore_ok;
    }

restore:
    (void)NRF_WriteReg(NRF_REG_RF_CH, (uint8_t)(original & 0x7FU));
    return 0U;
}

uint8_t NRF24_Init(void)
{
    nrf_detected = 0U;
    nrf_config_ok = 0U;
    nrf_init_error = NRF24_INIT_ERR_NONE;
    nrf_last_tx_result = NRF24_TX_NONE;

    NRF24_CE_Disable();
    NRF_CS_High();
    HAL_Delay(5U);

    nrf_init_stage = NRF24_INIT_SPI_PROBE;

    /* This confirms the HAL transaction completes. The RF_CH read/write
     * test below is the actual chip + MISO/SCK/MOSI/CSN communication test.
     */
    if (NRF_Command(NRF_CMD_NOP) == 0U)
    {
        nrf_init_error = (nrf_spi_last_transfer_ok != 0U) ?
                         NRF24_INIT_ERR_NO_DEVICE : NRF24_INIT_ERR_SPI;
        nrf_init_stage = NRF24_INIT_ERROR;
        return 0U;
    }

    nrf_init_stage = NRF24_INIT_CHIP_PROBE;

    if (NRF24_CheckConnection() == 0U)
    {
        nrf_init_error = NRF24_INIT_ERR_NO_DEVICE;
        nrf_init_stage = NRF24_INIT_ERROR;
        return 0U;
    }

    nrf_detected = 1U;
    nrf_init_stage = NRF24_INIT_REG_CONFIG;

    if ((NRF_WriteReg(NRF_REG_CONFIG, NRF_CONFIG_POWERDOWN) == 0U) ||
        (NRF_WriteReg(NRF_REG_EN_AA, NRF_EN_AA_PIPE0) == 0U) ||
        (NRF_WriteReg(NRF_REG_EN_RXADDR, NRF_EN_RX_PIPE0) == 0U) ||
        (NRF_WriteReg(NRF_REG_SETUP_AW, 0x03U) == 0U) ||
        (NRF_WriteReg(NRF_REG_SETUP_RETR, NRF_SETUP_RETR_VALUE) == 0U) ||
        (NRF_WriteReg(NRF_REG_RF_CH, NRF24_CHANNEL) == 0U) ||
        (NRF_WriteReg(NRF_REG_RF_SETUP, NRF_RF_SETUP_1MBPS_0DBM) == 0U) ||
        (NRF_WriteReg(NRF_REG_RX_PW_P0, NRF24_PAYLOAD_SIZE) == 0U) ||
        (NRF_WriteReg(NRF_REG_DYNPD, 0x00U) == 0U) ||
        (NRF_WriteReg(NRF_REG_FEATURE, 0x00U) == 0U) ||
        (NRF_WriteReg(NRF_REG_STATUS, 0x70U) == 0U) ||
        (NRF_Command(NRF_CMD_FLUSH_TX) == 0U) ||
        (NRF_Command(NRF_CMD_FLUSH_RX) == 0U))
    {
        nrf_init_error = NRF24_INIT_ERR_REGISTER;
        nrf_init_stage = NRF24_INIT_ERROR;
        return 0U;
    }

    if ((NRF_VerifyRegister(NRF_REG_CONFIG, NRF_CONFIG_POWERDOWN) == 0U) ||
        (NRF_VerifyRegister(NRF_REG_EN_AA, NRF_EN_AA_PIPE0) == 0U) ||
        (NRF_VerifyRegister(NRF_REG_EN_RXADDR, NRF_EN_RX_PIPE0) == 0U) ||
        (NRF_VerifyRegister(NRF_REG_SETUP_AW, 0x03U) == 0U) ||
        (NRF_VerifyRegister(NRF_REG_SETUP_RETR, NRF_SETUP_RETR_VALUE) == 0U) ||
        (NRF_VerifyRegister(NRF_REG_RF_CH, NRF24_CHANNEL) == 0U) ||
        (NRF_VerifyRegister(NRF_REG_RF_SETUP, NRF_RF_SETUP_1MBPS_0DBM) == 0U) ||
        (NRF_VerifyRegister(NRF_REG_RX_PW_P0, NRF24_PAYLOAD_SIZE) == 0U) ||
        (NRF_VerifyRegister(NRF_REG_DYNPD, 0x00U) == 0U))
    {
        nrf_init_error = NRF24_INIT_ERR_REGISTER;
        nrf_init_stage = NRF24_INIT_ERROR;
        return 0U;
    }

    nrf_config_ok = 1U;
    nrf_init_stage = NRF24_INIT_TX_CONFIG;
    return 1U;
}

uint8_t NRF24_TxMode(const uint8_t address[NRF24_ADDRESS_WIDTH],
                     uint8_t channel)
{
    uint8_t verify_address[NRF24_ADDRESS_WIDTH];
    uint8_t verify_config = 0U;

    if ((address == (const uint8_t *)0) || (channel > 125U))
    {
        nrf_init_error = NRF24_INIT_ERR_ADDRESS;
        nrf_init_stage = NRF24_INIT_ERROR;
        nrf_config_ok = 0U;
        return 0U;
    }

    NRF24_CE_Disable();

    if ((NRF_WriteReg(NRF_REG_RF_CH, channel) == 0U) ||
        (NRF_WriteMulti(NRF_REG_TX_ADDR, address, NRF24_ADDRESS_WIDTH) == 0U) ||
        (NRF_WriteMulti(NRF_REG_RX_ADDR_P0, address, NRF24_ADDRESS_WIDTH) == 0U))
    {
        nrf_init_error = NRF24_INIT_ERR_REGISTER;
        nrf_init_stage = NRF24_INIT_ERROR;
        nrf_config_ok = 0U;
        return 0U;
    }

    if ((NRF_ReadMulti(NRF_REG_TX_ADDR, verify_address, NRF24_ADDRESS_WIDTH) == 0U) ||
        (memcmp(verify_address, address, NRF24_ADDRESS_WIDTH) != 0))
    {
        nrf_init_error = NRF24_INIT_ERR_ADDRESS;
        nrf_init_stage = NRF24_INIT_ERROR;
        nrf_config_ok = 0U;
        return 0U;
    }

    if ((NRF_ReadMulti(NRF_REG_RX_ADDR_P0, verify_address, NRF24_ADDRESS_WIDTH) == 0U) ||
        (memcmp(verify_address, address, NRF24_ADDRESS_WIDTH) != 0))
    {
        nrf_init_error = NRF24_INIT_ERR_ADDRESS;
        nrf_init_stage = NRF24_INIT_ERROR;
        nrf_config_ok = 0U;
        return 0U;
    }

    if ((NRF_WriteReg(NRF_REG_CONFIG, NRF_CONFIG_TX_READY) == 0U))
    {
        nrf_init_error = NRF24_INIT_ERR_POWER_UP;
        nrf_init_stage = NRF24_INIT_ERROR;
        nrf_config_ok = 0U;
        return 0U;
    }

    /* nRF24L01+ requires up to 1.5 ms from PWR_UP=0 to standby.
     * 2 ms gives margin before CE is pulsed.
     */
    HAL_Delay(2U);

    if (NRF_ReadReg(NRF_REG_CONFIG, &verify_config) == 0U)
    {
        nrf_init_error = NRF24_INIT_ERR_SPI;
        nrf_init_stage = NRF24_INIT_ERROR;
        nrf_config_ok = 0U;
        return 0U;
    }

    if (verify_config != NRF_CONFIG_TX_READY)
    {
        nrf_init_error = NRF24_INIT_ERR_POWER_UP;
        nrf_init_stage = NRF24_INIT_ERROR;
        nrf_config_ok = 0U;
        return 0U;
    }

    if ((NRF_VerifyRegister(NRF_REG_RF_CH, channel) == 0U) ||
        (NRF_VerifyRegister(NRF_REG_EN_AA, NRF_EN_AA_PIPE0) == 0U) ||
        (NRF_VerifyRegister(NRF_REG_EN_RXADDR, NRF_EN_RX_PIPE0) == 0U) ||
        (NRF_VerifyRegister(NRF_REG_SETUP_RETR, NRF_SETUP_RETR_VALUE) == 0U))
    {
        nrf_init_error = NRF24_INIT_ERR_REGISTER;
        nrf_init_stage = NRF24_INIT_ERROR;
        nrf_config_ok = 0U;
        return 0U;
    }

    nrf_config_ok = 1U;
    nrf_init_error = NRF24_INIT_ERR_NONE;
    nrf_init_stage = NRF24_INIT_READY;
    return 1U;
}

uint8_t NRF24_Transmit(const uint8_t data[NRF24_PAYLOAD_SIZE])
{
    uint8_t command = NRF_CMD_W_TX_PAYLOAD;
    uint8_t command_rx = 0U;
    uint8_t tx_payload[NRF24_PAYLOAD_SIZE];
    uint8_t payload_rx[NRF24_PAYLOAD_SIZE];
    uint8_t reg_value = 0U;
    uint8_t status = 0U;
    uint8_t ok;
    uint32_t start_tick;

    if (data == (const uint8_t *)0)
    {
        nrf_last_tx_result = NRF24_TX_BAD_ARGUMENT;
        return 0U;
    }

    if ((nrf_detected == 0U) || (nrf_config_ok == 0U))
    {
        nrf_last_tx_result = NRF24_TX_NOT_READY;
        return 0U;
    }

    nrf_tx_attempt_count++;
    nrf_last_tx_result = NRF24_TX_NONE;
    NRF24_CE_Disable();

    /* Clear old IRQ flags and any payload left after MAX_RT. */
    if ((NRF_WriteReg(NRF_REG_STATUS, 0x70U) == 0U) ||
        (NRF_Command(NRF_CMD_FLUSH_TX) == 0U))
    {
        nrf_last_tx_result = NRF24_TX_SPI_ERROR;
        return 0U;
    }

    memcpy(tx_payload, data, NRF24_PAYLOAD_SIZE);

    NRF_CS_Low();
    ok = NRF_SPI_Transfer(&command, &command_rx, 1U, NRF24_SPI_TIMEOUT_MS);

    if (ok != 0U)
    {
        nrf_last_status = command_rx;
        if (NRF_StatusValid(command_rx) == 0U)
        {
            ok = 0U;
        }
        else
        {
            ok = NRF_SPI_Transfer(tx_payload,
                                  payload_rx,
                                  NRF24_PAYLOAD_SIZE,
                                  NRF24_SPI_TIMEOUT_MS);
        }
    }

    NRF_CS_High();

    if (ok == 0U)
    {
        nrf_last_tx_result = NRF24_TX_SPI_ERROR;
        return 0U;
    }

    /* CE pulse > 10 us. 1 ms is deliberately used instead of a compiler-
     * dependent software loop, and transmits exactly one FIFO payload.
     */
    NRF24_CE_Enable();
    HAL_Delay(1U);
    NRF24_CE_Disable();

    start_tick = HAL_GetTick();

    while ((uint32_t)(HAL_GetTick() - start_tick) < NRF24_TX_TIMEOUT_MS)
    {
        if (NRF_ReadReg(NRF_REG_STATUS, &status) == 0U)
        {
            (void)NRF_Command(NRF_CMD_FLUSH_TX);
            nrf_last_tx_result = NRF24_TX_SPI_ERROR;
            return 0U;
        }

        nrf_last_status = status;

        if ((status & NRF_STATUS_TX_DS) != 0U)
        {
            (void)NRF_WriteReg(NRF_REG_STATUS, NRF_STATUS_TX_DS);
            if (NRF_ReadReg(NRF_REG_OBSERVE_TX, &reg_value) != 0U)
            {
                nrf_last_observe_tx = reg_value;
            }
            if (NRF_ReadReg(NRF_REG_FIFO_STATUS, &reg_value) != 0U)
            {
                nrf_last_fifo = reg_value;
            }

            nrf_tx_ack_count++;
            nrf_last_tx_result = NRF24_TX_ACK;
            return 1U;
        }

        if ((status & NRF_STATUS_MAX_RT) != 0U)
        {
            (void)NRF_WriteReg(NRF_REG_STATUS, NRF_STATUS_MAX_RT);
            (void)NRF_Command(NRF_CMD_FLUSH_TX);
            if (NRF_ReadReg(NRF_REG_OBSERVE_TX, &reg_value) != 0U)
            {
                nrf_last_observe_tx = reg_value;
            }
            if (NRF_ReadReg(NRF_REG_FIFO_STATUS, &reg_value) != 0U)
            {
                nrf_last_fifo = reg_value;
            }

            nrf_tx_max_rt_count++;
            nrf_last_tx_result = NRF24_TX_MAX_RT;
            return 0U;
        }

        HAL_Delay(1U);
    }

    (void)NRF_Command(NRF_CMD_FLUSH_TX);
    if (NRF_ReadReg(NRF_REG_OBSERVE_TX, &reg_value) != 0U)
    {
        nrf_last_observe_tx = reg_value;
    }
    if (NRF_ReadReg(NRF_REG_FIFO_STATUS, &reg_value) != 0U)
    {
        nrf_last_fifo = reg_value;
    }

    nrf_tx_timeout_count++;
    nrf_last_tx_result = NRF24_TX_TIMEOUT;
    return 0U;
}
