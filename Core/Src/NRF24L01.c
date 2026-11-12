
#include "main.h"
#include "NRF24L01.h"

#include <string.h>
#include <stdbool.h>

/* =====================================================
   HARDWARE CONFIGURATION
   ===================================================== */

extern SPI_HandleTypeDef hspi1;

#define NRF_SPI             (&hspi1)

#define NRF_CE_PORT         CE_GPIO_Port
#define NRF_CE_PIN          CE_Pin

#define NRF_CSN_PORT        CSN_GPIO_Port
#define NRF_CSN_PIN         CSN_Pin


/* =====================================================
   DEBUG VARIABLES
   ===================================================== */

volatile uint32_t nrf_spi_error_count = 0;

volatile uint8_t nrf_last_status = 0;
volatile uint8_t nrf_last_fifo = 0;


/* =====================================================
   INTERNAL FUNCTIONS
   ===================================================== */

static void NRF_CS_Low(void)
{
    HAL_GPIO_WritePin(
        NRF_CSN_PORT,
        NRF_CSN_PIN,
        GPIO_PIN_RESET
    );
}

static void NRF_CS_High(void)
{
    HAL_GPIO_WritePin(
        NRF_CSN_PORT,
        NRF_CSN_PIN,
        GPIO_PIN_SET
    );
}

void NRF24_CE_Enable(void)
{
    HAL_GPIO_WritePin(
        NRF_CE_PORT,
        NRF_CE_PIN,
        GPIO_PIN_SET
    );
}

void NRF24_CE_Disable(void)
{
    HAL_GPIO_WritePin(
        NRF_CE_PORT,
        NRF_CE_PIN,
        GPIO_PIN_RESET
    );
}


/* =====================================================
   SPI TRANSFER
   ===================================================== */

static uint8_t NRF_SPI_Transfer(
    uint8_t *tx,
    uint8_t *rx,
    uint16_t size,
    uint32_t timeout
)
{
    if (tx == NULL || rx == NULL || size == 0)
        return 0;

    HAL_StatusTypeDef result;

    result = HAL_SPI_TransmitReceive(
        NRF_SPI,
        tx,
        rx,
        size,
        timeout
    );

    if (result != HAL_OK)
    {
        nrf_spi_error_count++;
        return 0;
    }

    return 1;
}


/* =====================================================
   REGISTER READ
   ===================================================== */

static uint8_t NRF_ReadReg(
    uint8_t reg,
    uint8_t *value
)
{
    uint8_t tx[2];
    uint8_t rx[2];

    if (value == NULL)
        return 0;

    tx[0] = NRF_CMD_R_REGISTER |
            (reg & NRF_CMD_REGISTER_MASK);

    tx[1] = NRF_CMD_NOP;

    NRF_CS_Low();

    uint8_t result = NRF_SPI_Transfer(
        tx,
        rx,
        2,
        NRF24_SPI_TIMEOUT
    );

    NRF_CS_High();

    if (!result)
        return 0;

    nrf_last_status = rx[0];

    *value = rx[1];

    return 1;
}


/* =====================================================
   REGISTER WRITE
   ===================================================== */

static uint8_t NRF_WriteReg(
    uint8_t reg,
    uint8_t value
)
{
    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = NRF_CMD_W_REGISTER |
            (reg & NRF_CMD_REGISTER_MASK);

    tx[1] = value;

    NRF_CS_Low();

    uint8_t result = NRF_SPI_Transfer(
        tx,
        rx,
        2,
        NRF24_SPI_TIMEOUT
    );

    NRF_CS_High();

    if (!result)
        return 0;

    nrf_last_status = rx[0];

    return 1;
}


/* =====================================================
   MULTI REGISTER WRITE
   ===================================================== */

static uint8_t NRF_WriteMulti(
    uint8_t reg,
    uint8_t *data,
    uint8_t size
)
{
    uint8_t command;
    uint8_t rx[5];
    uint8_t status_rx = 0;

    if (data == NULL ||
        size == 0 ||
        size > 5)
        return 0;

    command = NRF_CMD_W_REGISTER |
              (reg & NRF_CMD_REGISTER_MASK);

    NRF_CS_Low();

    uint8_t result = NRF_SPI_Transfer(
        &command,
        &status_rx,
        1,
        NRF24_SPI_TIMEOUT
    );

    if (result)
    {
        nrf_last_status = status_rx;

        result = NRF_SPI_Transfer(
            data,
            rx,
            size,
            NRF24_SPI_TIMEOUT
        );
    }

    NRF_CS_High();

    return result;
}


/* =====================================================
   SEND COMMAND
   ===================================================== */

static uint8_t NRF_Command(uint8_t command)
{
    uint8_t rx = 0;

    NRF_CS_Low();

    uint8_t result = NRF_SPI_Transfer(
        &command,
        &rx,
        1,
        NRF24_SPI_TIMEOUT
    );

    NRF_CS_High();

    if (result)
        nrf_last_status = rx;

    return result;
}


/* =====================================================
   PUBLIC REGISTER READ
   ===================================================== */

uint8_t NRF24_ReadRegister(uint8_t reg)
{
    uint8_t value = 0;

    if (!NRF_ReadReg(reg, &value))
        return 0xFF;

    return value;
}


/* =====================================================
   FLUSH TX
   ===================================================== */

void NRF24_FlushTX(void)
{
    NRF_Command(NRF_CMD_FLUSH_TX);
}


/* =====================================================
   CONNECTION TEST
   ===================================================== */

uint8_t NRF24_CheckConnection(void)
{
    uint8_t original = 0;
    uint8_t test1 = 0;
    uint8_t test2 = 0;

    uint8_t result = 0;

    if (!NRF_ReadReg(NRF_REG_RF_CH, &original))
        return 0;

    if (!NRF_WriteReg(NRF_REG_RF_CH, 0x2A))
        goto restore;

    if (!NRF_ReadReg(NRF_REG_RF_CH, &test1))
        goto restore;

    if (!NRF_WriteReg(NRF_REG_RF_CH, 0x55))
        goto restore;

    if (!NRF_ReadReg(NRF_REG_RF_CH, &test2))
        goto restore;

    if (test1 == 0x2A && test2 == 0x55)
        result = 1;

restore:

    NRF_WriteReg(
        NRF_REG_RF_CH,
        original & 0x7F
    );

    return result;
}


/* =====================================================
   INITIALIZATION
   ===================================================== */

uint8_t NRF24_Init(void)
{
    NRF24_CE_Disable();

    NRF_CS_High();

    HAL_Delay(5);

    nrf_spi_error_count = 0;

    if (!NRF24_CheckConnection())
        return 0;

    /* Power down before configuration */

    if (!NRF_WriteReg(NRF_REG_CONFIG, 0x08))
        return 0;

    /* Disable auto acknowledgment */

    if (!NRF_WriteReg(NRF_REG_EN_AA, 0x00))
        return 0;

    /* Disable RX pipes */

    if (!NRF_WriteReg(NRF_REG_EN_RXADDR, 0x00))
        return 0;

    /* 5-byte address */

    if (!NRF_WriteReg(NRF_REG_SETUP_AW, 0x03))
        return 0;

    /* Disable automatic retransmission */

    if (!NRF_WriteReg(NRF_REG_SETUP_RETR, 0x00))
        return 0;

    /* RF Channel */

    if (!NRF_WriteReg(
        NRF_REG_RF_CH,
        NRF24_CHANNEL))
        return 0;

    /* 1Mbps / 0dBm */

    if (!NRF_WriteReg(NRF_REG_RF_SETUP, 0x06))
        return 0;

    /* Clear interrupt flags */

    if (!NRF_WriteReg(NRF_REG_STATUS, 0x70))
        return 0;

    /* Disable dynamic payload */

    if (!NRF_WriteReg(NRF_REG_DYNPD, 0x00))
        return 0;

    if (!NRF_WriteReg(NRF_REG_FEATURE, 0x00))
        return 0;

    /* Clear FIFOs */

    if (!NRF_Command(NRF_CMD_FLUSH_TX))
        return 0;

    if (!NRF_Command(NRF_CMD_FLUSH_RX))
        return 0;

    NRF24_CE_Disable();

    return 1;
}


/* =====================================================
   TRANSMITTER MODE
   ===================================================== */

uint8_t NRF24_TxMode(
    uint8_t *Address,
    uint8_t channel
)
{
    if (Address == NULL || channel > 125)
        return 0;

    NRF24_CE_Disable();

    if (!NRF_WriteReg(
        NRF_REG_RF_CH,
        channel))
        return 0;

    /* Set transmitter address */

    if (!NRF_WriteMulti(
        NRF_REG_TX_ADDR,
        Address,
        5))
        return 0;

    /*
       EN_CRC = 1
       PWR_UP = 1
       PRIM_RX = 0
    */

    if (!NRF_WriteReg(
        NRF_REG_CONFIG,
        NRF_CONFIG_EN_CRC |
        NRF_CONFIG_PWR_UP))
        return 0;

    /* Wait for power-up */

    HAL_Delay(2);

    NRF24_CE_Disable();

    return 1;
}


/* =====================================================
   TRANSMISSION
   ===================================================== */

uint8_t NRF24_Transmit(uint8_t *data)
{
    uint8_t tx[NRF24_PAYLOAD_SIZE];
    uint8_t rx[NRF24_PAYLOAD_SIZE];

    uint8_t command;
    uint8_t status_rx = 0;

    if (data == NULL)
        return 0;

    NRF24_CE_Disable();

    /* Clear old TX flags */

    if (!NRF_WriteReg(
        NRF_REG_STATUS,
        NRF_STATUS_TX_DS |
        NRF_STATUS_MAX_RT))
        return 0;

    /* Flush old payload */

    if (!NRF_Command(NRF_CMD_FLUSH_TX))
        return 0;

    /* Copy exactly 32 bytes */

    memcpy(
        tx,
        data,
        NRF24_PAYLOAD_SIZE
    );

    /* Load payload */

    command = NRF_CMD_W_TX_PAYLOAD;

    NRF_CS_Low();

    uint8_t result = NRF_SPI_Transfer(
        &command,
        &status_rx,
        1,
        NRF24_SPI_TIMEOUT
    );

    if (result)
    {
        nrf_last_status = status_rx;

        result = NRF_SPI_Transfer(
            tx,
            rx,
            NRF24_PAYLOAD_SIZE,
            NRF24_SPI_TIMEOUT
        );
    }

    NRF_CS_High();

    if (!result)
    {
        NRF_Command(NRF_CMD_FLUSH_TX);
        return 0;
    }

    /* Trigger TX */

    NRF24_CE_Enable();

    /*
       CE pulse must be >= 10us.
       Verify this delay for the actual MCU clock.
    */

    for (volatile uint32_t i = 0; i < 1000; i++)
    {
        __NOP();
    }

    NRF24_CE_Disable();

    /* Wait for TX_DS or MAX_RT */

    uint32_t start = HAL_GetTick();

    while ((uint32_t)(HAL_GetTick() - start)
           < NRF24_TX_TIMEOUT)
    {
        uint8_t status;

        if (!NRF_ReadReg(
            NRF_REG_STATUS,
            &status))
        {
            NRF_Command(NRF_CMD_FLUSH_TX);
            return 0;
        }

        if (status & NRF_STATUS_TX_DS)
        {
            NRF_WriteReg(
                NRF_REG_STATUS,
                NRF_STATUS_TX_DS
            );

            nrf_last_fifo =
                NRF24_ReadRegister(
                    NRF_REG_FIFO_STATUS
                );

            return 1;
        }

        if (status & NRF_STATUS_MAX_RT)
        {
            NRF_WriteReg(
                NRF_REG_STATUS,
                NRF_STATUS_MAX_RT
            );

            NRF_Command(NRF_CMD_FLUSH_TX);

            nrf_last_fifo =
                NRF24_ReadRegister(
                    NRF_REG_FIFO_STATUS
                );

            return 0;
        }
    }

    /* Timeout */

    NRF_Command(NRF_CMD_FLUSH_TX);

    nrf_last_fifo =
        NRF24_ReadRegister(
            NRF_REG_FIFO_STATUS
        );

    return 0;
}
