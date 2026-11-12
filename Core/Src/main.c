
/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    main.c
 * @brief   CO2_Node - MH-Z19 + NRF24L01
 * @version 3.0
 ******************************************************************************
 */
/* USER CODE END Header */

#include "main.h"
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "NRF24L01.h"
#include "mhz19.h"

/* USER CODE END Includes */


/* =========================================================
   GLOBAL VARIABLES
   ========================================================= */

/* USER CODE BEGIN PV */

/* ================= MH-Z19 ================= */

MHZ19_Handle co2_sensor;


/* ================= NRF24 ================= */

uint8_t txAddress[5] =
{
    0xEE, 0xDD, 0xCC, 0xBB, 0xAA
};

uint8_t txData[32];


/* ================= SENSOR DATA ================= */

static uint16_t report_ppm = 0;

static MHZ19_Status report_status =
    MHZ19_STATUS_WARMING_UP;

static uint8_t request_pending = 0;
static uint8_t report_valid = 0;


/* ================= TIMING ================= */

#define CO2_REQUEST_INTERVAL_MS  2000U
#define NRF_REPORT_INTERVAL_MS   2000U

static uint32_t last_request_tick = 0U;
static uint32_t last_report_tick = 0U;
static uint32_t last_nrf_init_tick = 0U;

#define NRF_INIT_RETRY_INTERVAL_MS 1000U


/* ================= NRF STATUS ================= */

static uint8_t nrf_initialized = 0;

static uint8_t is_transmit = 0;
static uint8_t spi_status = 0;


/* ================= WATCH VARIABLES ================= */

/* General */
volatile uint32_t watch_counter = 0U;
volatile uint32_t watch_millis = 0U;
volatile uint32_t watch_tx_count = 0U;
volatile uint32_t watch_tx_fail_count = 0U;

/* SPI: config OK is separate from a completed HAL transaction. */
volatile uint8_t watch_spi_config_ok = 0U;
volatile uint8_t watch_spi_hal_ready = 0U;
volatile uint8_t watch_spi_last_transfer_ok = 0U;
volatile uint8_t watch_spi_last_hal_status = 0U;
volatile uint32_t watch_spi_hal_error = 0U;
volatile uint32_t watch_spi_transfers = 0U;
volatile uint32_t watch_spi_errors = 0U;
volatile uint32_t watch_nrf_status_errors = 0U;
volatile uint32_t watch_nrf_status_error_streak = 0U;

/* nRF24: detection/configuration are local; LINK_ACK requires a receiver. */
volatile uint8_t watch_nrf_initialized = 0U;
volatile uint8_t watch_nrf_detected = 0U;
volatile uint8_t watch_nrf_config_ok = 0U;
volatile uint8_t watch_nrf_init_stage = 0U;
volatile uint8_t watch_nrf_init_error = 0U;
volatile uint8_t watch_nrf_link_ack = 0U;
volatile uint8_t watch_nrf_tx_result = 0U;
volatile uint8_t watch_nrf_status = 0U;
volatile uint8_t watch_nrf_fifo = 0U;
volatile uint8_t watch_nrf_observe_tx = 0U;
volatile uint8_t watch_nrf_reg_status = 0U;
volatile uint8_t watch_nrf_reg_config = 0U;
volatile uint8_t watch_nrf_reg_rf_ch = 0U;
volatile uint8_t watch_nrf_reg_en_aa = 0U;
volatile uint8_t watch_nrf_reg_rf_setup = 0U;
volatile uint8_t watch_nrf_reg_setup_retr = 0U;
volatile uint8_t watch_nrf_reg_fifo_status = 0U;
volatile uint32_t watch_nrf_tx_attempts = 0U;
volatile uint32_t watch_nrf_tx_acks = 0U;
volatile uint32_t watch_nrf_tx_max_rt = 0U;
volatile uint32_t watch_nrf_tx_timeouts = 0U;

/* Set watch_nrf_test_request=1 in the debugger to send one fixed test frame. */
volatile uint8_t watch_nrf_test_request = 0U;
volatile uint8_t watch_nrf_test_done = 0U;
volatile uint8_t watch_nrf_test_result = 0U;

/* Sensor */
volatile uint16_t watch_co2 = 0U;
volatile uint8_t watch_sensor_ready = 0U;
volatile uint8_t watch_sensor_busy = 0U;
volatile uint8_t watch_request_pending = 0U;
volatile uint8_t watch_report_valid = 0U;
volatile uint8_t watch_sensor_status = 0U;
volatile uint32_t watch_request_count = 0U;
volatile uint32_t watch_response_count = 0U;

/* USER CODE END PV */


/* =========================================================
   FUNCTION PROTOTYPES
   ========================================================= */

void SystemClock_Config(void);


/* USER CODE BEGIN PFP */

static void CO2_Process(void);

static void NRF_Process(void);

static void Watch_Update(void);

static const char *MHZ19_GetErrorText(
    MHZ19_Status status
);

/* USER CODE END PFP */


/* =========================================================
   UART CALLBACKS
   ========================================================= */

/* USER CODE BEGIN 0 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    MHZ19_UART_RxCpltCallback(
        &co2_sensor,
        huart
    );
}


void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    MHZ19_UART_ErrorCallback(
        &co2_sensor,
        huart
    );
}


/* =========================================================
   ERROR TEXT
   ========================================================= */

static const char *MHZ19_GetErrorText(
    MHZ19_Status status
)
{
    switch (status)
    {
        case MHZ19_STATUS_TIMEOUT:
            return "TIMEOUT";

        case MHZ19_STATUS_UART_ERROR:
            return "UART_ERROR";

        case MHZ19_STATUS_BAD_HEADER:
            return "BAD_HEADER";

        case MHZ19_STATUS_BAD_CHECKSUM:
            return "BAD_CHECKSUM";

        case MHZ19_STATUS_WARMING_UP:
            return "WARMUP";

        case MHZ19_STATUS_BUSY:
            return "BUSY";

        case MHZ19_STATUS_INVALID_ARG:
            return "INVALID_ARG";

        default:
            return "NO_VALID_DATA";
    }
}


/* =========================================================
   CO2 PROCESS
   ========================================================= */

static void CO2_Process(void)
{
    uint32_t now = HAL_GetTick();

    /*
     * Process UART data
     */

    MHZ19_Process(&co2_sensor);


    /*
     * Process completed measurement
     */

    if (request_pending &&
        !MHZ19_IsBusy(&co2_sensor))
    {
        report_status = MHZ19_GetStatus(&co2_sensor);

        if (report_status == MHZ19_STATUS_OK &&
            MHZ19_GetCO2(&co2_sensor, &report_ppm))
        {
            report_valid = 1;

            watch_response_count++;
        }
        else
        {
            report_valid = 0;
        }

        request_pending = 0;
    }


    /*
     * Request new measurement
     */

    if (MHZ19_IsReady(&co2_sensor) &&
        !MHZ19_IsBusy(&co2_sensor) &&
        !request_pending &&
        (uint32_t)(now - last_request_tick) >=
        CO2_REQUEST_INTERVAL_MS)
    {
        MHZ19_Status result;

        result = MHZ19_Request(&co2_sensor);

        last_request_tick = now;

        if (result == MHZ19_STATUS_OK)
        {
            request_pending = 1;

            watch_request_count++;
        }
        else
        {
            report_status = result;

            report_valid = 0;
        }
    }
}


/* =========================================================
   NRF TRANSMISSION PROCESS
   ========================================================= */

static void NRF_Process(void)
{
    uint32_t now = HAL_GetTick();

    /* Retry initialization when the radio is absent or was disconnected. */
    if (nrf_initialized == 0U)
    {
        if ((uint32_t)(now - last_nrf_init_tick) < NRF_INIT_RETRY_INTERVAL_MS)
        {
            return;
        }

        last_nrf_init_tick = now;
        spi_status = NRF24_Init();

        if (spi_status != 0U)
        {
            nrf_initialized = NRF24_TxMode(txAddress, NRF24_CHANNEL);
        }
        else
        {
            nrf_initialized = 0U;
        }

        is_transmit = 0U;
        return;
    }

    /* A debugger can request a one-shot RF test without waiting for CO2. */
    if (watch_nrf_test_request != 0U)
    {
        uint8_t test_payload[NRF24_PAYLOAD_SIZE] = {0U};

        watch_nrf_test_request = 0U;
        watch_nrf_test_done = 0U;
        memcpy(test_payload, "NRF24_LINK_TEST",
               sizeof("NRF24_LINK_TEST") - 1U);
        test_payload[31] = (uint8_t)(now & 0xFFU);

        watch_nrf_test_result = NRF24_Transmit(test_payload);
        watch_nrf_test_done = 1U;
        is_transmit = watch_nrf_test_result;

        if (watch_nrf_test_result != 0U)
        {
            watch_tx_count++;
        }
        else
        {
            watch_tx_fail_count++;
        }

        /* Re-enter initialization if the SPI bus itself repeatedly fails. */
        if ((nrf_spi_consecutive_errors >= 3U) ||
            (nrf_status_consecutive_errors >= 3U))
        {
            nrf_initialized = 0U;
            nrf_detected = 0U;
            nrf_config_ok = 0U;
        }
        return;
    }

    if ((uint32_t)(now - last_report_tick) < NRF_REPORT_INTERVAL_MS)
    {
        return;
    }

    last_report_tick = now;
    memset(txData, 0, sizeof(txData));

    if (!MHZ19_IsReady(&co2_sensor))
    {
        (void)snprintf((char *)txData, sizeof(txData), "CO2=ERR,WARMUP");
    }
    else if (report_valid != 0U)
    {
        (void)snprintf((char *)txData, sizeof(txData), "CO2=%uppm",
                       (unsigned int)report_ppm);
    }
    else
    {
        const char *err = MHZ19_GetErrorText(report_status);

        if ((report_status == MHZ19_STATUS_OK) ||
            (report_status == MHZ19_STATUS_WARMING_UP))
        {
            err = "NO_VALID_DATA";
        }

        (void)snprintf((char *)txData, sizeof(txData), "CO2=ERR,%s", err);
    }

    if (NRF24_Transmit(txData) != 0U)
    {
        is_transmit = 1U;
        watch_tx_count++;
    }
    else
    {
        is_transmit = 0U;
        watch_tx_fail_count++;
    }

    if ((nrf_spi_consecutive_errors >= 3U) ||
        (nrf_status_consecutive_errors >= 3U))
    {
        nrf_initialized = 0U;
        nrf_detected = 0U;
        nrf_config_ok = 0U;
    }
}


/* =========================================================
   WATCH UPDATE
   ========================================================= */

static void Watch_Update(void)
{
    static uint32_t last_watch_tick = 0U;
    uint32_t now = HAL_GetTick();

    /* Avoid flooding the radio's SPI bus while the debugger is attached. */
    if ((uint32_t)(now - last_watch_tick) < 250U)
    {
        return;
    }
    last_watch_tick = now;

    watch_counter++;
    watch_millis = now;

    /* SPI hardware configuration and HAL state. */
    watch_spi_config_ok =
        ((hspi1.Init.Mode == SPI_MODE_MASTER) &&
         (hspi1.Init.Direction == SPI_DIRECTION_2LINES) &&
         (hspi1.Init.DataSize == SPI_DATASIZE_8BIT) &&
         (hspi1.Init.CLKPolarity == SPI_POLARITY_LOW) &&
         (hspi1.Init.CLKPhase == SPI_PHASE_1EDGE) &&
         (hspi1.Init.NSS == SPI_NSS_SOFT) &&
         (hspi1.Init.BaudRatePrescaler == SPI_BAUDRATEPRESCALER_8) &&
         (hspi1.Init.FirstBit == SPI_FIRSTBIT_MSB) &&
         (hspi1.Init.TIMode == SPI_TIMODE_DISABLE) &&
         (hspi1.Init.CRCCalculation == SPI_CRCCALCULATION_DISABLE)) ? 1U : 0U;
    watch_spi_hal_ready =
        (hspi1.State == HAL_SPI_STATE_READY) ? 1U : 0U;
    watch_spi_last_transfer_ok = nrf_spi_last_transfer_ok;
    watch_spi_last_hal_status = nrf_spi_last_hal_status;
    watch_spi_hal_error = nrf_spi_last_hal_error;
    watch_spi_transfers = nrf_spi_transfer_count;
    watch_spi_errors = nrf_spi_error_count;
    watch_nrf_status_errors = nrf_status_error_count;
    watch_nrf_status_error_streak = nrf_status_consecutive_errors;

    /* Radio status and counters. */
    watch_nrf_initialized = nrf_initialized;
    watch_nrf_detected = nrf_detected;
    watch_nrf_config_ok = nrf_config_ok;
    watch_nrf_init_stage = nrf_init_stage;
    watch_nrf_init_error = nrf_init_error;
    watch_nrf_link_ack = (nrf_last_tx_result == NRF24_TX_ACK) ? 1U : 0U;
    watch_nrf_tx_result = nrf_last_tx_result;
    watch_nrf_status = nrf_last_status;
    watch_nrf_fifo = nrf_last_fifo;
    watch_nrf_observe_tx = nrf_last_observe_tx;
    watch_nrf_tx_attempts = nrf_tx_attempt_count;
    watch_nrf_tx_acks = nrf_tx_ack_count;
    watch_nrf_tx_max_rt = nrf_tx_max_rt_count;
    watch_nrf_tx_timeouts = nrf_tx_timeout_count;

    if (nrf_detected != 0U)
    {
        watch_nrf_reg_status = NRF24_ReadRegister(NRF_REG_STATUS);
        watch_nrf_reg_config = NRF24_ReadRegister(NRF_REG_CONFIG);
        watch_nrf_reg_rf_ch = NRF24_ReadRegister(NRF_REG_RF_CH);
        watch_nrf_reg_en_aa = NRF24_ReadRegister(NRF_REG_EN_AA);
        watch_nrf_reg_rf_setup = NRF24_ReadRegister(NRF_REG_RF_SETUP);
        watch_nrf_reg_setup_retr = NRF24_ReadRegister(NRF_REG_SETUP_RETR);
        watch_nrf_reg_fifo_status = NRF24_ReadRegister(NRF_REG_FIFO_STATUS);
    }

    /* Sensor */
    watch_co2 = report_ppm;
    watch_sensor_ready = MHZ19_IsReady(&co2_sensor);
    watch_sensor_busy = MHZ19_IsBusy(&co2_sensor);
    watch_request_pending = request_pending;
    watch_report_valid = report_valid;
    watch_sensor_status = (uint8_t)report_status;
}


/* USER CODE END 0 */


/* =========================================================
   MAIN
   ========================================================= */

int main(void)
{
    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();

    MX_DMA_Init();

    MX_SPI1_Init();

    MX_USART1_UART_Init();


    /* USER CODE BEGIN 2 */


    /*
     * Initialize sensor
     */

    HAL_Delay(2000);

    MHZ19_Init(
        &co2_sensor,
        &huart1
    );


    /*
     * Initialize timing
     */

    last_request_tick = HAL_GetTick();

    last_report_tick = HAL_GetTick();


    /*
     * NRF initialization is performed by NRF_Process().
     * It retries once per second if the radio is absent or disconnected.
     */
    nrf_initialized = 0U;
    last_nrf_init_tick = 0U;


    /* USER CODE END 2 */


    /* =====================================================
       MAIN LOOP
       ===================================================== */

    while (1)
    {
        /*
         * MH-Z19
         */

        CO2_Process();


        /*
         * NRF24: initialization/recovery, diagnostics and transmissions.
         */
        NRF_Process();


        /*
         * Debug variables
         */

        Watch_Update();
    }
}


/* =========================================================
   SYSTEM CLOCK CONFIGURATION
   ========================================================= */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};

    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};


    RCC_OscInitStruct.OscillatorType =
        RCC_OSCILLATORTYPE_HSI;

    RCC_OscInitStruct.HSIState =
        RCC_HSI_ON;

    RCC_OscInitStruct.HSICalibrationValue =
        RCC_HSICALIBRATION_DEFAULT;

    RCC_OscInitStruct.PLL.PLLState =
        RCC_PLL_ON;

    RCC_OscInitStruct.PLL.PLLSource =
        RCC_PLLSOURCE_HSI;

    RCC_OscInitStruct.PLL.PLLMUL =
        RCC_PLL_MUL8;

    RCC_OscInitStruct.PLL.PREDIV =
        RCC_PREDIV_DIV1;


    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }


    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK |
        RCC_CLOCKTYPE_SYSCLK |
        RCC_CLOCKTYPE_PCLK1;

    RCC_ClkInitStruct.SYSCLKSource =
        RCC_SYSCLKSOURCE_PLLCLK;

    RCC_ClkInitStruct.AHBCLKDivider =
        RCC_SYSCLK_DIV1;

    RCC_ClkInitStruct.APB1CLKDivider =
        RCC_HCLK_DIV1;


    if (HAL_RCC_ClockConfig(
        &RCC_ClkInitStruct,
        FLASH_LATENCY_1) != HAL_OK)
    {
        Error_Handler();
    }


    PeriphClkInit.PeriphClockSelection =
        RCC_PERIPHCLK_USART1;

    PeriphClkInit.Usart1ClockSelection =
        RCC_USART1CLKSOURCE_PCLK1;


    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
}


/* =========================================================
   ERROR HANDLER
   ========================================================= */

void Error_Handler(void)
{
    __disable_irq();

    while (1)
    {
    }
}


#ifdef USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{
}

#endif
