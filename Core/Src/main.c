
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

static uint32_t last_request_tick = 0;
static uint32_t last_report_tick = 0;


/* ================= NRF STATUS ================= */

static uint8_t nrf_initialized = 0;

static uint8_t is_transmit = 0;
static uint8_t spi_status = 0;


/* ================= WATCH VARIABLES ================= */

/* General debug */

volatile uint32_t watch_counter = 0;
volatile uint32_t watch_test = 12345;
volatile float watch_float = 25.5f;
volatile uint32_t watch_millis = 0;


/* NRF debug */

volatile uint8_t watch_spi_status = 0;
volatile uint8_t watch_nrf_initialized = 0;
volatile uint8_t watch_is_transmit = 0;
volatile uint8_t watch_nrf_tx_result = 0;

volatile uint8_t watch_nrf_status = 0;
volatile uint8_t watch_nrf_fifo = 0;

volatile uint32_t watch_spi_errors = 0;

volatile uint32_t watch_tx_count = 0;
volatile uint32_t watch_tx_fail_count = 0;


/* Sensor debug */

volatile uint16_t watch_co2 = 0;

volatile uint8_t watch_sensor_ready = 0;
volatile uint8_t watch_sensor_busy = 0;
volatile uint8_t watch_request_pending = 0;
volatile uint8_t watch_report_valid = 0;
volatile uint8_t watch_sensor_status = 0;

volatile uint32_t watch_request_count = 0;
volatile uint32_t watch_response_count = 0;


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

    /*
     * Transmission interval
     */

    if ((uint32_t)(now - last_report_tick) <
        NRF_REPORT_INTERVAL_MS)
    {
        return;
    }

    last_report_tick = now;


    /*
     * Clear TX buffer
     */

    memset(txData, 0, sizeof(txData));


    /*
     * Build CO2 message
     */

    if (!MHZ19_IsReady(&co2_sensor))
    {
        snprintf(
            (char *)txData,
            sizeof(txData),
            "CO2=ERR,WARMUP"
        );
    }

    else if (report_valid)
    {
        snprintf(
            (char *)txData,
            sizeof(txData),
            "CO2=%uppm",
            (unsigned int)report_ppm
        );
    }

    else
    {
        const char *err =
            MHZ19_GetErrorText(report_status);

        if (report_status == MHZ19_STATUS_OK ||
            report_status == MHZ19_STATUS_WARMING_UP)
        {
            err = "NO_VALID_DATA";
        }

        snprintf(
            (char *)txData,
            sizeof(txData),
            "CO2=ERR,%s",
            err
        );
    }


    /*
     * Send NRF packet
     */

    if (NRF24_Transmit(txData))
    {
        is_transmit = 1;

        watch_nrf_tx_result = 1;

        watch_tx_count++;
    }
    else
    {
        is_transmit = 0;

        watch_nrf_tx_result = 0;

        watch_tx_fail_count++;
    }
}


/* =========================================================
   WATCH UPDATE
   ========================================================= */

static void Watch_Update(void)
{
    watch_counter++;

    watch_test++;

    watch_float += 0.1f;

    watch_millis = HAL_GetTick();


    /* Sensor */

    watch_co2 = report_ppm;

    watch_sensor_ready =
        MHZ19_IsReady(&co2_sensor);

    watch_sensor_busy =
        MHZ19_IsBusy(&co2_sensor);

    watch_request_pending =
        request_pending;

    watch_report_valid =
        report_valid;

    watch_sensor_status =
        (uint8_t)report_status;


    /* NRF */

    watch_spi_status = spi_status;

    watch_nrf_initialized =
        nrf_initialized;

    watch_is_transmit = is_transmit;

    watch_nrf_status = nrf_last_status;

    watch_nrf_fifo = nrf_last_fifo;

    watch_spi_errors = nrf_spi_error_count;
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
     * Initialize NRF
     */

    spi_status = NRF24_Init();

    if (spi_status)
    {
        nrf_initialized = NRF24_TxMode(
            txAddress,
            NRF24_CHANNEL
        );
    }
    else
    {
        nrf_initialized = 0;
    }


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
         * NRF24
         */

        if (nrf_initialized)
        {
            NRF_Process();
        }


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
