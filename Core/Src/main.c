/* CO2 node: cooperative sensor and radio services; no RF busy wait. */
#include "main.h"
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "NRF24L01.h"
#include "co2_link.h"
#include "mhz19.h"

MHZ19_Handle co2_sensor;
NRF24_Radio radio;
static uint16_t report_ppm;
static MHZ19_Status report_status=MHZ19_STATUS_WARMING_UP;
static uint8_t request_pending,report_valid,radio_config_ok;
static uint32_t last_request_tick,last_report_tick,sequence;
static uint8_t test_pending;
static uint32_t test_tag,test_events,test_tick;
#define CO2_REQUEST_INTERVAL_MS 2000U
volatile uint32_t watch_counter,watch_millis,watch_request_count,watch_response_count;
volatile uint16_t watch_co2;
volatile uint8_t watch_report_valid,watch_sensor_status;
volatile uint8_t watch_nrf_initialized,watch_nrf_link_ack,watch_nrf_tx_result;
volatile uint8_t watch_nrf_test_request,watch_nrf_test_done,watch_nrf_test_result;

void SystemClock_Config(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *h) { MHZ19_UART_RxCpltCallback(&co2_sensor,h); }
void HAL_UART_ErrorCallback(UART_HandleTypeDef *h) { MHZ19_UART_ErrorCallback(&co2_sensor,h); }
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

static uint8_t SensorStatus(uint32_t age)
{
    if(!MHZ19_IsReady(&co2_sensor)) return CO2_WARMUP;
    if(report_valid && age<=CO2_LINK_STALE_MS) return CO2_GOOD;
    switch(report_status) {
    case MHZ19_STATUS_TIMEOUT: return CO2_TIMEOUT;
    case MHZ19_STATUS_UART_ERROR: return CO2_UART_ERROR;
    case MHZ19_STATUS_BAD_HEADER: return CO2_BAD_HEADER;
    case MHZ19_STATUS_BAD_CHECKSUM: return CO2_BAD_CHECKSUM;
    default: return CO2_NOT_READY;
    }
}
static void NRF_Process(void)
{
    uint32_t now=HAL_GetTick();
    if(!radio_config_ok) return;
    NRF24_Process(&radio);
    if(test_pending) {
        if(radio.stats.tx_completed!=test_events && radio.completed_tag==test_tag) {
            watch_nrf_test_result=(uint8_t)radio.last_tx_result;
            watch_nrf_test_done=1U; test_pending=0U;
        } else if((uint32_t)(now-test_tick)>=2000U) {
            watch_nrf_test_result=NRF24_TX_TIMEOUT;
            watch_nrf_test_done=1U; test_pending=0U;
        }
    }
    if(!radio.ready) return;
    uint8_t frame[32];
    if(watch_nrf_test_request && !test_pending) {
        CO2_Packet p={0};
        p.node=CO2_LINK_NODE; p.type=CO2_LINK_TEST; p.sequence=sequence++;
        p.uptime_ms=now; p.sample_age_ms=UINT32_MAX; p.status=CO2_NOT_READY;
        CO2_Encode(frame,&p);
        test_tag=p.sequence; test_events=radio.stats.tx_completed; test_tick=now;
        test_pending=1U; watch_nrf_test_request=0U; watch_nrf_test_done=0U;
        (void)NRF24_Queue(&radio,frame,p.sequence);
        return;
    }
    if(test_pending || (uint32_t)(now-last_report_tick)<CO2_LINK_REPORT_MS) return;
    last_report_tick=now;
    CO2_Packet p={0};
    p.type=CO2_LINK_SAMPLE; p.node=CO2_LINK_NODE; p.sequence=sequence++;
    p.uptime_ms=now; p.ppm=report_ppm; p.sample_count=watch_response_count;
    p.flags=co2_sensor.has_valid_measurement?CO2_LINK_HAS_SAMPLE:0U;
    p.sample_age_ms=p.flags?(uint32_t)(now-MHZ19_GetLastSuccessTick(&co2_sensor)):UINT32_MAX;
    p.status=SensorStatus(p.sample_age_ms);
    CO2_Encode(frame,&p);
    (void)NRF24_Queue(&radio,frame,p.sequence);
}
int main(void)
{
    HAL_Init(); SystemClock_Config();
    MX_GPIO_Init(); MX_DMA_Init(); MX_SPI1_Init(); MX_USART1_UART_Init();
    MHZ19_Init(&co2_sensor,&huart1);
    radio_config_ok=NRF24_BoardInit(&radio,NRF24_TRANSMITTER);
    last_request_tick=last_report_tick=HAL_GetTick();
    for(;;) {
        CO2_Process(); NRF_Process();
        watch_counter++; watch_millis=HAL_GetTick(); watch_co2=report_ppm;
        watch_report_valid=report_valid; watch_sensor_status=(uint8_t)report_status;
        watch_nrf_initialized=radio.ready;
        watch_nrf_link_ack=radio.last_tx_result==NRF24_TX_ACK;
        watch_nrf_tx_result=(uint8_t)radio.last_tx_result;
        __WFI(); /* SysTick/UART wake the cooperative services; no busy spin. */
    }
}
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
