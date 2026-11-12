#ifndef MHZ19_H
#define MHZ19_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f0xx_hal.h"   /* For STM32F030C8T6. Change for another STM32 family. */
#include <stdint.h>

#define MHZ19_FRAME_SIZE             9U
#define MHZ19_DEFAULT_WARMUP_MS      180000UL
#define MHZ19_DEFAULT_TIMEOUT_MS     500UL

/* Result returned by API calls and stored in sensor->status. */
typedef enum
{
    MHZ19_STATUS_OK = 0,          /* Latest received frame is valid / request accepted */
    MHZ19_STATUS_BUSY,
    MHZ19_STATUS_WARMING_UP,
    MHZ19_STATUS_TIMEOUT,
    MHZ19_STATUS_UART_ERROR,
    MHZ19_STATUS_BAD_HEADER,
    MHZ19_STATUS_BAD_CHECKSUM,
    MHZ19_STATUS_INVALID_ARG
} MHZ19_Status;

typedef enum
{
    MHZ19_STATE_IDLE = 0,
    MHZ19_STATE_WAITING_RX,
    MHZ19_STATE_RX_DONE,
    MHZ19_STATE_ERROR_PENDING
} MHZ19_State;

typedef struct
{
    UART_HandleTypeDef *huart;

    /* These members are shared with HAL interrupt callbacks. */
    volatile MHZ19_State state;
    volatile MHZ19_Status status;

    uint8_t tx_buffer[MHZ19_FRAME_SIZE];
    uint8_t rx_buffer[MHZ19_FRAME_SIZE];

    /* Data is updated in MHZ19_Process(), not inside the ISR. */
    uint16_t co2_ppm;
    uint8_t has_valid_measurement;
    uint8_t new_measurement;
    uint32_t last_success_tick;

    uint32_t init_tick;
    uint32_t request_tick;
    uint32_t warmup_ms;
    uint32_t timeout_ms;
} MHZ19_Handle;

/* Call after MX_USARTx_UART_Init() and DMA initialization. */
void MHZ19_Init(MHZ19_Handle *sensor, UART_HandleTypeDef *huart);

/* Optional timing configuration. Set warmup_ms=0 if already warmed up. */
void MHZ19_SetWarmupTime(MHZ19_Handle *sensor, uint32_t warmup_ms);
void MHZ19_SetTimeout(MHZ19_Handle *sensor, uint32_t timeout_ms);
uint8_t MHZ19_IsReady(const MHZ19_Handle *sensor);
uint8_t MHZ19_IsBusy(const MHZ19_Handle *sensor);

/* Starts one asynchronous request using UART DMA. Call MHZ19_Process() often. */
MHZ19_Status MHZ19_Request(MHZ19_Handle *sensor);
void MHZ19_Process(MHZ19_Handle *sensor);

/* Reads the last valid measurement without consuming it. */
uint8_t MHZ19_GetCO2(const MHZ19_Handle *sensor, uint16_t *ppm);

/* Reads a newly received valid measurement once; returns 1 only for new data. */
uint8_t MHZ19_GetNewCO2(MHZ19_Handle *sensor, uint16_t *ppm);

MHZ19_Status MHZ19_GetStatus(const MHZ19_Handle *sensor);
uint32_t MHZ19_GetLastSuccessTick(const MHZ19_Handle *sensor);

/* Forward these from the matching HAL callbacks in main.c. */
void MHZ19_UART_RxCpltCallback(MHZ19_Handle *sensor,
                               UART_HandleTypeDef *huart);
void MHZ19_UART_ErrorCallback(MHZ19_Handle *sensor,
                              UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* MHZ19_H */
