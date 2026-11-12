#include "mhz19.h"
#include <string.h>

static uint8_t MHZ19_Checksum(const uint8_t frame[MHZ19_FRAME_SIZE])
{
    uint8_t sum = 0U;
    uint8_t i;

    /* Sum bytes 1..7 (byte 0, 0xFF, is excluded). */
    for (i = 1U; i < 8U; i++)
    {
        sum = (uint8_t)(sum + frame[i]);
    }

    return (uint8_t)(0xFFU - sum + 1U);
}

static void MHZ19_AbortTransfers(MHZ19_Handle *sensor)
{
    /* Aborts may return HAL_ERROR if that transfer already completed. */
    (void)HAL_UART_AbortReceive(sensor->huart);
    (void)HAL_UART_AbortTransmit(sensor->huart);
}

void MHZ19_Init(MHZ19_Handle *sensor, UART_HandleTypeDef *huart)
{
    if ((sensor == NULL) || (huart == NULL))
    {
        return;
    }

    memset(sensor, 0, sizeof(*sensor));
    sensor->huart = huart;
    sensor->state = MHZ19_STATE_IDLE;
    sensor->status = MHZ19_STATUS_WARMING_UP;
    sensor->init_tick = HAL_GetTick();
    sensor->warmup_ms = MHZ19_DEFAULT_WARMUP_MS;
    sensor->timeout_ms = MHZ19_DEFAULT_TIMEOUT_MS;

    /* MH-Z19 read CO2 command: FF 01 86 00 00 00 00 00 79 */
    sensor->tx_buffer[0] = 0xFFU;
    sensor->tx_buffer[1] = 0x01U;
    sensor->tx_buffer[2] = 0x86U;
    sensor->tx_buffer[3] = 0x00U;
    sensor->tx_buffer[4] = 0x00U;
    sensor->tx_buffer[5] = 0x00U;
    sensor->tx_buffer[6] = 0x00U;
    sensor->tx_buffer[7] = 0x00U;
    sensor->tx_buffer[8] = 0x79U;
}

void MHZ19_SetWarmupTime(MHZ19_Handle *sensor, uint32_t warmup_ms)
{
    if (sensor != NULL)
    {
        sensor->warmup_ms = warmup_ms;
        sensor->init_tick = HAL_GetTick();
        sensor->status = (warmup_ms == 0U) ? MHZ19_STATUS_OK
                                           : MHZ19_STATUS_WARMING_UP;
    }
}

void MHZ19_SetTimeout(MHZ19_Handle *sensor, uint32_t timeout_ms)
{
    if ((sensor != NULL) && (timeout_ms > 0U))
    {
        sensor->timeout_ms = timeout_ms;
    }
}

uint8_t MHZ19_IsReady(const MHZ19_Handle *sensor)
{
    if ((sensor == NULL) || (sensor->huart == NULL))
    {
        return 0U;
    }

    return ((uint32_t)(HAL_GetTick() - sensor->init_tick) >= sensor->warmup_ms)
           ? 1U : 0U;
}

uint8_t MHZ19_IsBusy(const MHZ19_Handle *sensor)
{
    if (sensor == NULL)
    {
        return 0U;
    }

    return (sensor->state != MHZ19_STATE_IDLE) ? 1U : 0U;
}

MHZ19_Status MHZ19_Request(MHZ19_Handle *sensor)
{
    HAL_StatusTypeDef hal_status;

    if ((sensor == NULL) || (sensor->huart == NULL))
    {
        return MHZ19_STATUS_INVALID_ARG;
    }

    if (sensor->state != MHZ19_STATE_IDLE)
    {
        sensor->status = MHZ19_STATUS_BUSY;
        return MHZ19_STATUS_BUSY;
    }

    if (MHZ19_IsReady(sensor) == 0U)
    {
        sensor->status = MHZ19_STATUS_WARMING_UP;
        return MHZ19_STATUS_WARMING_UP;
    }

    sensor->request_tick = HAL_GetTick();
    sensor->state = MHZ19_STATE_WAITING_RX;
    sensor->status = MHZ19_STATUS_BUSY;

    /* Arm DMA RX before TX so a fast response is not missed. */
    hal_status = HAL_UART_Receive_DMA(sensor->huart,
                                      sensor->rx_buffer,
                                      MHZ19_FRAME_SIZE);
    if (hal_status != HAL_OK)
    {
        sensor->state = MHZ19_STATE_IDLE;
        sensor->status = MHZ19_STATUS_UART_ERROR;
        return sensor->status;
    }

    hal_status = HAL_UART_Transmit_DMA(sensor->huart,
                                       sensor->tx_buffer,
                                       MHZ19_FRAME_SIZE);
    if (hal_status != HAL_OK)
    {
        MHZ19_AbortTransfers(sensor);
        sensor->state = MHZ19_STATE_IDLE;
        sensor->status = MHZ19_STATUS_UART_ERROR;
        return sensor->status;
    }

    return MHZ19_STATUS_OK; /* Request accepted, result arrives asynchronously. */
}

void MHZ19_Process(MHZ19_Handle *sensor)
{
    uint8_t *r;

    if ((sensor == NULL) || (sensor->huart == NULL))
    {
        return;
    }

    if (sensor->state == MHZ19_STATE_ERROR_PENDING)
    {
        MHZ19_AbortTransfers(sensor);
        sensor->state = MHZ19_STATE_IDLE;
        sensor->status = MHZ19_STATUS_UART_ERROR;
        return;
    }

    if (sensor->state == MHZ19_STATE_RX_DONE)
    {
        r = sensor->rx_buffer;

        if ((r[0] != 0xFFU) || (r[1] != 0x86U))
        {
            sensor->status = MHZ19_STATUS_BAD_HEADER;
        }
        else if (MHZ19_Checksum(r) != r[8])
        {
            sensor->status = MHZ19_STATUS_BAD_CHECKSUM;
        }
        else
        {
            sensor->co2_ppm = (uint16_t)(((uint16_t)r[2] << 8) | r[3]);
            sensor->has_valid_measurement = 1U;
            sensor->new_measurement = 1U;
            sensor->last_success_tick = HAL_GetTick();
            sensor->status = MHZ19_STATUS_OK;
        }

        sensor->state = MHZ19_STATE_IDLE;
        return;
    }

    if ((sensor->state == MHZ19_STATE_WAITING_RX) &&
        ((uint32_t)(HAL_GetTick() - sensor->request_tick) >= sensor->timeout_ms))
    {
        MHZ19_AbortTransfers(sensor);
        sensor->state = MHZ19_STATE_IDLE;
        sensor->status = MHZ19_STATUS_TIMEOUT;
    }
}

uint8_t MHZ19_GetCO2(const MHZ19_Handle *sensor, uint16_t *ppm)
{
    if ((sensor == NULL) || (ppm == NULL) ||
        (sensor->has_valid_measurement == 0U))
    {
        return 0U;
    }

    *ppm = sensor->co2_ppm;
    return 1U;
}

uint8_t MHZ19_GetNewCO2(MHZ19_Handle *sensor, uint16_t *ppm)
{
    if ((sensor == NULL) || (ppm == NULL) ||
        (sensor->new_measurement == 0U))
    {
        return 0U;
    }

    *ppm = sensor->co2_ppm;
    sensor->new_measurement = 0U;
    return 1U;
}

MHZ19_Status MHZ19_GetStatus(const MHZ19_Handle *sensor)
{
    if (sensor == NULL)
    {
        return MHZ19_STATUS_INVALID_ARG;
    }

    return sensor->status;
}

uint32_t MHZ19_GetLastSuccessTick(const MHZ19_Handle *sensor)
{
    return (sensor != NULL) ? sensor->last_success_tick : 0U;
}

void MHZ19_UART_RxCpltCallback(MHZ19_Handle *sensor,
                               UART_HandleTypeDef *huart)
{
    if ((sensor == NULL) || (huart == NULL) || (sensor->huart != huart))
    {
        return;
    }

    if (sensor->state == MHZ19_STATE_WAITING_RX)
    {
        /* Keep ISR short; validation happens in MHZ19_Process(). */
        sensor->state = MHZ19_STATE_RX_DONE;
    }
}

void MHZ19_UART_ErrorCallback(MHZ19_Handle *sensor,
                              UART_HandleTypeDef *huart)
{
    if ((sensor == NULL) || (huart == NULL) || (sensor->huart != huart))
    {
        return;
    }

    if (sensor->state == MHZ19_STATE_WAITING_RX)
    {
        /* Abort/recovery is deferred to main context. */
        sensor->state = MHZ19_STATE_ERROR_PENDING;
    }
}
