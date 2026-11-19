#ifndef MHZ19_TEST_HAL_H
#define MHZ19_TEST_HAL_H
#include <stdint.h>
typedef struct {int unused;} UART_HandleTypeDef;
typedef enum {HAL_OK,HAL_ERROR,HAL_BUSY,HAL_TIMEOUT} HAL_StatusTypeDef;
uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *,uint8_t *,uint16_t);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *,uint8_t *,uint16_t);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *);
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *);
#endif
