#ifndef NRF24L01_H
#define NRF24L01_H
#include <stdint.h>
#include <stddef.h>
#define NRF24_CHANNEL 10U
#define NRF24_PAYLOAD_SIZE 32U
#define NRF24_RX_QUEUE 3U
#define NRF24_RETRY_MS 1000U
#define NRF24_TX_TIMEOUT_MS 100U
/* Both peers: EE DD CC BB AA, channel 10, 1 Mbps, 0 dBm, CRC16,
 * auto-ACK on pipe 0, 750 us / 15 hardware retries, fixed 32-byte frames. */
typedef enum { NRF24_RECEIVER=0, NRF24_TRANSMITTER=1 } NRF24_Role;
typedef enum { NRF24_TX_NONE, NRF24_TX_ACK, NRF24_TX_MAX_RT, NRF24_TX_SPI_ERROR,
               NRF24_TX_TIMEOUT } NRF24_TxResult;
typedef enum { NRF24_BOOT, NRF24_POWER, NRF24_RX, NRF24_IDLE, NRF24_CE_PULSE,
               NRF24_WAIT_ACK, NRF24_RETRY } NRF24_State;
typedef struct {
    void *context;
    uint32_t (*millis)(void *);
    uint8_t (*transfer)(void *,const uint8_t *,uint8_t *,uint8_t);
    void (*ce)(void *,uint8_t);
    void (*csn)(void *,uint8_t);
} NRF24_IO;
typedef struct {
    uint32_t spi_errors, config_errors, recoveries, tx_attempts, tx_acks;
    uint32_t tx_max_rt, tx_timeouts, tx_completed, rx_packets, rx_dropped;
} NRF24_Stats;
typedef struct {
    NRF24_IO io;
    NRF24_Stats stats;
    NRF24_State state;
    NRF24_Role role;
    NRF24_TxResult last_tx_result;
    uint32_t since_ms, poll_ms, health_ms, pending_tag, active_tag, completed_tag;
    uint8_t ready, detected, pending, status, pending_data[32];
    uint8_t queue[NRF24_RX_QUEUE][32], head, count;
} NRF24_Radio;
/* Process/Queue/Read: one main/task owner; callbacks do not call these APIs.
 * RF waits are state-driven. Each Process drains at most three RX frames.
 * SPI transport is short/bounded, not a wait for RF completion. */
uint8_t NRF24_Init(NRF24_Radio *,NRF24_Role,const NRF24_IO *);
void NRF24_Process(NRF24_Radio *);
/* Queue holds the latest unsent frame, replacing an older pending telemetry frame. */
uint8_t NRF24_Queue(NRF24_Radio *,const uint8_t payload[32],uint32_t tag);
uint8_t NRF24_Read(NRF24_Radio *,uint8_t payload[32]);
#ifndef NRF24_HOST_TEST
uint8_t NRF24_BoardInit(NRF24_Radio *,NRF24_Role);
#endif
#endif
