#include "NRF24L01.h"
#include <string.h>
#ifndef NRF24_HOST_TEST
#include "main.h"
#include "nrf24_board.h"
extern SPI_HandleTypeDef NRF24_BOARD_SPI;
#endif
static const uint8_t address[5]={0xEE,0xDD,0xCC,0xBB,0xAA};
#define REG_CONFIG 0x00U
#define REG_STATUS 0x07U
#define REG_FIFO 0x17U
#define RX_DR 0x40U
#define TX_DS 0x20U
#define MAX_RT 0x10U

static uint8_t Exchange(NRF24_Radio *n,uint8_t command,const uint8_t *data,uint8_t *out,uint8_t length)
{
    uint8_t tx[33],rx[33];
    if(length>32U) return 0U;
    tx[0]=command;
    for(uint8_t i=0;i<length;i++) tx[i+1U]=data?data[i]:0xFFU;
    n->io.csn(n->io.context,0U);
    uint8_t ok=n->io.transfer(n->io.context,tx,rx,(uint8_t)(length+1U));
    n->io.csn(n->io.context,1U);
    if(!ok || (rx[0]&0x80U)) { n->stats.spi_errors++; return 0U; }
    n->status=rx[0];
    if(out) memcpy(out,rx+1,length);
    return 1U;
}
static uint8_t ReadReg(NRF24_Radio *n,uint8_t reg,uint8_t *v) { return Exchange(n,reg&31U,0,v,1U); }
static uint8_t WriteReg(NRF24_Radio *n,uint8_t reg,uint8_t v) { return Exchange(n,(reg&31U)|0x20U,&v,0,1U); }
static uint8_t Command(NRF24_Radio *n,uint8_t cmd) { return Exchange(n,cmd,0,0,0U); }
static uint8_t CheckReg(NRF24_Radio *n,uint8_t reg,uint8_t expected)
{
    uint8_t value=0;
    return ReadReg(n,reg,&value) && value==expected;
}
static void Complete(NRF24_Radio *n,NRF24_TxResult result)
{
    n->last_tx_result=result;
    n->completed_tag=n->active_tag;
    n->stats.tx_completed++;
}
static void Recover(NRF24_Radio *n,uint32_t now)
{
    if(n->state==NRF24_CE_PULSE || n->state==NRF24_WAIT_ACK)
        Complete(n,NRF24_TX_SPI_ERROR);
    n->io.ce(n->io.context,0U);
    n->io.csn(n->io.context,1U);
    n->ready=n->detected=0U;
    n->pending=0U; /* Caller rebuilds fresh telemetry after recovery. */
    n->state=NRF24_RETRY;
    n->since_ms=now;
    n->stats.recoveries++;
}
static uint8_t Configure(NRF24_Radio *n)
{
    uint8_t bytes[5];
    /* Probe two values, so floating/stuck MISO cannot claim a detected chip. */
    if(!WriteReg(n,5U,0x2AU) || !CheckReg(n,5U,0x2AU) ||
       !WriteReg(n,5U,0x55U) || !CheckReg(n,5U,0x55U)) return 0U;
    n->detected=1U;
    const uint8_t regs[]={0U,1U,2U,3U,4U,5U,6U,0x11U,0x1CU,0x1DU};
    const uint8_t values[]={0x0CU,1U,1U,3U,0x2FU,NRF24_CHANNEL,0x06U,32U,0U,0U};
    for(uint8_t i=0;i<sizeof(regs);i++)
        if(!WriteReg(n,regs[i],values[i]) || !CheckReg(n,regs[i],values[i])) return 0U;
    if(!Exchange(n,0x2AU,address,0,5U) || !Exchange(n,0x30U,address,0,5U) ||
       !Exchange(n,0x0AU,0,bytes,5U) || memcmp(bytes,address,5U) ||
       !Exchange(n,0x10U,0,bytes,5U) || memcmp(bytes,address,5U)) return 0U;
    if(!WriteReg(n,REG_STATUS,0x70U) || !Command(n,0xE1U) || !Command(n,0xE2U)) return 0U;
    uint8_t config=n->role==NRF24_RECEIVER?0x0FU:0x0EU;
    return WriteReg(n,REG_CONFIG,config) && CheckReg(n,REG_CONFIG,config);
}
uint8_t NRF24_Init(NRF24_Radio *n,NRF24_Role role,const NRF24_IO *io)
{
    if(!n || !io || !io->millis || !io->transfer || !io->ce || !io->csn ||
       (role!=NRF24_RECEIVER && role!=NRF24_TRANSMITTER)) return 0U;
    memset(n,0,sizeof(*n));
    n->io=*io; n->role=role; n->state=NRF24_BOOT;
    n->since_ms=io->millis(io->context);
    io->ce(io->context,0U); io->csn(io->context,1U);
    return 1U;
}
uint8_t NRF24_Queue(NRF24_Radio *n,const uint8_t payload[32],uint32_t tag)
{
    if(!n || n->role!=NRF24_TRANSMITTER || !payload) return 0U;
    memcpy(n->pending_data,payload,32U);
    n->pending_tag=tag; n->pending=1U;
    return 1U;
}
uint8_t NRF24_Read(NRF24_Radio *n,uint8_t payload[32])
{
    if(!n || !payload || !n->count) return 0U;
    memcpy(payload,n->queue[n->head],32U);
    n->head=(n->head+1U)%NRF24_RX_QUEUE; n->count--;
    return 1U;
}
void NRF24_Process(NRF24_Radio *n)
{
    if(!n || !n->io.millis) return;
    uint32_t now=n->io.millis(n->io.context);
    uint8_t value;
    if(n->state==NRF24_RETRY) {
        if((uint32_t)(now-n->since_ms)<NRF24_RETRY_MS) return;
        n->state=NRF24_BOOT; n->since_ms=now;
    }
    if(n->state==NRF24_BOOT) {
        if((uint32_t)(now-n->since_ms)<100U) return; /* Power-on reset margin. */
        if(!Configure(n)) { n->stats.config_errors++; Recover(n,now); return; }
        n->state=NRF24_POWER; n->since_ms=n->io.millis(n->io.context);
        return;
    }
    if(n->state==NRF24_POWER) {
        if((uint32_t)(now-n->since_ms)<5U) return; /* PWR_UP -> standby. */
        n->ready=1U; n->health_ms=now; n->poll_ms=now; n->since_ms=now;
        n->state=n->role==NRF24_RECEIVER?NRF24_RX:NRF24_IDLE;
        if(n->role==NRF24_RECEIVER) n->io.ce(n->io.context,1U);
        return;
    }
    if(n->state==NRF24_CE_PULSE) {
        if((uint32_t)(now-n->since_ms)<2U) return; /* >= 1 ms actual CE pulse. */
        n->io.ce(n->io.context,0U);
        n->state=NRF24_WAIT_ACK;
    }
    if(n->state==NRF24_WAIT_ACK) {
        if((uint32_t)(now-n->poll_ms)<1U) return;
        n->poll_ms=now;
        if(!Command(n,0xFFU)) { Recover(n,now); return; }
        if((n->status&(TX_DS|MAX_RT))==(TX_DS|MAX_RT)) { Recover(n,now); return; }
        if(n->status&TX_DS) {
            if(!WriteReg(n,REG_STATUS,TX_DS)) { Recover(n,now); return; }
            n->stats.tx_acks++; Complete(n,NRF24_TX_ACK); n->state=NRF24_IDLE;
        } else if(n->status&MAX_RT) {
            if(!WriteReg(n,REG_STATUS,MAX_RT) || !Command(n,0xE1U)) { Recover(n,now); return; }
            n->stats.tx_max_rt++; Complete(n,NRF24_TX_MAX_RT); n->state=NRF24_IDLE;
        } else if((uint32_t)(now-n->since_ms)>=NRF24_TX_TIMEOUT_MS) {
            n->stats.tx_timeouts++; Complete(n,NRF24_TX_TIMEOUT);
            /* Recover must not emit another completion for this same attempt. */
            n->state=NRF24_IDLE; Recover(n,now);
        }
        return;
    }
    if(!n->ready) return;
    if((uint32_t)(now-n->health_ms)>=1000U) {
        n->health_ms=now;
        if(!CheckReg(n,REG_CONFIG,n->role==NRF24_RECEIVER?0x0FU:0x0EU) ||
           !CheckReg(n,5U,NRF24_CHANNEL) || !CheckReg(n,6U,0x06U)) {
            Recover(n,now); return;
        }
    }
    if(n->state==NRF24_IDLE && n->pending) {
        if(!WriteReg(n,REG_STATUS,TX_DS|MAX_RT) || !Command(n,0xE1U) ||
           !Exchange(n,0xA0U,n->pending_data,0,32U)) { Recover(n,now); return; }
        n->active_tag=n->pending_tag; n->pending=0U;
        n->stats.tx_attempts++; n->last_tx_result=NRF24_TX_NONE;
        n->io.ce(n->io.context,1U);
        n->since_ms=n->io.millis(n->io.context); n->state=NRF24_CE_PULSE;
        return;
    }
    if(n->state!=NRF24_RX || (uint32_t)(now-n->poll_ms)<1U) return;
    n->poll_ms=now;
    for(uint8_t budget=0;budget<3U;budget++) {
        if(!ReadReg(n,REG_FIFO,&value)) { Recover(n,now); return; }
        if(value&1U) break;
        if(((n->status>>1U)&7U)!=0U) { Recover(n,now); return; }
        uint8_t payload[32];
        if(!Exchange(n,0x61U,0,payload,32U)) { Recover(n,now); return; }
        if(n->count==NRF24_RX_QUEUE) {
            n->head=(n->head+1U)%NRF24_RX_QUEUE; n->count--; n->stats.rx_dropped++;
        }
        memcpy(n->queue[(n->head+n->count)%NRF24_RX_QUEUE],payload,32U);
        n->count++; n->stats.rx_packets++;
    }
    /* Poll FIFO next time too: an arrival racing RX_DR clear cannot be lost. */
    if(!WriteReg(n,REG_STATUS,RX_DR)) Recover(n,now);
}
#ifndef NRF24_HOST_TEST
static uint32_t BoardMillis(void *unused) { (void)unused; return HAL_GetTick(); }
static void BoardCE(void *unused,uint8_t on) { (void)unused; HAL_GPIO_WritePin(CE_GPIO_Port,CE_Pin,on?GPIO_PIN_SET:GPIO_PIN_RESET); }
static void BoardCSN(void *unused,uint8_t on) { (void)unused; HAL_GPIO_WritePin(CSN_GPIO_Port,CSN_Pin,on?GPIO_PIN_SET:GPIO_PIN_RESET); }
static uint8_t BoardTransfer(void *unused,const uint8_t *tx,uint8_t *rx,uint8_t size)
{
    (void)unused;
    SPI_HandleTypeDef *spi=&NRF24_BOARD_SPI;
    uint32_t start=HAL_GetTick();
    /* Only one byte can be outstanding: avoids RX overrun under XIP/IRQ latency.
     * CSN remains asserted across the entire command, including byte gaps. */
    for(uint8_t i=0;i<size;i++) {
        uint8_t byte=tx[i]; /* Older STM32F0 HAL lacks const on its TX pointer. */
        if((uint32_t)(HAL_GetTick()-start)>5U ||
           HAL_SPI_TransmitReceive(spi,&byte,rx+i,1U,2U)!=HAL_OK) {
            __HAL_SPI_DISABLE(spi); __HAL_SPI_CLEAR_OVRFLAG(spi);
            spi->State=HAL_SPI_STATE_READY; spi->Lock=HAL_UNLOCKED;
            return 0U;
        }
    }
    return 1U;
}
uint8_t NRF24_BoardInit(NRF24_Radio *n,NRF24_Role role)
{
    SPI_HandleTypeDef *spi=&NRF24_BOARD_SPI;
    uint32_t br=spi->Init.BaudRatePrescaler;
    if((br&~SPI_CR1_BR) || NRF24_BOARD_PCLK_HZ/(2U<<(br>>3U))>1000000U) return 0U;
    if(spi->Init.Mode!=SPI_MODE_MASTER || spi->Init.Direction!=SPI_DIRECTION_2LINES ||
       spi->Init.DataSize!=SPI_DATASIZE_8BIT || spi->Init.CLKPolarity!=SPI_POLARITY_LOW ||
       spi->Init.CLKPhase!=SPI_PHASE_1EDGE || spi->Init.NSS!=SPI_NSS_SOFT ||
       spi->Init.FirstBit!=SPI_FIRSTBIT_MSB || spi->Init.TIMode!=SPI_TIMODE_DISABLE ||
       spi->Init.CRCCalculation!=SPI_CRCCALCULATION_DISABLE) return 0U;
    NRF24_IO io={0,BoardMillis,BoardTransfer,BoardCE,BoardCSN};
    return NRF24_Init(n,role,&io);
}
#endif
