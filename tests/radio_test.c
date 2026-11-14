#include "NRF24L01.h"
#include "co2_link.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

typedef struct Chip {
    uint8_t regs[32][5],status,connected,ce,csn,tx[32],have_tx;
    uint8_t fifo[3][32],count,fail,stuck;
    uint32_t send_at,flush_rx;
    struct Chip *peer;
} Chip;
static uint32_t tick;
static uint32_t Now(void *p) { (void)p; return tick; }
static void CSN(void *p,uint8_t on) { ((Chip*)p)->csn=on; }
static uint8_t Status(Chip *c) { return (c->status&0x70U)|(c->count?0U:0x0EU); }
static void Update(Chip *c)
{
    if(!c->have_tx || c->stuck || (uint32_t)(tick-c->send_at)<3U) return;
    Chip *p=c->peer;
    if(p && p->connected && p->ce && (p->regs[0][0]&1U) && p->count<3U &&
       p->regs[5][0]==c->regs[5][0] && p->regs[6][0]==c->regs[6][0] &&
       (p->regs[0][0]&12U)==(c->regs[0][0]&12U) &&
       !memcmp(p->regs[10],c->regs[16],5U)) {
        memcpy(p->fifo[p->count++],c->tx,32U); p->status|=0x40U;
        if(p->regs[1][0]&1U) c->status|=0x20U; else c->status|=0x10U;
    } else c->status|=0x10U;
    c->have_tx=0;
}
static void CE(void *v,uint8_t on)
{
    Chip *c=v;
    if(on && !c->ce && !(c->regs[0][0]&1U) && c->have_tx) c->send_at=tick;
    c->ce=on;
}
static uint8_t Transfer(void *v,const uint8_t *tx,uint8_t *rx,uint8_t n)
{
    Chip *c=v;
    assert(!c->csn && n>=1U && n<=33U);
    if(c->fail) { c->fail=0; return 0; }
    memset(rx,0,n);
    if(!c->connected) { memset(rx,0xFFU,n); return 1; }
    Update(c); rx[0]=Status(c);
    uint8_t cmd=tx[0];
    if(cmd<=31U) {
        uint8_t reg=cmd&31U;
        if(reg==7U) rx[1]=Status(c);
        else if(reg==23U) rx[1]=c->count?0U:1U;
        else memcpy(rx+1,c->regs[reg],n-1U);
    } else if(cmd>=0x20U && cmd<=0x3FU) {
        uint8_t reg=cmd&31U;
        if(reg==7U) c->status&=(uint8_t)~tx[1];
        else memcpy(c->regs[reg],tx+1,n-1U);
    } else if(cmd==0xA0U) { assert(n==33U); memcpy(c->tx,tx+1,32U); c->have_tx=1; }
    else if(cmd==0x61U) {
        assert(n==33U && c->count);
        memcpy(rx+1,c->fifo[0],32U);
        c->count--; memmove(c->fifo[0],c->fifo[1],c->count*32U);
    } else if(cmd==0xE1U) c->have_tx=0;
    else if(cmd==0xE2U) { c->count=0; c->flush_rx++; }
    else assert(cmd==0xFFU);
    return 1;
}
static void Advance(NRF24_Radio *tx,NRF24_Radio *rx,uint32_t ms)
{
    while(ms--) { tick++; NRF24_Process(rx); NRF24_Process(tx); }
}
int main(int argc,char **argv)
{
    (void)argv;
    tick=argc>1?UINT32_MAX-50U:0U;
    Chip t={.connected=1,.csn=1},r={.connected=1,.csn=1}; t.peer=&r; r.peer=&t;
    NRF24_IO ti={&t,Now,Transfer,CE,CSN},ri={&r,Now,Transfer,CE,CSN};
    NRF24_Radio tx,rx;
    NRF24_Init(&tx,NRF24_TRANSMITTER,&ti); NRF24_Init(&rx,NRF24_RECEIVER,&ri);
    Advance(&tx,&rx,99U); assert(!tx.ready && !rx.ready);
    Advance(&tx,&rx,10U); assert(tx.ready && rx.ready);
    assert(t.regs[5][0]==10U && r.regs[5][0]==10U && t.regs[6][0]==6U && r.regs[6][0]==6U);
    assert(t.regs[0][0]==14U && r.regs[0][0]==15U && r.regs[1][0]==1U && r.regs[17][0]==32U);
    uint8_t frame[32],out[32];
    CO2_Packet p={.sequence=1,.uptime_ms=tick,.sample_age_ms=123,.sample_count=1,
                  .node=1,.ppm=850,.type=CO2_LINK_SAMPLE,.flags=1,.status=CO2_GOOD};
    CO2_Encode(frame,&p);
    for(unsigned i=0;i<32;i++) for(unsigned bit=0;bit<8;bit++) {
        uint8_t corrupt[32]; CO2_Packet decoded;
        memcpy(corrupt,frame,32U); corrupt[i]^=(uint8_t)(1U<<bit);
        assert(!CO2_Decode(corrupt,&decoded));
    }
    NRF24_Queue(&tx,frame,1U); Advance(&tx,&rx,10U);
    assert(tx.last_tx_result==NRF24_TX_ACK && tx.completed_tag==1U && tx.stats.tx_completed==1U);
    assert(NRF24_Read(&rx,out) && !memcmp(frame,out,32U)); assert(!NRF24_Read(&rx,out));
    CO2_Snapshot cache={0}; assert(CO2_Accept(&cache,out,tick));
    CO2_Snapshot snapshot=CO2_GetSnapshot(&cache,tick);
    assert(snapshot.valid && snapshot.ppm==850U);
    uint32_t stamp=cache.last_valid_ms;
    assert(CO2_Accept(&cache,out,tick+20U)); assert(cache.last_valid_ms==stamp && cache.duplicates==1U);
    p.sequence++; p.uptime_ms++; p.status=CO2_UART_ERROR; CO2_Encode(frame,&p);
    CO2_Accept(&cache,frame,tick+30U); snapshot=CO2_GetSnapshot(&cache,tick+30U);
    assert(snapshot.ppm==850U && snapshot.has_valid_sample && !snapshot.valid);
    p.sequence++; p.uptime_ms++; p.status=CO2_GOOD; p.sample_age_ms=7000U; CO2_Encode(frame,&p);
    CO2_Accept(&cache,frame,tick+40U); assert(!CO2_GetSnapshot(&cache,tick+40U).valid);
    assert(CO2_GetSnapshot(&cache,tick+7000U).stale && CO2_GetSnapshot(&cache,tick+7000U).link_stale);
    p.type=CO2_LINK_TEST; p.sequence++; CO2_Encode(frame,&p); CO2_Accept(&cache,frame,tick);
    assert(cache.test_frames==1U && cache.ppm==850U);
    p.type=CO2_LINK_SAMPLE; p.status=CO2_GOOD; p.sample_age_ms=0;
    p.sequence=0; p.uptime_ms=1; CO2_Encode(frame,&p); assert(CO2_Accept(&cache,frame,tick));
    assert(CO2_GetSnapshot(&cache,tick).valid); /* Sender reboot is accepted. */
    /* Do not flush remaining RX packets; ring overflow drops oldest only. */
    unsigned flushes=r.flush_rx;
    for(unsigned i=0;i<6;i++) {
        p.sequence=i+10U; p.uptime_ms=tick; CO2_Encode(frame,&p);
        NRF24_Queue(&tx,frame,i+10U); Advance(&tx,&rx,10U);
    }
    assert(rx.count==3U && rx.stats.rx_dropped==3U && r.flush_rx==flushes);
    for(unsigned i=0;i<3;i++) { CO2_Packet d; assert(NRF24_Read(&rx,out)); assert(CO2_Decode(out,&d) && d.sequence==13U+i); }
    /* Auto-ACK failure is not mislabeled a successful send. */
    r.connected=0; NRF24_Queue(&tx,frame,99U); Advance(&tx,&rx,10U);
    assert(tx.last_tx_result==NRF24_TX_MAX_RT && tx.completed_tag==99U);
    r.connected=1; Advance(&tx,&rx,1200U); assert(rx.ready);
    t.stuck=1; unsigned completions=tx.stats.tx_completed;
    NRF24_Queue(&tx,frame,100U); Advance(&tx,&rx,120U);
    assert(tx.last_tx_result==NRF24_TX_TIMEOUT && tx.stats.tx_completed==completions+1U && !tx.ready);
    t.stuck=0; Advance(&tx,&rx,1200U); assert(tx.ready);
    t.fail=1; Advance(&tx,&rx,1100U); assert(tx.stats.spi_errors && !tx.ready);
    Advance(&tx,&rx,1200U); assert(tx.ready);
    /* A stuck/floating chip never passes configuration read-back checks. */
    t.connected=0; Advance(&tx,&rx,1200U); assert(!tx.ready && tx.stats.spi_errors);
    t.connected=1; Advance(&tx,&rx,1200U); assert(tx.ready);
    printf("PASS%s: matching RF profile, ACK/MAX_RT/timeout, bounded recovery, FIFO/ring, CRC/version, reboot/wrap, last-good and stale\n",argc>1?" (wrap)":"");
    return 0;
}
