#ifndef CO2_LINK_H
#define CO2_LINK_H
#include <stdint.h>
#include <string.h>
#define CO2_LINK_SIZE 32U
#define CO2_LINK_NODE 1U
#define CO2_LINK_STALE_MS 6000U
#define CO2_LINK_REPORT_MS 2000U
#define CO2_LINK_SAMPLE 1U
#define CO2_LINK_TEST 2U
#define CO2_LINK_HAS_SAMPLE 1U
typedef enum { CO2_GOOD=0, CO2_WARMUP, CO2_TIMEOUT, CO2_UART_ERROR,
               CO2_BAD_HEADER, CO2_BAD_CHECKSUM, CO2_NOT_READY } CO2_Status;
typedef struct {
    uint32_t sequence, uptime_ms, sample_age_ms, sample_count;
    uint16_t node, ppm;
    uint8_t type, flags, status;
} CO2_Packet;
typedef struct {
    uint32_t last_packet_ms,last_valid_ms,last_sequence,last_uptime;
    uint32_t accepted,duplicates,rejected,test_frames;
    uint16_t ppm;
    uint8_t has_packet,has_valid_sample,last_status,stale,link_stale,valid,latest_valid;
} CO2_Snapshot;
static inline void CO2_Put16(uint8_t *p,uint16_t v) { p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); }
static inline void CO2_Put32(uint8_t *p,uint32_t v) { for(uint8_t i=0;i<4U;i++) p[i]=(uint8_t)(v>>(i*8U)); }
static inline uint16_t CO2_Get16(const uint8_t *p) { return (uint16_t)(p[0]|((uint16_t)p[1]<<8)); }
static inline uint32_t CO2_Get32(const uint8_t *p) { return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24); }
static inline uint16_t CO2_CRC(const uint8_t *p,uint8_t length)
{
    uint16_t crc=0xFFFFU;
    while(length--) {
        crc^=(uint16_t)*p++<<8;
        for(uint8_t bit=0;bit<8U;bit++) crc=(uint16_t)((crc&0x8000U)?((uint32_t)crc<<1)^0x1021U:((uint32_t)crc<<1));
    }
    return crc;
}
static inline void CO2_Encode(uint8_t out[32],const CO2_Packet *p)
{
    memset(out,0,32U);
    out[0]='N'; out[1]='R'; out[2]=1U; out[3]=p->type;
    out[4]=p->flags; out[5]=p->status; CO2_Put16(out+6,p->node);
    CO2_Put32(out+8,p->sequence); CO2_Put32(out+12,p->uptime_ms);
    CO2_Put32(out+16,p->sample_age_ms); CO2_Put16(out+20,p->ppm);
    CO2_Put32(out+24,p->sample_count); CO2_Put16(out+30,CO2_CRC(out,30U));
}
static inline uint8_t CO2_Decode(const uint8_t in[32],CO2_Packet *p)
{
    if(in[0]!='N' || in[1]!='R' || in[2]!=1U ||
       (in[3]!=CO2_LINK_SAMPLE && in[3]!=CO2_LINK_TEST) ||
       (in[4]&~CO2_LINK_HAS_SAMPLE) || in[5]>CO2_NOT_READY ||
       in[22] || in[23] || in[28] || in[29] ||
       CO2_Get16(in+30)!=CO2_CRC(in,30U)) return 0U;
    p->type=in[3]; p->flags=in[4]; p->status=in[5]; p->node=CO2_Get16(in+6);
    p->sequence=CO2_Get32(in+8); p->uptime_ms=CO2_Get32(in+12);
    p->sample_age_ms=CO2_Get32(in+16); p->ppm=CO2_Get16(in+20);
    p->sample_count=CO2_Get32(in+24);
    return p->node==CO2_LINK_NODE;
}
static inline uint8_t CO2_Accept(CO2_Snapshot *s,const uint8_t frame[32],uint32_t now)
{
    CO2_Packet p;
    if(!CO2_Decode(frame,&p)) { s->rejected++; return 0U; }
    if(p.type==CO2_LINK_TEST) { s->test_frames++; return 1U; }
    if(s->has_packet && s->last_sequence==p.sequence && s->last_uptime==p.uptime_ms) {
        s->duplicates++; s->last_packet_ms=now; return 1U;
    }
    /* The pair tolerates sender reboot and 32-bit wraps. One sender owns this link. */
    s->has_packet=1U; s->accepted++; s->last_packet_ms=now;
    s->last_sequence=p.sequence; s->last_uptime=p.uptime_ms; s->last_status=p.status;
    s->latest_valid=(p.flags&CO2_LINK_HAS_SAMPLE) && p.status==CO2_GOOD && p.sample_age_ms<=CO2_LINK_STALE_MS;
    if(s->latest_valid) {
        s->ppm=p.ppm; s->has_valid_sample=1U;
        s->last_valid_ms=now-p.sample_age_ms;
    }
    return 1U;
}
static inline CO2_Snapshot CO2_GetSnapshot(const CO2_Snapshot *stored,uint32_t now)
{
    CO2_Snapshot s=*stored;
    s.stale=!s.has_valid_sample || (uint32_t)(now-s.last_valid_ms)>CO2_LINK_STALE_MS;
    s.link_stale=!s.has_packet || (uint32_t)(now-s.last_packet_ms)>CO2_LINK_STALE_MS;
    s.valid=s.latest_valid && s.has_valid_sample && !s.stale && !s.link_stale && s.last_status==CO2_GOOD;
    return s;
}
#endif
