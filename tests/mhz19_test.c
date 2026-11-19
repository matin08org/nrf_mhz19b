#include "mhz19.h"
#include "co2_link.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static uint32_t now;
static uint8_t *rx;
uint32_t HAL_GetTick(void){return now;}
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *h,uint8_t *p,uint16_t n){(void)h;assert(n==9);rx=p;return HAL_OK;}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *h,uint8_t *p,uint16_t n){(void)h;assert(n==9&&p[2]==0x86&&p[8]==0x79);return HAL_OK;}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *h){(void)h;return HAL_OK;}
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *h){(void)h;return HAL_OK;}
static void Reply(MHZ19_Handle *s,UART_HandleTypeDef *u,uint16_t ppm,int corrupt)
{
    assert(MHZ19_Request(s)==MHZ19_STATUS_OK);
    uint8_t f[9]={0xff,0x86,(uint8_t)(ppm>>8),(uint8_t)ppm,65,0,0,0,0};
    uint8_t sum=0;for(unsigned i=1;i<8;i++)sum=(uint8_t)(sum+f[i]);f[8]=(uint8_t)(0U-sum);
    if(corrupt)f[8]^=1;
    memcpy(rx,f,9);MHZ19_UART_RxCpltCallback(s,u);MHZ19_Process(s);
}
int main(void)
{
    UART_HandleTypeDef u={0};MHZ19_Handle s;MHZ19_Init(&s,&u);
    assert(MHZ19_Request(&s)==MHZ19_STATUS_WARMING_UP);
    now=180000;Reply(&s,&u,5000,0);uint16_t ppm=0;
    assert(MHZ19_GetCO2(&s,&ppm)&&ppm==5000&&s.last_frame_ppm==5000);
    assert(s.last_frame[2]==0x13&&s.last_frame[3]==0x88&&s.last_frame_checksum_ok);
    now+=2000;Reply(&s,&u,420,0);
    assert(MHZ19_GetCO2(&s,&ppm)&&ppm==420); /* no software clamp/stuck-at-5000 */
    now+=2000;Reply(&s,&u,5000,1);
    assert(s.bad_checksums==1&&s.last_frame_ppm==5000&&!s.last_frame_checksum_ok);
    assert(s.co2_ppm==420&&s.valid_frames==2&&s.frames_received==3);
    CO2_Packet p={.node=CO2_LINK_NODE,.type=CO2_LINK_SAMPLE,.flags=CO2_LINK_HAS_SAMPLE,.status=CO2_GOOD,.ppm=420};
    uint8_t frame[32];CO2_Encode(frame,&p);CO2_Snapshot snap={0};
    assert(CO2_Accept(&snap,frame,now)&&CO2_GetSnapshot(&snap,now).ppm==420);
    p.ppm=5000;p.sequence++;CO2_Encode(frame,&p);
    assert(CO2_Accept(&snap,frame,now)&&CO2_GetSnapshot(&snap,now).ppm==5000);
    assert(MHZ19_Request(&s)==MHZ19_STATUS_OK);now+=500;MHZ19_Process(&s);
    assert(s.status==MHZ19_STATUS_TIMEOUT&&s.timeouts==1);
    assert(s.last_frame_ppm==5000&&s.co2_ppm==420); /* raw rejected frame is not a valid sample */
    puts("PASS MH-Z19: warmup, exact 5000/420 decode, bad-checksum rejection, stable raw diagnostics, RF roundtrip and timeout");
}
