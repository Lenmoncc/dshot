#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <wiringPi.h>
#include <stdbool.h>
#include <time.h>

#define Throttle 1046
#define PIN  1
#define MODE OUTPUT
//#define Freq 2.67   
//#define A_BIT_TIME 1.67 //  时间按比例放缩从ns到us 高低电平比例约位2:1
#define HIGH_TIME 22
#define LOW_TIME 11
#define high 1
#define low 0
#define ESC_BUFF_LEN 18
uint16_t ESC_DATA[ESC_BUFF_LEN]={0};
#define ESC_BIT_1 1
#define ESC_BIT_0 0


//throttle范围：0-2047 1046
uint16_t data_dealing(uint16_t throttle)
{
    throttle=throttle<2048?throttle:2047;//油门值 前11位
    bool telemetry=false;
    uint16_t value;
    value = (throttle << 1) | (telemetry ? 1 : 0);//value为前12的值

    unsigned crc_sum=0;
    unsigned crc_predata=value;
    //计算crc校验和
    crc_sum^=crc_predata;
    crc_predata>>4;
    crc_sum^=crc_predata;
    crc_predata>>4;
    crc_sum^=crc_predata;

    crc_sum &= 0xF;
    value = (value<<4)|crc_sum; //value为16位值
    
    return value;
}

void esc_cmd(uint16_t *ESC_DATA,uint16_t throttle)
{
    uint16_t value = data_dealing(throttle);
    //for(int i=0;i<16;i++)
    //{
        // ESC_DATA[i] = (value & (1 << (15 - i))) ? ESC_BIT_1 : ESC_BIT_0;
    //}
    ESC_DATA[0]  = (value & 0x8000) ? ESC_BIT_1 : ESC_BIT_0;//1000 0000 0000 0000
    ESC_DATA[1]  = (value & 0x4000) ? ESC_BIT_1 : ESC_BIT_0;//0100 0000 0000 0000
    ESC_DATA[2]  = (value & 0x2000) ? ESC_BIT_1 : ESC_BIT_0;//0010 0000 0000 0000
    ESC_DATA[3]  = (value & 0x1000) ? ESC_BIT_1 : ESC_BIT_0;
    ESC_DATA[4]  = (value & 0x0800) ? ESC_BIT_1 : ESC_BIT_0;
    ESC_DATA[5]  = (value & 0x0400) ? ESC_BIT_1 : ESC_BIT_0;
    ESC_DATA[6]  = (value & 0x0200) ? ESC_BIT_1 : ESC_BIT_0;
    ESC_DATA[7]  = (value & 0x0100) ? ESC_BIT_1 : ESC_BIT_0;
    ESC_DATA[8]  = (value & 0x0080) ? ESC_BIT_1 : ESC_BIT_0;
    ESC_DATA[9]  = (value & 0x0040) ? ESC_BIT_1 : ESC_BIT_0;
    ESC_DATA[10] = (value & 0x0020) ? ESC_BIT_1 : ESC_BIT_0;
    ESC_DATA[11] = (value & 0x0010) ? ESC_BIT_1 : ESC_BIT_0; 	
    ESC_DATA[12] = (value & 0x8) ? ESC_BIT_1 : ESC_BIT_0;//
    ESC_DATA[13] = (value & 0x4) ? ESC_BIT_1 : ESC_BIT_0;
    ESC_DATA[14] = (value & 0x2) ? ESC_BIT_1 : ESC_BIT_0;
    ESC_DATA[15] = (value & 0x1) ? ESC_BIT_1 : ESC_BIT_0;
   

    ESC_DATA[16]=0;
    ESC_DATA[17]=0;

}

void ns_delay(long time)
{
    struct timespec ts;
    ts.tv_sec=0;
    ts.tv_nsec=time;
    nanosleep(&ts,NULL);
    int ret = nanosleep(&ts,NULL);
    if(ret<0)
    {
        perror("nanosleep");
    }
}

void sendbit(uint16_t *ESC_DATA)
{
    for(int i=0;i<18;i++)
    {
    int bit=ESC_DATA[i];
    if(bit)
    {
    digitalWrite(PIN,1);
    delay(HIGH_TIME);
    //ns_delay(HIGH_TIME);
    //delayMicroseconds(HIGH_TIME);
    digitalWrite(PIN,0);
    delay(LOW_TIME);
    //ns_delay(LOW_TIME);
    //delayMicroseconds(LOW_TIME);
    }
    else
    {
    digitalWrite(PIN,1);
    delay(LOW_TIME);
    //ns_delay(LOW_TIME);
    //delayMicroseconds(LOW_TIME);
    digitalWrite(PIN,0);
    delay(HIGH_TIME);
    //ns_delay(HIGH_TIME);
    //delayMicroseconds(HIGH_TIME);
    }
    }
}

int main()
{
    int state;
    state=wiringPiSetup();
    if(state<0)
    {
        printf("fail to initialize\n");
    }

    pinMode(PIN,MODE);
    esc_cmd(ESC_DATA,Throttle);

    while (1)
    {
        sendbit(ESC_DATA);

        delay(30);
        //delayMicroseconds(3);
    }
    

}



