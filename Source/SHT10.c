#include "SENSOR.h"
#include "iocc2530.h"

extern void Sensor_PIN_INT(void);

char uart_buffer;

uint8 ReadTemp(void)
{
   uint8 temp;
   unsigned long lTemp;
    float RHTValue;
    unsigned char buf[8];
    P0DIR |= 0x01;  //设置P0.0为输出方式；P0.4和P0.5为输入方式
    P2DIR |= 0x01;  //设置P2.0为输出方式
    P0_0 = 1; 
    P2_0 = 1;       //熄灭LED
    CLKCONCMD &= ~0x40;          //选择32M晶振
    while(!(SLEEPSTA & 0x40));   //等待XSOC稳定
    CLKCONCMD = 0xb8;            //TICHSPD 128分频，CLKSPD 不分频
    SLEEPCMD |= 0x04;            //关闭不用的RC 振荡器
    PERCFG = 0x00;               //位置1 P0 口
    P0SEL = 0x3c;                //P0 用作串口
    U0CSR |= 0x80;               //UART 方式
    U0GCR |= 10;                 //baud_e = 10;
    U0BAUD |= 216;               //波特率设为57600
    UTX0IF = 1;
    U0CSR |= 0X40;               //允许接收
    IEN0 |= 0x84;                //开总中断，接收中断  	
    Sensor_PIN_INT();

    while(1){ 
	lTemp = ReadSHT1(3);//14bit温度
        lTemp = lTemp >> 8;
        RHTValue = lTemp;
        RHTValue = 0.01 * RHTValue - 39.64;
        buf[0] = (uint8)RHTValue;//温湿度传感器温度 
	buf[0] = ( ((buf[0]/10)<<4) + (buf[0]%10) );
        buf[1] = (buf[0]>>4)&0xf;
	if(buf[1] > 0x9)
            buf[1] = buf[1] - 0XA + 'A';
	else
	    buf[1] = buf[1] + '0';
	buf[2] = (buf[0])&0xf;
	if(buf[2] > 0x9)
	    buf[2] = buf[2] -0XA + 'A';
	else
	    buf[2] = buf[2] + '0';
        

        temp = (uint8)RHTValue;
    
        return temp;
}
}