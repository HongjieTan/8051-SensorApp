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
    P0DIR |= 0x01;  //����P0.0Ϊ�����ʽ��P0.4��P0.5Ϊ���뷽ʽ
    P2DIR |= 0x01;  //����P2.0Ϊ�����ʽ
    P0_0 = 1; 
    P2_0 = 1;       //Ϩ��LED
    CLKCONCMD &= ~0x40;          //ѡ��32M����
    while(!(SLEEPSTA & 0x40));   //�ȴ�XSOC�ȶ�
    CLKCONCMD = 0xb8;            //TICHSPD 128��Ƶ��CLKSPD ����Ƶ
    SLEEPCMD |= 0x04;            //�رղ��õ�RC ����
    PERCFG = 0x00;               //λ��1 P0 ��
    P0SEL = 0x3c;                //P0 ��������
    U0CSR |= 0x80;               //UART ��ʽ
    U0GCR |= 10;                 //baud_e = 10;
    U0BAUD |= 216;               //��������Ϊ57600
    UTX0IF = 1;
    U0CSR |= 0X40;               //�������
    IEN0 |= 0x84;                //�����жϣ������ж�  	
    Sensor_PIN_INT();

    while(1){ 
	lTemp = ReadSHT1(3);//14bit�¶�
        lTemp = lTemp >> 8;
        RHTValue = lTemp;
        RHTValue = 0.01 * RHTValue - 39.64;
        buf[0] = (uint8)RHTValue;//��ʪ�ȴ������¶� 
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