

#include "N79E81x.h"
#include"VOICE.h"


//延时 X 毫秒
void delay_ms(unsigned int count)
{
	unsigned int i,j;
	for(i=0;i<count;i++)
	{
		for(j=0;j<500;j++);
	}
}




//延时 X 微秒
void delay_us(unsigned int count)
{
	unsigned int i,j;
	for(i=0;i<count;i++)
	{
		for(j=0;j<2;j++);
	}
}
  
//控制地址段放音
void SC_Speech(unsigned char cnt)
{
	unsigned char i;
//	SC_RST=1;
//	delay_ms(15); //DAC, 大于 32 段为 15MS
	SC_RST=0;
	delay_ms(30);
	SC_RST=1;
	delay_ms(30);
	for(i=0;i < cnt;i++)
	{
		SC_DATA=1; // 数据脉冲高
		delay_us(250); // 延时 100US
		SC_DATA=0; // 数据脉冲低
		delay_us(250); // 延时 100US
	}
}

void noVoice()
{
	P14=0;
	SC_DATA=0;
	SC_RST=0;
	delay_us(1000); // 上电复位防止干扰发声
	SC_RST=1;
	delay_us(1000);
	P14=1;
}


