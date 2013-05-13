


#include"N79E81x.h"
#include<intrins.h>
#include"AD.h"







/*
函数名：Delay
功能：延时
说明：
入口参数：WORD n
返回值：无
*/
void Delay(unsigned int n)
{
	unsigned int x;
	while(n--)
	{
		x=5000;
		while(x--);
	}
}


/*
函数名： void InitADC(void)
功能： 	初始化ADC
说明：
入口参数：无
返回值：  无
*/
//void InitADC(void)
//{
//	P1M0=0x03;
//	P1M1=0x03;//设置P1.0和P1.1口为高阻态，进行AD
//	P1=0xff;//初始化P1口
//	ADC_DATA=0;
//	ADC_CONTR=ADC_POWER|ADC_SPEEDLL;
//	Delay(2);
//}



/*
函数名：  GetADCResult
功能： 	读取指定通道的ADC转换值
说明：
入口参数： 指定的通道
返回值：   读回的AD值
*/
unsigned char GetADCResult(unsigned char ch)
{
	unsigned char DataL=0;
	unsigned char DataH=0;
	if(ch==5)//检测拾音器的电量大小
	{
		P0DIDS|=0x20; // Set ADC0 (P0.5 default) is input only mode
//		P0M1&=0xe7;
//		P0M2&=0xe7;

		AADR0=0;	//选择通道P0.5
		AADR1=0;
		AADR2=1;
	                                                                  
	    ADCCON1|=0x80;                                          // Enable ADC Function   
		
		ADCI=0;                                           // Clear ADC flag (ADCI=0)
    	ADCS=1;  
		
		DataL=ADCCON0;
		DataL=DataL>>6; 

		DataH=ADCH;
		DataH=(DataH<<2)+DataL;

		return DataH;
	}
	else if(ch==6) //电池检测
	{
		P0DIDS|=0x40; // Set ADC0 (P0.6 default) is input only mode
//		P0M1&=0xe7;
//		P0M2&=0xe7;

		AADR0=1;	//选择通道P0.6
		AADR1=0;
		AADR2=1;
	                                                                  
	    ADCCON1|=0x80;                                          // Enable ADC Function   
		
		ADCI=0;                                           // Clear ADC flag (ADCI=0)
    	ADCS=1;  
		
		DataL=ADCCON0;
		DataL=DataL>>6; 

		DataH=ADCH;
		DataH=(DataH<<2)+DataL;

		return DataH;
	}

}



