


#include"N79E81x.h"
#include<intrins.h>
#include"AD.h"







/*
��������Delay
���ܣ���ʱ
˵����
��ڲ�����WORD n
����ֵ����
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
�������� void InitADC(void)
���ܣ� 	��ʼ��ADC
˵����
��ڲ�������
����ֵ��  ��
*/
//void InitADC(void)
//{
//	P1M0=0x03;
//	P1M1=0x03;//����P1.0��P1.1��Ϊ����̬������AD
//	P1=0xff;//��ʼ��P1��
//	ADC_DATA=0;
//	ADC_CONTR=ADC_POWER|ADC_SPEEDLL;
//	Delay(2);
//}



/*
��������  GetADCResult
���ܣ� 	��ȡָ��ͨ����ADCת��ֵ
˵����
��ڲ����� ָ����ͨ��
����ֵ��   ���ص�ADֵ
*/
unsigned char GetADCResult(unsigned char ch)
{
	unsigned char DataL=0;
	unsigned char DataH=0;
	if(ch==5)//���ʰ�����ĵ�����С
	{
		P0DIDS|=0x20; // Set ADC0 (P0.5 default) is input only mode
//		P0M1&=0xe7;
//		P0M2&=0xe7;

		AADR0=0;	//ѡ��ͨ��P0.5
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
	else if(ch==6) //��ؼ��
	{
		P0DIDS|=0x40; // Set ADC0 (P0.6 default) is input only mode
//		P0M1&=0xe7;
//		P0M2&=0xe7;

		AADR0=1;	//ѡ��ͨ��P0.6
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



