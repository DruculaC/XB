#include "N79E81x.h"
#include"T1.h"


void InitT1()
{
	TMOD=0x11;//��ʱ��1����1��ʽ
	TH1=timer1H;
	TL1=timer1L;
	TR1=1;
}