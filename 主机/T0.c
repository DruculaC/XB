

#include"N79E81x.h"
#include<intrins.h>
#include"T0.h"


void InitT0()
{
	TMOD=0x21;//��ʱ��0����1��ʽ
	TH0=timer0H;//
	TL0=timer0L;	  //ԼΪ100us
	TR0=1;
}

