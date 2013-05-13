//#define timer0H 0x4c  //设置定时器的时间，公式65536-timer*FOSC/12，然后来进行填充
//#define timer0L 0x00  //现在这个设置在11.0592M的晶振下为0.05s
#define timer0H 0x24  //设置13.5MHz晶体时，定时器0为50ms
#define timer0L 0x45

void InitT0();