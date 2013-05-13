
#define timer1H 0xff  //设置定时器的时间，公式(65536-timer)*FOSC/12，然后来进行填充
#define timer1L 0x8e  //现在这个设置在11.0592M的晶振下为0.000999s


void InitT1();//初始化定时器01