
//#define timer1H 0xff  //设置定时器的时间，公式65536-timer*FOSC/12，然后来进行填充
//#define timer1L 0xa4  //现在这个设置在11.0592M的晶振下为0.000999s
#define timer1H 0xff  //设置定时器的时间，公式(65536-timer*FOSC/12)/256，timer为需要定时的时间，其中余数×256就是timer1L的值
#define timer1L 0x8e  //现在这个设置在13.5M的晶振下为100us


void InitT1();//初始化定时器01