


/************** 端口定义 *****************/
sbit SC_RST=P2^0; //P2.0 是脉冲复位脚
sbit SC_DATA=P2^1; //P2.1 是脉冲数据脚

void delay_us(unsigned int count);
void SC_Speech(unsigned char cnt);
void noVoice();