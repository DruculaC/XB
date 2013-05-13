

#define FOSC 13500000L
#define BAUD 600

extern unsigned char code PassWord[];

void InitUART();

void SendData(unsigned char dat);

void SendNByte(unsigned char *buf,unsigned char len);