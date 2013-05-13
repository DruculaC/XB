
typedef unsigned char BYTE;
typedef unsigned int  WORD;


//#define FOSC 11059200L
//#define BAUD 9600
#define FOSC 13500000L
#define BAUD 600

 extern unsigned char code PassWord[];
 void InitUART();

 void SendData(unsigned char dat);
 void SendNByte(unsigned char *buf,unsigned char len);

 unsigned int select(unsigned char receData);//查询函数，查询密码表里面的密码对的住的就返回此密码的地址