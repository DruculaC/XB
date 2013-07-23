#include"N79E81x.h"
#include<intrins.h>
#include"AD.h"
#include"T0.h"
#include"voice.h"
#include"pwm.h"
#include"T1.h"

//定义通信命令

#define CmdStart 0x00 //开机命令
#define CmdStop 0x01  //关机命令

#define ComMode_1 0xc1 //通信模式1 
#define ComMode_2 0xc2 //通信模式2
#define ComMode_3 0xc3 //通信模式3
#define ComMode_4 0xc4 //抬起指令
#define ComMode_5 0xc5 //倒地指令

#define Succeed 0xce  //通信成功
#define Wrong 0xff    //通信失败

#define CmdHead 0xc8
#define CmdHead1 0x33 //数据帧的首部1, 00110011,11
#define CmdHead2 0xcc //数据帧的首部2,11001100,00
#define CmdHead3 0x3c //数据帧的首部3,11000011,01
#define CmdHead4 0xcc //数据帧的首部4,11001100,00

#define MyAddress 0xe0
#define MyAddress1 0x33 //本机地址1, 00110011,11
#define MyAddress2 0x3c //本机地址2, 11000011,01
#define MyAddress3 0xcc //本机地址3,11001100,00
#define MyAddress4 0xcc //本机地址4,11001100,00

sbit ModeControl_1=P2^6;	//无线发射机模式控制，1为小功率，0为大功率
sbit tran_en=P2^7;		 	//无线发射机使能控制，1为打开发射机，0为关闭发射机
sbit Moto=P2^4;		   		//马达控制端，1马达不振动，0马达振动
sbit PAshutdown=P1^4;		//功放开关控制，1为打开功放，0为关闭功放
sbit receive_en=P1^3;		//接收机使能，要加上拉电阻

bit receiveFlag=0;	//接收到数据标志
bit commuFlag=0;	//开启通信标志，1表示开始通信，0表示没有通信
bit alarmFlag2=0;	//编码2报警标志
bit alarmFlag3=0;	//编码3报警标志
bit alarmFlag4=0;	//抬起报警标志
bit alarmFlag5=0;	//倒地报警标志
unsigned char alarmCount2=0;	//报警2循环次数
unsigned char alarmCount3=0;	//报警3循环次数
unsigned char alarmCount4=0;	//抬起报警循环次数
unsigned char alarmCount5=0;	//倒地报警循环次数

unsigned char count=0;			//串口接收部分的计数器
unsigned int time0Count_3=0;	//定时器T0的计数

unsigned char TestFlag=0;		//每3s计数加1，如果通信成功，则将其归零。设置如果连续n次都没有归零，则说明不在场了

//作为接收和发送的缓存区
unsigned char TxRxBuf[28]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
//一个头字节，一个地址字节，一个命令字节，两个编码地址字节，两个编码
unsigned char myTxRxData[7]={0x00,0x00,0x00,0x00,0x00,0x00,0x00};//处理完后的通信数据的缓冲区
 
//通信数据定义
unsigned char DataBetween=0;	//作为接收数据的中间变量
unsigned char RecData=0;		//接收到的数据
unsigned char DataTime=0;		//作为接收的数据的移位次数计数
bit ComFlag=1;					//检测上跳沿和下跳沿的标识
unsigned char T1highcount=0;	//定时器T1在没有信号到来的时候，对高电平计数，一旦超过某个值，则将Datatime清0

unsigned int Check=0;			//作为AD检测值，检测电池电量
 
unsigned char powerflag=1;		//定义电压报警标志位


void ComMode_1_Data(void);		//编码1函数

void Delay3(unsigned int t)		//t=1时，延迟100us左右
{
	unsigned int i,j;
	for(i=0;i<t;i++)		
	for(j=0;j<23;j++);
}

void initsignal()				//init signal，发送编码信号前的起始信号，用于将接收机的自动增益打开
{
	unsigned char k,k1;
	unsigned char mystartbuffer=0xaa;
	for(k1=0;k1<3;k1++)
	{
		for(k=0;k<1;k++)
		{
			if((mystartbuffer&0x80)==0x80)//为1
			{
				P10=0;
				Delay3(80);		//延时4.5ms以上，由于定时器占用问题，只能用这种延时来实现
			}
			else//为0的情况
			{
				P10=0;
				Delay3(80);		//延时2ms，由于定时器占用问题，只能用这种延时来实现
			}
			P10=1;//常态为高电平
			mystartbuffer<<=1;
			Delay3(150);		//延时要大于2ms
		}
		mystartbuffer=0xaa;
		Delay3(80);
	}
	P10=1;
}

void main(void)
{
	noVoice();			 	//开机不发声音

	InitT0();			 	//初始化定时器0和定时器1
	InitT1();

	Moto=1;					//关闭马达
	tran_en=0;	   			//关闭发射机
	receive_en=0;			//关闭接收机
	ET0=1;				 	//使能定时器0和1的中断
	ET1=1;
	PT1=1;				 	//设置定时器1为较高优先级
	EA=1;				 	//所有中断使能

	myPwm();				//方波输出打开

	P10=1;					//发射端口，先置为高

	PAshutdown=0;		  	//开机时，将功放关闭
	Check=GetADCResult(6);	//执行一次电池电量检测

	PAshutdown=1;
	SC_Speech(4);
	Delay(200);
	PAshutdown=0;
	commuFlag=1;//开启通信

	while(1)
	{
 		if((alarmFlag2==1)&&(alarmCount2<2))	//编码2开始相应的报警
		{
			alarmCount2++;

			PAshutdown=1;
			SC_Speech(1);
			Delay(160);
			PAshutdown=0;			

			Moto=0;		//开震动
			Delay(10);
			Moto=1;
		}
		if((alarmFlag3==1)&&(alarmCount3<2))	//编码3开始相应的报警
		{
			alarmCount3++;
			
			PAshutdown=1;
			SC_Speech(11);
			Delay(150);
			Moto=0;		//开震动
			Delay(20);
			Moto=1;
			PAshutdown=0;	
 		}

		if((powerflag==1)&&(Check<=0x35a))
		{
			powerflag=0;
			PAshutdown=1;
			SC_Speech(7);	//电压不充足提示
			Delay(120);
			PAshutdown=0;
		}
		else if((powerflag==0)&&(Check>=0x377))
		{
			powerflag=1;
			PAshutdown=1;
			SC_Speech(6);	//电压充足提示
			Delay(120);
			PAshutdown=0;
		}
	}
}

void timeT1() interrupt 3 		//定时器1中断接收数据
{
//	unsigned int newAddr=0;
	TH1=timer1H;				//重装载
	TL1=timer1L;
   
	if(P11==0)					//正常情况为高电平,有低电平说明有信号
	{
		DataBetween++;
		ComFlag=0;
		if(DataBetween==150)	//低电平持续的最大时间	
		{
			DataBetween=0;
		}
	}
	else						//为高电平了
	{
		if(ComFlag==0)			//说明有一个低电平
		{
			ComFlag=1;
			if((DataBetween>60)&&(DataBetween<=100))	//低电平持续的时间小于10ms，则为0
			{
				RecData<<=1;
				RecData &= 0xfe;
				DataTime++;
				T1highcount=0;
			}
			else if((DataBetween>100))					//低电平持续的时间大于10ms，则为1
			{
				RecData<<=1;
				RecData |= 0x01;
				DataTime++;
				T1highcount=0;
			}
			else
			{
				T1highcount++;	
			}
			DataBetween=0;
		}
		else
		{
			T1highcount++;
			if(T1highcount>=120)
			{
				DataTime=0;
				ComFlag=1;
				count=0;
			}		
		}
	}

	if(DataTime==8)					//说明一个字节的数据已经接受完全
	{
		DataTime=0;
		myTxRxData[count]=RecData;
		if(count==0&&myTxRxData[0]==CmdHead)
		{
			count=1;
		}
		else if(count==1&&myTxRxData[1]==MyAddress)
		{
			count=2;
		}
		else if(count>=2&&count<=5)
		{
			count++;
		}
		else if(count==6)
		{
			receiveFlag=1;
			count=0;
		}
		else 
		{
			count=0;
		}
	}

	if(receiveFlag==1)			//说明接收到了数据，开始处理
	{
		receiveFlag=0;			//清接收标志
		receive_en=0;			//关闭接收机
		switch(myTxRxData[2])	//解析指令
		{
			case ComMode_2:		//说明在30m内，正常，不用报警
			{
				TestFlag=0;		//清超时标志
				alarmCount2=0;	//清报警计数器
				alarmFlag2=0;	//清报警标志
				alarmCount3=0;	//清报警计数器
				alarmFlag3=0;	//清报警标志
				Moto=0;//开震动
				Delay(10);
				Moto=1;
			}
			break;
			
			case ComMode_3:		//接到编码3信号，开始报警
			{
				TestFlag=0;		//清超时标志				
				alarmFlag3=1;
				alarmCount2=0;	//清报警计数器
				alarmFlag2=0;	//清报警标志			
			}
			break;
		}
	}
//	}
}

void time0() interrupt 1		//作为整个系统自己的时钟
{
	TH0=timer0H;				//重装载
	TL0=timer0L;

	time0Count_3++;

	if(time0Count_3>=60)		//串口每3S发送一次的数据的时间标志
	{
		if(commuFlag==1)		//说明开启了通信
		{
			receive_en=0;
			ComMode_1_Data();	//发送模式1信号
			receive_en=1;		//打开接收机
			TestFlag++;
			if(TestFlag==6)		//连续数次没有收到编码2，则表示主机丢失
			{
				alarmFlag2=1;
				//加入相应处理代码	
			}
 		}

		Check=GetADCResult(6);	//每隔3s做一次电量检测
		time0Count_3=0;			//3s的重新计数
	}
}

void ComMode_1_Data()			//发送编码1
{
	unsigned char i,n;

	ModeControl_1=0;			//大功率发射模式
	tran_en=1;
	myTxRxData[0]=CmdHead;
	myTxRxData[1]=MyAddress;
	myTxRxData[2]=ComMode_1;
	myTxRxData[3]=0x00;
	myTxRxData[4]=0x00;
	myTxRxData[5]=0x00;
	myTxRxData[6]=0x00;

	initsignal();
	for(i=0;i<7;i++)
	{
		for(n=0;n<8;n++)
		{
			if((myTxRxData[i]&0x80)==0x80)		//为1
			{
				P10=0;
				Delay3(120);		//延时4.5ms以上，由于定时器占用问题，只能用这种延时来实现
			}
			else//为0的情况
			{
				P10=0;
				Delay3(80);			//延时2ms，由于定时器占用问题，只能用这种延时来实现
			}
			P10=1;//常态为高电平
			myTxRxData[i]<<=1;
			Delay3(50);//延时要大于2ms
		}
	}
	tran_en=0;
}