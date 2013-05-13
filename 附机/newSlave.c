

#include"N79E81x.h"
#include<intrins.h>
#include"AD.h"
#include"UART.h"
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

//三路循环
sbit onePin=P1^6;
sbit twoPin=P1^7;
sbit threePin=P0^0;

//附机的发射部分的控制端口
//sbit PWMout=P3^5;//发射机的方波输出口，现在使用PWM外设了
sbit ModeControl_1=P2^6;	//发射机模式控制,0为大功率，1为小功率

//接收机控制
sbit SwitchControl=P1^3;	//1关闭接收机，0开启接收机

//开关按键
sbit Turn=P0^3;	

//模式选择按键
sbit ModeChange=P0^4;

//马达控制端
sbit Moto=P2^4;

//电池控制 	AD的1号通道为电池的电量检测端
sbit BatteryControl=P1^2;	//控制电池是否充电的开关，0为充电，1为不充电

//开机状态标记位
bit TurnFlag=0;	//0为关机状态，1为开机状态

//模式选择位，0则用模式1,1则用模式2
bit ModeFlag=0;

bit receiveFlag=0;	//接收到数据标志
bit commuFlag=0;	//开启通信标志，1表示开始通信，0表示没有通信

bit alarmFlag2=0;	//编码2报警标志
bit alarmFlag3=0;	//编码3报警标志
bit alarmFlag4=0;	//抬起报警标志
bit alarmFlag5=0;	//倒地报警标志
unsigned char alarmCount2=0;//报警2循环次数
unsigned char alarmCount3=0;//报警3循环次数
unsigned char alarmCount4=0;//抬起报警循环次数
unsigned char alarmCount5=0;//倒地报警循环次数

bit threeFlag=0;	//三路循环开关标志

unsigned char voiceFlag=0;	//声音循环开关 

unsigned char dataFirst=0;	//用于存储上次编码类型

unsigned char count=0;	//串口接收部分的计数器

unsigned int time0Count_3=0;	//定时器T0的计数

unsigned int lastAddr=0;	//上一次接收到的编码的地址
unsigned char TestFlag=0;	//1、2、3分别为每1S后的计数，在串口的成功指令里会执行将去归零的操作
                			//如果连续3次都没有归零，则说明不在场了

//作为接收和发送的缓存区
unsigned char TxRxBuf[28]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
//一个头字节，一个地址字节，一个命令字节，两个编码地址字节，两个编码
unsigned char myTxRxData[7]={0x00,0x00,0x00,0x00,0x00,0x00,0x00};//处理完后的通信数据的缓冲区
 
//串行数据定义
unsigned char DataBetween=0;//作为接收数据的中间变量
unsigned char RecData=0;//接收到的数据
unsigned char DataTime=0;//作为接收的数据的移位次数计数
bit ComFlag=1;	//检测上跳沿和下跳沿的标识
unsigned char T1highcount=0;	   //定时器T1在没有信号到来的时候，对高电平计数，一旦超过某个值，则将Datatime清0

unsigned int Check=0;	//作为AD检测值，检测电池电量

//功放开关控制，1为打开功放，0为关闭功放
sbit PAshutdown=P1^4;

//定义一个计数，来表示信号接收后，多长时间使接收机打开，即控制SwitchControl的高电平时间。
unsigned int SwitchControlcount=0;

//定义电压报警标志位
unsigned char powerflag=1;

//用来控制所有报警的总开关
//unsigned char alarmflagall=0;	

//函数声明
//void codeData(unsigned char *doData,unsigned char len);		//编码 ,电平1变为0011，电平0变为1100
//void transCode(unsigned char *doData,unsigned char len);//解码，将接收到得数据还原
void ComMode_1_Data(void);//发送边码1

//开机函数
void StartAll(void);
//关机函数
void StopAll(void);

//t=1时，延迟100us左右
void Delay3(unsigned int t)
{
	unsigned int i,j;
	for(i=0;i<t;i++)		
	for(j=0;j<23;j++);
}

//init signal，发送编码信号前的起始信号，用于将接收机的自动增益打开
void initsignal()
{
	unsigned char k,k1;
	unsigned char mystartbuffer=0xaa;
	for(k1=0;k1<3;k1++)
	{
		for(k=0;k<8;k++)
		{
			if((mystartbuffer&0x80)==0x80)//为1
			{
				P10=0;
				Delay3(80);//延时4.5ms以上，由于定时器占用问题，只能用这种延时来实现
			}
			else//为0的情况
			{
				P10=0;
				Delay3(80);//延时2ms，由于定时器占用问题，只能用这种延时来实现
			}
			P10=1;//常态为高电平
			mystartbuffer<<=1;
			Delay3(150);//延时要大于2ms
		}
		mystartbuffer=0xaa;
		Delay3(150);
	}
	P10=1;
	Delay3(80);
}

void main(void)
{
	noVoice();			 //开机不发声音

//	InitUART();
	InitT0();			 //初始化定时器0和定时器1
	InitT1();
//	TI=0;
//	RI=0;

//	SwitchControl=0;	 //接收机的控制端为高电平，即关闭接收机 

	Turn=1;		  		//开机键置1，后面检测是否为0
	ModeChange=1;		//P04置1，后面检测是否为0，来改变模式，乘车模式和普通模式

	BatteryControl=0;	//附机上电的时候置0，即可以充电，电池在没有充满的情况下为低电平

	Moto=1;				//关闭马达
	SwitchControl=0;	 //开起接收机的控制端为高电平

//	ES=1;
	ET0=1;				 //使能定时器0和1的中断
	ET1=1;
//	PS=1;
	PT1=1;				 //设置定时器1为较高优先级
	EA=1;				 //所有中断使能

	myPwm();	//开发射机

//	P1M1=0x02;
//	P1M2=0x00;
	P10=1;
//	P11=1;

	PAshutdown=0;		  //开机时，将功放关闭
	Check=GetADCResult(6);//电池电量检测

	while(1)
	{
		if(Turn==0)
		{
		 	Delay(30);
			if(Turn==0)
			{
//				while(Turn==0);
				if(TurnFlag==0)		 //说明是关机状态,则开机
				{
//					SwitchControl=0;	 //开起接收机的控制端为高电平 

					PAshutdown=1;
					SC_Speech(4);
					Delay(200);
					PAshutdown=0;

					ModeControl_1=0;  //发射机模式控制端,开机时设为1.5M模式,开启发射机					
			
					if(Check>=0x35a)//设置比较电压，此处为3.3V
					{
						PAshutdown=1;
						SC_Speech(6);//电压充足提示
						Delay(200);
						PAshutdown=0;
//						poweroverflag=1;			   
					}
					else
					{
						PAshutdown=1;
						SC_Speech(7);//电压不充足提示
						Delay(200);
						PAshutdown=0;
//						poweroverflag=0;
					}

					StartAll();//开机给主机发送开机指令	
					commuFlag=1;//开启通信
					TurnFlag=1;
				}
				else
				{	 
//					SwitchControl=0;//关闭接收机控制端为低电平
					
					PAshutdown=1;
					SC_Speech(5);
					Delay(120);
					PAshutdown=0;

					Moto=1;//停止马达震动

//					Check=GetADCResult(6);//电池电量检测
					if(Check>=0x35a)//设置比较电压，此处为4V
					{
						PAshutdown=1;
						SC_Speech(6);//电压充足提示
						Delay(120);	
						PAshutdown=0;
//						poweroverflag=1;
					}
					else
					{
						PAshutdown=1;
						SC_Speech(7);//电压不充足提示
						Delay(120);
						PAshutdown=0;
//						poweroverflag=0;
					}
					commuFlag=0;//关闭通信

					StopAll();
					Delay3(150);
					StopAll();
					Delay3(150);
					StopAll();
					Delay3(150);

					TurnFlag=0;
					alarmCount2=0;//清报警计数器
					alarmFlag2=0;//清报警标志
					alarmCount3=0;//清报警计数器
					alarmFlag3=0;//清报警标志
				}
			}
		}
		if(ModeChange==0)
		{
			Delay(20);
			if(ModeChange==0)
			{
//				while(ModeChange==0);
				if(ModeFlag==0&&TurnFlag==1)//开机状态可以调模式
				{
					ModeControl_1=1;//切换发射机模式
					ModeFlag=1;
			
					PAshutdown=1;
					SC_Speech(2);
					Delay(140);
					PAshutdown=0;
				}
				else if(ModeFlag==1&&TurnFlag==1)
				{
					ModeControl_1=0; //切换发射机模式
					ModeFlag=0;
			
					PAshutdown=1;
					SC_Speech(3);
					Delay(140);
					PAshutdown=0;
				}
			}
		}

		if((alarmFlag2==1)&&(alarmCount2<1))//编码2开始相应的报警
		{
			alarmCount2++;

			PAshutdown=1;
			SC_Speech(1);
			Delay(160);
			PAshutdown=0;			

			Moto=0;	//开震动
			Delay(10);
			Moto=1;
			
			PAshutdown=1;
			SC_Speech(1);
			Delay(160);
			PAshutdown=0;

			Moto=0;//开震动
			Delay(10);
			Moto=1;
		}
		
//		if(alarmCount2>=1)  //调节语音的段数
//		{
//			alarmCount2=0;//清报警计数器
//			alarmFlag2=0;//清报警标志
//		}

		if((alarmFlag3==1)&&(alarmCount3<1))//编码3开始相应的报警
		{
			alarmCount3++;
			
			PAshutdown=1;
			SC_Speech(10);
			Delay(150);
			Moto=0;//开震动
			Delay(20);
			Moto=1;
	
			SC_Speech(10);
			Delay(150);
			Moto=0;//开震动
			Delay(20);
			Moto=1;
			PAshutdown=0;
 		}

//		if(alarmCount3>=1) //调节语音的段数	   
//		{
//			alarmCount3=0;//清报警计数器
//			alarmFlag3=0;//清报警标志
//		}

		if(Check>=0x377) //表示电池充包了
		{
			BatteryControl=1;//开漏模式，这样为高阻态	
		}
		else
		{
			BatteryControl=0;//电池在没有充满的情况下为低电平
		}

		if(TurnFlag==1)
		{
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

//		if(SwitchControlcount==18000)
//		{
//			SwitchControl=0;
//			SwitchControlcount=0;	
//		}
	}
}

void timeT1() interrupt 3 //定时器1中断接收数据
{
//	unsigned int newAddr=0;
	TH1=timer1H;//重装载
	TL1=timer1L;
   
//	if(SwitchControl==1)
//	{
//		SwitchControlcount++;
//	}
//	else
//	{
	if(P11==0)//正常情况为高电平,有低电平说明有信号
	{
		DataBetween++;
		ComFlag=0;
		if(DataBetween==150)//低电平持续的最大时间	
		{
			DataBetween=0;
		}
	}
	else//为高电平了
	{
		if(ComFlag==0)//说明有一个低电平
		{
			ComFlag=1;
//				RecData<<=1;

			if((DataBetween>60)&&(DataBetween<=100))	//低电平持续的时间小于10ms，则为0
			{
				RecData<<=1;
				RecData &= 0xfe;
				DataTime++;
				T1highcount=0;
			}
			else if((DataBetween>100))//低电平持续的时间大于4.5ms，则为1
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
//				DataTime++;
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

	if(DataTime==8)//说明一个字节的数据已经接受完全
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

	if(receiveFlag==1)	//说明接收到了数据，开始处理
	{
		receiveFlag=0;	//清接收标志
		SwitchControl=1;

//		transCode(TxRxBuf,0x1c);//将接收到得数据解码
 
		switch(myTxRxData[2])//解析指令
		{
/*
			case ComMode_1://接收到的是主机发送过来的编码1的信号，说明主机在3M内，是正常的
			{	
//					newAddr=(newAddr|myTxRxData[4])<<8;//高八位
//					newAddr=newAddr+myTxRxData[3];		   //低八位
//					if(PassWord[newAddr]==myTxRxData[5]&&PassWord[newAddr+1]==myTxRxData[6])//密码表是否滚动了一个然后对的住
//					{
//						if(newAddr>=999)
//						{
//							lastAddr=0;
				TestFlag=0;

//					alarmCount2=0;//清报警计数器
//					alarmFlag2=0;//清报警标志
//					alarmCount3=0;//清报警计数器
//					alarmFlag3=0;//清报警标志
//				SwitchControl=1;
//						}
//						else
//						{
//							 lastAddr+=1;
//							 TestFlag=0;//正常情况，清超时标志
//						}
//				dataFirst=ComMode_1;
//				if(ModeFlag==2||ModeFlag==3)
//				{
//					SC_Speech(0x01);	//恢复了正常，做相应复位动作
//				}
			}
			break;
*/				
			case ComMode_2://说明在30m内，已不正常
			{
				TestFlag=0;//清超时标志
//					alarmFlag2=1;
				alarmCount2=0;//清报警计数器
				alarmFlag2=0;//清报警标志
				alarmCount3=0;//清报警计数器
				alarmFlag3=0;//清报警标志
			}
			break;
			
			case ComMode_3:
			{
//					TestFlag=0;//清超时标志				
				alarmFlag3=1;
				alarmCount2=3;			
			}
			break;
			
			case ComMode_4://留作抬起信号使用
			{
				TestFlag=0;//清超时标志	
				alarmFlag4=1;//抬起报警
			}
			break;

			case ComMode_5://留作倒地信号使用
			{
				TestFlag=0;//清超时标志
				alarmFlag5=1;	//倒地报警
			}
			break;
		}
	}
//	}
}

void time0() interrupt 1	//作为整个系统自己的时钟
{
	TH0=timer0H;//重装载
	TL0=timer0L;

	time0Count_3++;

	if(time0Count_3>=60)//串口每1S发送一次的数据的时间标志
	{
/*
		if(onePin==0&&threeFlag==0)
		{
			onePin=1;
			twoPin=0;
			threePin=0;
			threeFlag=1;
		}
		else if(twoPin==0&&onePin==1)
		{
			onePin=0;
			twoPin=1;
			threePin=0;
		}
		else if(threePin==0&&twoPin==1)
		{
			onePin=0;
			twoPin=0;
			threePin=1;
			threeFlag=0;
		}
*/

		if(commuFlag==1)//说明开启了通信
		{
			ComMode_1_Data();//发送模式1信号
//			SendData(0xbc);	 //测试数据
			
			TestFlag++;
			if(TestFlag>=6)//说明已经出了300M了。收不到任何信号了，要做报警
			{
				TestFlag=7;
				alarmFlag2=1;
				//加入相应处理代码	
			}
 		}

		Check=GetADCResult(6);//电池电量检测
		time0Count_3=0;
	}
}

void StartAll()	//发送开始信号
{
//	P0M1&=0xfd;	 //高阻模式	，相当关发射机
//	P0M2&=0xfd;
//	ModeControl_1=0;//发射功率选择，有按键来确定				
//	myPwm();	//开发射机

/*
	SendData(0x55);
	SendData(0xf0);
	SendData(0xaa);
	SendData(0x0f);
*/
	unsigned char i,n;

	myTxRxData[0]=CmdHead;
	myTxRxData[1]=MyAddress;
	myTxRxData[2]=CmdStart;
	myTxRxData[3]=0x00;
	myTxRxData[4]=0x00;
	myTxRxData[5]=0x00;
	myTxRxData[6]=0x00;

	initsignal();

	for(i=0;i<7;i++)
	{
		for(n=0;n<8;n++)
		{
			if((myTxRxData[i]&0x80)==0x80)//为1
			{
				P10=0;
				Delay3(110);//延时4.5ms以上，由于定时器占用问题，只能用这种延时来实现
			}
			else//为0的情况
			{
				P10=0;
				Delay3(70);//延时2ms，由于定时器占用问题，只能用这种延时来实现
			}
			P10=1;//常态为高电平
			myTxRxData[i]<<=1;
			Delay3(50);//延时要大于2ms
		}
	}
//	codeData(myTxRxData,7);

//	SendNByte(TxRxBuf,28);

//	测试
//	SendNByte(myTxRxData,7);

//	PWMCON0=0x00;//关闭PWM
//	P0M1|=0x02;	 //高阻模式	，相当关发射机
//	P0M2&=0xfd;
}

void StopAll() //发送停止信号
{
//	P0M1&=0xfd;	 //高阻模式	，相当关发射机
//	P0M2&=0xfd;
//ModeControl_1=0;//发射功率选择，有按键来确定				
//	myPwm();	//开发射机
/*
	SendData(0x55);
	SendData(0xf0);
	SendData(0xaa);
	SendData(0x0f);
*/

	unsigned char i,n;

	myTxRxData[0]=CmdHead;
	myTxRxData[1]=MyAddress;
	myTxRxData[2]=CmdStop;
	myTxRxData[3]=0x00;
	myTxRxData[4]=0x00;
	myTxRxData[5]=0x00;
	myTxRxData[6]=0x00;

	initsignal();

	for(i=0;i<7;i++)
	{
		for(n=0;n<8;n++)
		{
			if((myTxRxData[i]&0x80)==0x80)//为1
			{
				P10=0;
				Delay3(110);//延时4.5ms以上，由于定时器占用问题，只能用这种延时来实现
			}
			else//为0的情况
			{
				P10=0;
				Delay3(70);//延时2ms，由于定时器占用问题，只能用这种延时来实现
			}
			P10=1;//常态为高电平
			myTxRxData[i]<<=1;
			Delay3(50);//延时要大于2ms
		}
	}
//	codeData(myTxRxData,7);
//	SendNByte(TxRxBuf,28);
//测试
//	SendNByte(myTxRxData,7);

//	PWMCON0=0x00;//关闭PWM
//	P0M1|=0x02;	 //高阻模式	，相当关发射机
//	P0M2&=0xfd;
}

void ComMode_1_Data()//发送边码1
{
//	P0M1&=0xfd;	 //高阻模式	，相当关发射机
//	P0M2&=0xfd;
	
					
//	myPwm();	//开发射机
//	ModeTurn=0;
/*
	SendData(0x55);
	SendData(0xf0);
	SendData(0xaa);
	SendData(0x0f);
*/
	unsigned char i,n;

	ModeControl_1=0;//30M发射功率

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
			if((myTxRxData[i]&0x80)==0x80)//为1
			{
				P10=0;
				Delay3(110);//延时4.5ms以上，由于定时器占用问题，只能用这种延时来实现
			}
			else//为0的情况
			{
				P10=0;
				Delay3(70);//延时2ms，由于定时器占用问题，只能用这种延时来实现
			}
			P10=1;//常态为高电平
			myTxRxData[i]<<=1;
			Delay3(50);//延时要大于2ms
		}
	}
//	codeData(myTxRxData,7);
//	SendNByte(TxRxBuf,28);
//	测试
//	SendNByte(myTxRxData,7);
//	PWMCON0=0x00;//关闭PWM
//	P0M1|=0x02;	 //高阻模式	，相当关发射机
//	P0M2&=0xfd;
}


/*
void codeData(unsigned char *doData,unsigned char len)		//编码 ,电平1变为0011，电平0变为1100
{
	unsigned char n,j,i=0;
	for(n=0;n<len;n++)
	{	
		
		for(j=0;j<8;j++)	
		{
			if(j==0||j==1)
			{	
				 TxRxBuf[i]<<=4;
				if((*doData&0x80)==0x80)
				{
					TxRxBuf[i]|=0x03;	
				}
				else
				{
					TxRxBuf[i]|=0x0c;
				}
				*doData<<=1;
			}
			else if(j==2||j==3)
			{
				TxRxBuf[i+1]<<=4;
				if((*doData&0x80)==0x80)
				{
					TxRxBuf[i+1]|=0x03;	
				}
				else
				{
					TxRxBuf[i+1]|=0x0c;
				}
				*doData<<=1;
			}
			else if(j==4||j==5)
			{
				TxRxBuf[i+2]<<=4;
				if((*doData&0x80)==0x80)
				{
					TxRxBuf[i+2]|=0x03;		
				}
				else
				{
					TxRxBuf[i+2]|=0x0c;
				}
				*doData<<=1;
			}
			else if(j==6||j==7)
			{
				TxRxBuf[i+3]<<=4;
				if((*doData&0x80)==0x80)
				{
					TxRxBuf[i+3]|=0x03;						
				}
				else
				{
					TxRxBuf[i+3]|=0x0c;				
				}
				*doData<<=1;
			}

		}
		i+=4;
		doData++;
	}
}
*/

/*
void transCode(unsigned char *doData,unsigned char len)//解码，将接收到得数据还原
{
	unsigned char i,j;
	for(i=0;i<len;i++)
	{
		for(j=0;j<2;j++)
		{
			myTxRxData[i/4]<<=1;

			if((*doData&0x30)==0x30)//说明为1
			{
				myTxRxData[i/4]|=0x01;
			}
			else if((*doData&0xc0)==0xc0)  //说明为0
			{
				myTxRxData[i/4]&=0xfe;	
			}

			*doData<<=4;
		}
		doData++;
	}
}
*/