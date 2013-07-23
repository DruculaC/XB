#include"N79E81x.h"
#include<intrins.h>
#include"AD.h"
#include"T0.h"
#include"voice.h"
#include"pwm.h"
#include"T1.h"

//����ͨ������

#define CmdStart 0x00 //��������
#define CmdStop 0x01  //�ػ�����

#define ComMode_1 0xc1 //ͨ��ģʽ1 
#define ComMode_2 0xc2 //ͨ��ģʽ2
#define ComMode_3 0xc3 //ͨ��ģʽ3
#define ComMode_4 0xc4 //̧��ָ��
#define ComMode_5 0xc5 //����ָ��

#define Succeed 0xce  //ͨ�ųɹ�
#define Wrong 0xff    //ͨ��ʧ��

#define CmdHead 0xc8
#define CmdHead1 0x33 //����֡���ײ�1, 00110011,11
#define CmdHead2 0xcc //����֡���ײ�2,11001100,00
#define CmdHead3 0x3c //����֡���ײ�3,11000011,01
#define CmdHead4 0xcc //����֡���ײ�4,11001100,00

#define MyAddress 0xe0
#define MyAddress1 0x33 //������ַ1, 00110011,11
#define MyAddress2 0x3c //������ַ2, 11000011,01
#define MyAddress3 0xcc //������ַ3,11001100,00
#define MyAddress4 0xcc //������ַ4,11001100,00

sbit ModeControl_1=P2^6;	//���߷����ģʽ���ƣ�1ΪС���ʣ�0Ϊ����
sbit tran_en=P2^7;		 	//���߷����ʹ�ܿ��ƣ�1Ϊ�򿪷������0Ϊ�رշ����
sbit Moto=P2^4;		   		//�����ƶˣ�1��ﲻ�񶯣�0�����
sbit PAshutdown=P1^4;		//���ſ��ؿ��ƣ�1Ϊ�򿪹��ţ�0Ϊ�رչ���
sbit receive_en=P1^3;		//���ջ�ʹ�ܣ�Ҫ����������

bit receiveFlag=0;	//���յ����ݱ�־
bit commuFlag=0;	//����ͨ�ű�־��1��ʾ��ʼͨ�ţ�0��ʾû��ͨ��
bit alarmFlag2=0;	//����2������־
bit alarmFlag3=0;	//����3������־
bit alarmFlag4=0;	//̧�𱨾���־
bit alarmFlag5=0;	//���ر�����־
unsigned char alarmCount2=0;	//����2ѭ������
unsigned char alarmCount3=0;	//����3ѭ������
unsigned char alarmCount4=0;	//̧�𱨾�ѭ������
unsigned char alarmCount5=0;	//���ر���ѭ������

unsigned char count=0;			//���ڽ��ղ��ֵļ�����
unsigned int time0Count_3=0;	//��ʱ��T0�ļ���

unsigned char TestFlag=0;		//ÿ3s������1�����ͨ�ųɹ���������㡣�����������n�ζ�û�й��㣬��˵�����ڳ���

//��Ϊ���պͷ��͵Ļ�����
unsigned char TxRxBuf[28]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
//һ��ͷ�ֽڣ�һ����ַ�ֽڣ�һ�������ֽڣ����������ַ�ֽڣ���������
unsigned char myTxRxData[7]={0x00,0x00,0x00,0x00,0x00,0x00,0x00};//��������ͨ�����ݵĻ�����
 
//ͨ�����ݶ���
unsigned char DataBetween=0;	//��Ϊ�������ݵ��м����
unsigned char RecData=0;		//���յ�������
unsigned char DataTime=0;		//��Ϊ���յ����ݵ���λ��������
bit ComFlag=1;					//��������غ������صı�ʶ
unsigned char T1highcount=0;	//��ʱ��T1��û���źŵ�����ʱ�򣬶Ըߵ�ƽ������һ������ĳ��ֵ����Datatime��0

unsigned int Check=0;			//��ΪAD���ֵ������ص���
 
unsigned char powerflag=1;		//�����ѹ������־λ


void ComMode_1_Data(void);		//����1����

void Delay3(unsigned int t)		//t=1ʱ���ӳ�100us����
{
	unsigned int i,j;
	for(i=0;i<t;i++)		
	for(j=0;j<23;j++);
}

void initsignal()				//init signal�����ͱ����ź�ǰ����ʼ�źţ����ڽ����ջ����Զ������
{
	unsigned char k,k1;
	unsigned char mystartbuffer=0xaa;
	for(k1=0;k1<3;k1++)
	{
		for(k=0;k<1;k++)
		{
			if((mystartbuffer&0x80)==0x80)//Ϊ1
			{
				P10=0;
				Delay3(80);		//��ʱ4.5ms���ϣ����ڶ�ʱ��ռ�����⣬ֻ����������ʱ��ʵ��
			}
			else//Ϊ0�����
			{
				P10=0;
				Delay3(80);		//��ʱ2ms�����ڶ�ʱ��ռ�����⣬ֻ����������ʱ��ʵ��
			}
			P10=1;//��̬Ϊ�ߵ�ƽ
			mystartbuffer<<=1;
			Delay3(150);		//��ʱҪ����2ms
		}
		mystartbuffer=0xaa;
		Delay3(80);
	}
	P10=1;
}

void main(void)
{
	noVoice();			 	//������������

	InitT0();			 	//��ʼ����ʱ��0�Ͷ�ʱ��1
	InitT1();

	Moto=1;					//�ر����
	tran_en=0;	   			//�رշ����
	receive_en=0;			//�رս��ջ�
	ET0=1;				 	//ʹ�ܶ�ʱ��0��1���ж�
	ET1=1;
	PT1=1;				 	//���ö�ʱ��1Ϊ�ϸ����ȼ�
	EA=1;				 	//�����ж�ʹ��

	myPwm();				//���������

	P10=1;					//����˿ڣ�����Ϊ��

	PAshutdown=0;		  	//����ʱ�������Źر�
	Check=GetADCResult(6);	//ִ��һ�ε�ص������

	PAshutdown=1;
	SC_Speech(4);
	Delay(200);
	PAshutdown=0;
	commuFlag=1;//����ͨ��

	while(1)
	{
 		if((alarmFlag2==1)&&(alarmCount2<2))	//����2��ʼ��Ӧ�ı���
		{
			alarmCount2++;

			PAshutdown=1;
			SC_Speech(1);
			Delay(160);
			PAshutdown=0;			

			Moto=0;		//����
			Delay(10);
			Moto=1;
		}
		if((alarmFlag3==1)&&(alarmCount3<2))	//����3��ʼ��Ӧ�ı���
		{
			alarmCount3++;
			
			PAshutdown=1;
			SC_Speech(11);
			Delay(150);
			Moto=0;		//����
			Delay(20);
			Moto=1;
			PAshutdown=0;	
 		}

		if((powerflag==1)&&(Check<=0x35a))
		{
			powerflag=0;
			PAshutdown=1;
			SC_Speech(7);	//��ѹ��������ʾ
			Delay(120);
			PAshutdown=0;
		}
		else if((powerflag==0)&&(Check>=0x377))
		{
			powerflag=1;
			PAshutdown=1;
			SC_Speech(6);	//��ѹ������ʾ
			Delay(120);
			PAshutdown=0;
		}
	}
}

void timeT1() interrupt 3 		//��ʱ��1�жϽ�������
{
//	unsigned int newAddr=0;
	TH1=timer1H;				//��װ��
	TL1=timer1L;
   
	if(P11==0)					//�������Ϊ�ߵ�ƽ,�е͵�ƽ˵�����ź�
	{
		DataBetween++;
		ComFlag=0;
		if(DataBetween==150)	//�͵�ƽ���������ʱ��	
		{
			DataBetween=0;
		}
	}
	else						//Ϊ�ߵ�ƽ��
	{
		if(ComFlag==0)			//˵����һ���͵�ƽ
		{
			ComFlag=1;
			if((DataBetween>60)&&(DataBetween<=100))	//�͵�ƽ������ʱ��С��10ms����Ϊ0
			{
				RecData<<=1;
				RecData &= 0xfe;
				DataTime++;
				T1highcount=0;
			}
			else if((DataBetween>100))					//�͵�ƽ������ʱ�����10ms����Ϊ1
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

	if(DataTime==8)					//˵��һ���ֽڵ������Ѿ�������ȫ
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

	if(receiveFlag==1)			//˵�����յ������ݣ���ʼ����
	{
		receiveFlag=0;			//����ձ�־
		receive_en=0;			//�رս��ջ�
		switch(myTxRxData[2])	//����ָ��
		{
			case ComMode_2:		//˵����30m�ڣ����������ñ���
			{
				TestFlag=0;		//�峬ʱ��־
				alarmCount2=0;	//�屨��������
				alarmFlag2=0;	//�屨����־
				alarmCount3=0;	//�屨��������
				alarmFlag3=0;	//�屨����־
				Moto=0;//����
				Delay(10);
				Moto=1;
			}
			break;
			
			case ComMode_3:		//�ӵ�����3�źţ���ʼ����
			{
				TestFlag=0;		//�峬ʱ��־				
				alarmFlag3=1;
				alarmCount2=0;	//�屨��������
				alarmFlag2=0;	//�屨����־			
			}
			break;
		}
	}
//	}
}

void time0() interrupt 1		//��Ϊ����ϵͳ�Լ���ʱ��
{
	TH0=timer0H;				//��װ��
	TL0=timer0L;

	time0Count_3++;

	if(time0Count_3>=60)		//����ÿ3S����һ�ε����ݵ�ʱ���־
	{
		if(commuFlag==1)		//˵��������ͨ��
		{
			receive_en=0;
			ComMode_1_Data();	//����ģʽ1�ź�
			receive_en=1;		//�򿪽��ջ�
			TestFlag++;
			if(TestFlag==6)		//��������û���յ�����2�����ʾ������ʧ
			{
				alarmFlag2=1;
				//������Ӧ�������	
			}
 		}

		Check=GetADCResult(6);	//ÿ��3s��һ�ε������
		time0Count_3=0;			//3s�����¼���
	}
}

void ComMode_1_Data()			//���ͱ���1
{
	unsigned char i,n;

	ModeControl_1=0;			//���ʷ���ģʽ
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
			if((myTxRxData[i]&0x80)==0x80)		//Ϊ1
			{
				P10=0;
				Delay3(120);		//��ʱ4.5ms���ϣ����ڶ�ʱ��ռ�����⣬ֻ����������ʱ��ʵ��
			}
			else//Ϊ0�����
			{
				P10=0;
				Delay3(80);			//��ʱ2ms�����ڶ�ʱ��ռ�����⣬ֻ����������ʱ��ʵ��
			}
			P10=1;//��̬Ϊ�ߵ�ƽ
			myTxRxData[i]<<=1;
			Delay3(50);//��ʱҪ����2ms
		}
	}
	tran_en=0;
}