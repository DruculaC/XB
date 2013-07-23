#include"N79E81x.h"
#include<intrins.h>
#include"AD.h"
#include"T0.h"
#include"voice.h"
#include"pwm.h"
#include"T1.h"

#define CmdStart 0x00 //��������
#define CmdStop 0x01  //�ػ�����

#define ComMode_1 0xc1 //ͨ��ģʽ1 
#define ComMode_2 0xc2 //ͨ��ģʽ2
#define ComMode_3 0xc3 //ͨ��ģʽ3
#define ComMode_4 0xc4 //̧��ָ��
#define ComMode_5 0xc5//����ָ��

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
sbit ReceWave=P0^7;			//���ᴫ��������ˣ�1��ʾ���񶯣�0��ʾ����
sbit SensorControl=P2^5;	//���ᴫ����ʹ�ܶˣ�1�򿪴�������0�رմ�����
sbit tran_en=P2^7;		 	//���߷����ʹ�ܿ��ƣ�1Ϊ�򿪷������0Ϊ�رշ����
sbit MagentControl_1=P2^2;	//����������˿ڣ����Ƶ������ת���߷�ת
sbit MagentControl_2=P2^3;
sbit VoiceControl=P2^4;		//ʰ����ʹ�ܶˣ�1��ʰ������0�ر�ʰ����
sbit PAshutdown=P1^4;		//���ſ��ؿ��ƣ�1Ϊ�򿪹��ţ�0Ϊ�رչ���

unsigned char count=0;//���ݽ��ղ��ֵļ�����

unsigned int time0Count_1=0;//��Ϊ���ᴫ������������֮���ʱ������ʱ
unsigned int time0Count_2=0;//��Ϊ���ᴫ�����ļ�ʱ
unsigned int time0Count_3=0;//��ʱ��T0�ļ���
unsigned int time0Count_4=0;//��Ϊ̧�������ʱ������ʱ
unsigned int time0Count_5=0;//��Ϊ���������ʱ������ʱ

bit SensorFlag=0; 				//���ᴫ�����ĵ͵�ƽ��־λ
unsigned char SensorCount=0; 	//��Ϊ���ᴫ��������ļ���
unsigned char TestFlag=0;		//ÿ3s������1�����ͨ�ųɹ���������㡣�����������n�ζ�û�й��㣬��˵�����ڳ���
unsigned char ModeFlag=1;		//ģʽѡ��λ��1����ģʽ1,2����ģʽ2,3��Ϊģʽ3

bit alarmFlag=0;				//���������Ŀ�����־
bit alarmFlag2=0;				//����������־2
unsigned char alarmCount=0;		//���������Ĵ���

bit downUpFlag=0;  				//���غ�̧�����־
bit downFlag=0;					//���صı�־
bit upFlag=0;					//̧��ı�־
bit downFlagSend=0;				//���ط��͵ı�־
bit upFlagSend=0;				//̧���͵ı�־

//��Ϊ���պͷ��͵Ļ�����
unsigned char TxRxBuf[28]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

//һ��ͷ�ֽڣ�һ����ַ�ֽڣ�һ�������ֽڣ����������ַ�ֽڣ���������
unsigned char myTxRxData[7]={0x00,0x00,0x00,0x00,0x00,0x00,0x00};//��������ͨ�����ݵĻ�����

unsigned int Check2=0;		//�������
unsigned char Check1=0;		//��ΪAD���ֵ��ʰ������⣬ƽʱ����Ϊ1V���ڣ����ź�ʱΪ1.5V

bit receiveFlag=0;			//���յ����ݱ�־
bit commuFlag=0;			//����ͨ�ű�־

unsigned char DataBetween=0;//��Ϊ�������ݵ��м����
unsigned char RecData=0;	//���յ�������
unsigned char DataTime=0;	//��Ϊ���յ����ݵ���λ��������
bit ComFlag=1;				//�������ص�һ����־
unsigned char T1highcount=0;	   	//��ʱ��T1��û���źŵ�����ʱ�򣬶Ըߵ�ƽ������һ������ĳ��ֵ����Datatime��0

unsigned char magnetflag=0;		   	//��������ת�����
unsigned char powerflag=1;		   	//�����ѹ������־λ
unsigned char commode2_flag=0; 		//���ͱ���2�ı�־λ

//��������
void ComMode_1_Data(void);			//����ģʽ1����
void ComMode_2_Data(void);			//����ģʽ2����
void ComMode_22_Data(void);			//����ģʽ2����
void ComMode_3_Data(void);			//����ģʽ3����
void ComMode_4_Data(void);			//����̧�����
void ComMode_5_Data(void);			//���͵��ر���

void Delay3(unsigned int t)			//t=1ʱ���ӳ�100us����
{
	unsigned int i,j;
	for(i=0;i<t;i++)		
	for(j=0;j<19;j++);
}
void Delay33(unsigned int t)
{
	unsigned int i,j;
	for(i=0;i<t;i++)		
	for(j=0;j<26;j++);
}

void initsignal()					//init signal�����ͱ����ź�ǰ����ʼ�źţ����ڽ����ջ����Զ������
{
	unsigned char k,k1;
	unsigned char mystartbuffer=0xb4;
	for(k1=0;k1<1;k1++)
	{
		for(k=0;k<8;k++)
		{
			if((mystartbuffer&0x80)==0x80)//Ϊ1
			{
				P10=0;
				Delay3(300);		//��ʱ4.5ms���ϣ����ڶ�ʱ��ռ�����⣬ֻ����������ʱ��ʵ��
			}
			else					//Ϊ0�����
			{
				P10=0;
				Delay3(300);		//��ʱ2ms�����ڶ�ʱ��ռ�����⣬ֻ����������ʱ��ʵ��
			}
			P10=1;					//��̬Ϊ�ߵ�ƽ
			mystartbuffer<<=1;
			Delay3(100);			//��ʱҪ����2ms
		}
		mystartbuffer=0xaa;
		Delay3(50);
	}
	P10=1;
}

void initsignal2()
{
	unsigned char k,k1;
	unsigned char mystartbuffer=0xb4;
	for(k1=0;k1<8;k1++)
	{
		for(k=0;k<8;k++)
		{
			if((mystartbuffer&0x80)==0x80)		//Ϊ1
			{
				P10=0;
				Delay3(205);					//��ʱ4.5ms���ϣ����ڶ�ʱ��ռ�����⣬ֻ����������ʱ��ʵ��
			}
			else								//Ϊ0�����
			{
				P10=0;
				Delay3(205);					//��ʱ2ms�����ڶ�ʱ��ռ�����⣬ֻ����������ʱ��ʵ��
			}
			P10=1;//��̬Ϊ�ߵ�ƽ
			mystartbuffer<<=1;
			Delay3(70);							//��ʱҪ����2ms
		}
		mystartbuffer=0xaa;
		Delay3(35);
	}
	P10=1;
}
void main()
{
	noVoice();
	InitT0();
	InitT1();
	ET0=1;
	ET1=1;
	PT1=1;
	EA=1;
	P10=1;

	myPwm();				//�������
	VoiceControl=0;   		//�ϵ��ʱ��ʰ������
	PAshutdown=0;			//�����Źر�
	tran_en=0;				//������ر�
	ModeControl_1=1;    	//�����ģʽΪ�͹���
	MagentControl_1=0;		//�رմ���
	MagentControl_2=1;
	Delay(27);
	MagentControl_1=0;		//������̬Ϊ����ģʽ
	MagentControl_2=0;
	magnetflag=0;
	
	Check2=0x3ff;
	commode2_flag=0;
	Check1=GetADCResult(5);		//ʰ�����ļ��
	Check2=GetADCResult(6);		//�������
	powerflag=0;
	SensorControl=0;			//�رմ�����

	while(1)
	{
		if(Check1>=0x100)	//���ñȽϵ�ѹ���˴�Ϊ3V����
		{
			PAshutdown=0;	//ʰ��������ĳ����ѹʱ���򿪹���
		}
		else
		{
			PAshutdown=0;	//ʰ����һ������ĳ����ѹ����رչ���
		}

		if((powerflag==1)&&(Check2<=0x35a))
		{
			powerflag=0;
			PAshutdown=1;
			SC_Speech(7);	//��ѹ��������ʾ
			Delay(120);
			PAshutdown=0;
		}
		else if((powerflag==0)&&(Check2>=0x377))
		{
			powerflag=1;
			PAshutdown=1;
			SC_Speech(6);	//��ѹ������ʾ
			Delay(120);
			PAshutdown=0;
		}

	}
}

void timeT1() interrupt 3 	//��ʱ��1�жϽ�������
{
	TH1=timer1H;			//��װ��
	TL1=timer1L;

	if(P11==0)				//�������Ϊ�ߵ�ƽ,�е͵�ƽ˵�����ź�
	{
		DataBetween++;
		ComFlag=0;
		if(DataBetween==150)//�͵�ƽ���������ʱ��	
		{
			DataBetween=0;
		}
	}
	else					//Ϊ�ߵ�ƽ��
	{
		if(ComFlag==0)		//˵����һ���͵�ƽ
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
			if(T1highcount>=150)
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

	if(receiveFlag==1)
	{
		receiveFlag=0;
		switch(myTxRxData[2]) 				//������֡���������д���
		{
			case ComMode_1:  				//�������͹�����ֻ��ģʽ1��˵�������������ģ����ݲ���Ϊ����ĵ�һ�͵ڶ����ֽڣ�Ϊ������ڵ��������Ŀ�ʼ�ֽڵ��Ǹ���ַ��Ȼ���������֡�������������ݷ��ͳ�ȥ
			{
				ComMode_22_Data();			//�ظ�ȷ���ź�
				alarmFlag=0;				//�ر�����־λ
				alarmCount=0;				//����������������
				commode2_flag=0;  			//�رձ���2�źŵķ���
				SensorControl=0;			//�ر����ᴫ����
				downUpFlag=0; 				//�ص��ء�̧����
							
				if(magnetflag==0)
				{
					MagentControl_1=1;		//��������
					MagentControl_2=0;
					Delay(27);
					MagentControl_1=0;		//������̬Ϊ����ģʽ
					MagentControl_2=0;
					magnetflag=1;
				}
				TestFlag=0;	
				
				if(ModeFlag==3||ModeFlag==2)
				{
					ModeFlag=1;
				}
			}
			break;
		}
	}
}

void time0() interrupt 1			//��Ϊ����ϵͳ�Լ���ʱ��
{
	TH0=timer0H;					//��װ��
	TL0=timer0L;
	time0Count_3++;

	if(time0Count_3>=60)			//����ÿ3S����һ�ε����ݵ�ʱ���־
	{
		if(commuFlag==1)			//˵��������ͨ��
		{
			TestFlag++;
			if(TestFlag==3&&ModeFlag==1)		//˵��û�н��յ������Ѿ���3���ˣ������Ѿ�����3M�����ھ�Ҫ�Ӵ��ʣ��л���ģʽ2,30M�ٿ��ܲ��ܽ��յ�����
			{
				TestFlag=5;
				if(ModeFlag==1)
				{
					if(magnetflag==1)
					{
						MagentControl_1=0;		//��������
						MagentControl_2=1;
						Delay(27);
						MagentControl_1=0;		//������̬Ϊ����ģʽ
						MagentControl_2=0;
						magnetflag=0;
					}
					SensorControl=1;			//�������ᴫ����
					downUpFlag=1;				//�������ء�̧���־
					ModeFlag=2;
					commode2_flag=1;
				}	
			}

			if(commode2_flag==1)
			{
				ComMode_2_Data();				//�򸽻����ͱ���2	
			}
		}
		time0Count_3=0;
		Check1=GetADCResult(5);					//ʰ�����ļ��
		Check2=GetADCResult(6);					//�������
	}

	if(SensorControl==1)						//������ᴫ�����Ƿ��
	{
		if(ReceWave==0)							//˵���д����������ʼ��ʱ
		{
			time0Count_2++;
			if(time0Count_2>=10)				//˵���Ѿ�����0.5S
			{
				time0Count_2 =0;				//��ʱ������
				SensorCount++;					//���ᴫ�������������1
				alarmFlag2=1;
			}		
		}
		else if(ReceWave==1&&SensorCount!=0)	//˵���Ѿ���һ�����õ�����
		{
			time0Count_1++;
			if(time0Count_1>=200)				//����10S
			{
				SensorCount=0;
			}
		}
	}
 
	if(ModeFlag==2&&SensorCount>=1)				//���ᴫ�����������Ӧ����
	{
		if(SensorCount==1&&alarmFlag2==1)		//���ᴫ����һ�δ���,alarmFlag2���Ʒ���1��
		{					
			VoiceControl=0;						//ʹ������ʱҪ�ر�ʰ����
			PAshutdown=1;  						//�ȹر�ʰ������Ȼ��򿪹���
			SC_Speech(1);  						//������ʾ
			Delay(140);
			PAshutdown=0;  						//������ɺ󣬹رչ��ţ�Ȼ���ʰ����
			VoiceControl=1;						//����ʰ����
			alarmFlag2=0;
		}
		if(SensorCount>=2)						//���ᴫ����һ�δ���
		{
			ModeFlag=3;							//���ᴫ�����Ѿ���3�δ����ˣ�Ҫ�ı䷢��ģʽ��
			alarmFlag=1;						//����������λ
			alarmFlag2=0;
			SensorCount=0; 						//�����������
			Delay(1);
			commode2_flag=0;					//�����ͱ���2��
			ComMode_3_Data(); 					//�򸽻����ͱ���3
		}
	}

	if(ModeFlag==3)
	{
		if(alarmFlag==1)
		{
			VoiceControl=0;			//ʹ������ʱҪ�ر�ʰ����
			PAshutdown=1;
			SC_Speech(3);  			//�ػ���������
			Delay(120);
			PAshutdown=0;
			VoiceControl=1;			//����ʰ����
		}
		if(alarmFlag==1)
		{
			VoiceControl=0;			//ʹ������ʱҪ�ر�ʰ����
			PAshutdown=1;
			SC_Speech(4);  			//�ػ���������
			Delay(190);
			PAshutdown=0;
			VoiceControl=1;			//����ʰ����
		}
		if(ModeFlag==3)
		{
			ComMode_3_Data(); 		//�򸽻����ͱ���3
		}
		if(alarmFlag==1)
		{
			VoiceControl=0;			//ʹ������ʱҪ�ر�ʰ����
			PAshutdown=1;
			SC_Speech(3);  			//�ػ���������
			Delay(120);
			PAshutdown=0;
			VoiceControl=1;			//����ʰ����
		}
		if(alarmFlag==1)
		{
			VoiceControl=0;			//ʹ������ʱҪ�ر�ʰ����
			PAshutdown=1;
			SC_Speech(11);  		//�ػ���������
			Delay(190);
			PAshutdown=0;
			VoiceControl=1;			//����ʰ����
		}
		if(ModeFlag==3)
		{
			ComMode_3_Data(); 		//�򸽻����ͱ���3
		}
		if(alarmCount>=20) 			//���������Ķ���
		{
			alarmCount=0;			//�屨��������
			alarmFlag=0;			//�屨����־
		}
		alarmCount++;
	}
}

void ComMode_1_Data()				//���ͱ���1
{
	unsigned char i,n;
	ModeControl_1=1;				//С���ʷ���ģʽ				
	tran_en=1;
	myTxRxData[0]=CmdHead;
	myTxRxData[1]=MyAddress;
	myTxRxData[2]=ComMode_1;
	myTxRxData[3]=0x00;
	myTxRxData[4]=0x00;
	myTxRxData[5]=0x00;
	myTxRxData[6]=0x00;

	initsignal2();													   

	for(i=0;i<7;i++)
	{
		for(n=0;n<8;n++)
		{
			if((myTxRxData[i]&0x80)==0x80)			//Ϊ1
			{
				P10=0;
				Delay3(120);						//��ʱ4.5ms���ϣ����ڶ�ʱ��ռ�����⣬ֻ����������ʱ��ʵ��
			}
			else									//Ϊ0�����
			{
				P10=0;
				Delay3(80);							//��ʱ2ms�����ڶ�ʱ��ռ�����⣬ֻ����������ʱ��ʵ��
			}
			P10=1;									//��̬Ϊ�ߵ�ƽ
			myTxRxData[i]<<=1;
			Delay3(50);								//��ʱҪ����2ms
		}
	}
	tran_en=0;
}

void ComMode_2_Data()								//���ͱ���2
{
	unsigned char i,n;
	ModeControl_1=0;								//С����
	tran_en=1;
	myTxRxData[0]=CmdHead;
	myTxRxData[1]=MyAddress;
	myTxRxData[2]=0xc2;
	myTxRxData[3]=0x00;
	myTxRxData[4]=0x00;
	myTxRxData[5]=0x00;
	myTxRxData[6]=0x00;

	initsignal2();
	for(i=0;i<7;i++)
	{
		for(n=0;n<8;n++)
		{
			if((myTxRxData[i]&0x80)==0x80)
			{
				P10=0;
				Delay3(120);
			}
			else
			{
				P10=0;
				Delay3(80);
			}
			P10=1;
			myTxRxData[i]<<=1;
			Delay3(50);
		}
	}
	tran_en=0;
}

void ComMode_22_Data()
{
	unsigned char i,n;
	ModeControl_1=0;
	tran_en=1;
	myTxRxData[0]=CmdHead;
	myTxRxData[1]=MyAddress;
	myTxRxData[2]=0xc2;
	myTxRxData[3]=0x00;
	myTxRxData[4]=0x00;
	myTxRxData[5]=0x00;
	myTxRxData[6]=0x00;

	initsignal();
	for(i=0;i<7;i++)
	{
		for(n=0;n<8;n++)
		{
			if((myTxRxData[i]&0x80)==0x80)
			{
				P10=0;
				Delay33(120);
			}
			else
			{
				P10=0;
				Delay33(80);
			}
			P10=1;
			myTxRxData[i]<<=1;
			Delay33(50);
		}
	}
	tran_en=0;
}

void ComMode_3_Data()
{
	unsigned char i,n;
	ModeControl_1=0;							//����
	tran_en=1;
	myTxRxData[0]=CmdHead;
	myTxRxData[1]=MyAddress;
	myTxRxData[2]=ComMode_3;
	myTxRxData[3]=0x00;
	myTxRxData[4]=0x00;
	myTxRxData[5]=0x00;
	myTxRxData[6]=0x00;

	initsignal2();

	for(i=0;i<7;i++)
	{
		for(n=0;n<8;n++)
		{
			if((myTxRxData[i]&0x80)==0x80)
			{
				P10=0;
				Delay3(120);
			}
			else
			{
				P10=0;
				Delay3(80);
			}
			P10=1;
			myTxRxData[i]<<=1;
			Delay3(50);
		}
	}
	tran_en=0;
	ModeControl_1=1;						//���ͱ���3��ɺ󣬻���С����
}