

#include"N79E81x.h"
#include<intrins.h>
#include"AD.h"
#include"UART.h"
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

//��·ѭ��
sbit onePin=P1^6;
sbit twoPin=P1^7;
sbit threePin=P0^0;

//�����ķ��䲿�ֵĿ��ƶ˿�
//sbit PWMout=P3^5;//������ķ�������ڣ�����ʹ��PWM������
sbit ModeControl_1=P2^6;	//�����ģʽ����,0Ϊ���ʣ�1ΪС����

//���ջ�����
sbit SwitchControl=P1^3;	//1�رս��ջ���0�������ջ�

//���ذ���
sbit Turn=P0^3;	

//ģʽѡ�񰴼�
sbit ModeChange=P0^4;

//�����ƶ�
sbit Moto=P2^4;

//��ؿ��� 	AD��1��ͨ��Ϊ��صĵ�������
sbit BatteryControl=P1^2;	//���Ƶ���Ƿ���Ŀ��أ�0Ϊ��磬1Ϊ�����

//����״̬���λ
bit TurnFlag=0;	//0Ϊ�ػ�״̬��1Ϊ����״̬

//ģʽѡ��λ��0����ģʽ1,1����ģʽ2
bit ModeFlag=0;

bit receiveFlag=0;	//���յ����ݱ�־
bit commuFlag=0;	//����ͨ�ű�־��1��ʾ��ʼͨ�ţ�0��ʾû��ͨ��

bit alarmFlag2=0;	//����2������־
bit alarmFlag3=0;	//����3������־
bit alarmFlag4=0;	//̧�𱨾���־
bit alarmFlag5=0;	//���ر�����־
unsigned char alarmCount2=0;//����2ѭ������
unsigned char alarmCount3=0;//����3ѭ������
unsigned char alarmCount4=0;//̧�𱨾�ѭ������
unsigned char alarmCount5=0;//���ر���ѭ������

bit threeFlag=0;	//��·ѭ�����ر�־

unsigned char voiceFlag=0;	//����ѭ������ 

unsigned char dataFirst=0;	//���ڴ洢�ϴα�������

unsigned char count=0;	//���ڽ��ղ��ֵļ�����

unsigned int time0Count_3=0;	//��ʱ��T0�ļ���

unsigned int lastAddr=0;	//��һ�ν��յ��ı���ĵ�ַ
unsigned char TestFlag=0;	//1��2��3�ֱ�Ϊÿ1S��ļ������ڴ��ڵĳɹ�ָ�����ִ�н�ȥ����Ĳ���
                			//�������3�ζ�û�й��㣬��˵�����ڳ���

//��Ϊ���պͷ��͵Ļ�����
unsigned char TxRxBuf[28]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
//һ��ͷ�ֽڣ�һ����ַ�ֽڣ�һ�������ֽڣ����������ַ�ֽڣ���������
unsigned char myTxRxData[7]={0x00,0x00,0x00,0x00,0x00,0x00,0x00};//��������ͨ�����ݵĻ�����
 
//�������ݶ���
unsigned char DataBetween=0;//��Ϊ�������ݵ��м����
unsigned char RecData=0;//���յ�������
unsigned char DataTime=0;//��Ϊ���յ����ݵ���λ��������
bit ComFlag=1;	//��������غ������صı�ʶ
unsigned char T1highcount=0;	   //��ʱ��T1��û���źŵ�����ʱ�򣬶Ըߵ�ƽ������һ������ĳ��ֵ����Datatime��0

unsigned int Check=0;	//��ΪAD���ֵ������ص���

//���ſ��ؿ��ƣ�1Ϊ�򿪹��ţ�0Ϊ�رչ���
sbit PAshutdown=P1^4;

//����һ������������ʾ�źŽ��պ󣬶೤ʱ��ʹ���ջ��򿪣�������SwitchControl�ĸߵ�ƽʱ�䡣
unsigned int SwitchControlcount=0;

//�����ѹ������־λ
unsigned char powerflag=1;

//�����������б������ܿ���
//unsigned char alarmflagall=0;	

//��������
//void codeData(unsigned char *doData,unsigned char len);		//���� ,��ƽ1��Ϊ0011����ƽ0��Ϊ1100
//void transCode(unsigned char *doData,unsigned char len);//���룬�����յ������ݻ�ԭ
void ComMode_1_Data(void);//���ͱ���1

//��������
void StartAll(void);
//�ػ�����
void StopAll(void);

//t=1ʱ���ӳ�100us����
void Delay3(unsigned int t)
{
	unsigned int i,j;
	for(i=0;i<t;i++)		
	for(j=0;j<23;j++);
}

//init signal�����ͱ����ź�ǰ����ʼ�źţ����ڽ����ջ����Զ������
void initsignal()
{
	unsigned char k,k1;
	unsigned char mystartbuffer=0xaa;
	for(k1=0;k1<3;k1++)
	{
		for(k=0;k<8;k++)
		{
			if((mystartbuffer&0x80)==0x80)//Ϊ1
			{
				P10=0;
				Delay3(80);//��ʱ4.5ms���ϣ����ڶ�ʱ��ռ�����⣬ֻ����������ʱ��ʵ��
			}
			else//Ϊ0�����
			{
				P10=0;
				Delay3(80);//��ʱ2ms�����ڶ�ʱ��ռ�����⣬ֻ����������ʱ��ʵ��
			}
			P10=1;//��̬Ϊ�ߵ�ƽ
			mystartbuffer<<=1;
			Delay3(150);//��ʱҪ����2ms
		}
		mystartbuffer=0xaa;
		Delay3(150);
	}
	P10=1;
	Delay3(80);
}

void main(void)
{
	noVoice();			 //������������

//	InitUART();
	InitT0();			 //��ʼ����ʱ��0�Ͷ�ʱ��1
	InitT1();
//	TI=0;
//	RI=0;

//	SwitchControl=0;	 //���ջ��Ŀ��ƶ�Ϊ�ߵ�ƽ�����رս��ջ� 

	Turn=1;		  		//��������1���������Ƿ�Ϊ0
	ModeChange=1;		//P04��1���������Ƿ�Ϊ0�����ı�ģʽ���˳�ģʽ����ͨģʽ

	BatteryControl=0;	//�����ϵ��ʱ����0�������Գ�磬�����û�г����������Ϊ�͵�ƽ

	Moto=1;				//�ر����
	SwitchControl=0;	 //������ջ��Ŀ��ƶ�Ϊ�ߵ�ƽ

//	ES=1;
	ET0=1;				 //ʹ�ܶ�ʱ��0��1���ж�
	ET1=1;
//	PS=1;
	PT1=1;				 //���ö�ʱ��1Ϊ�ϸ����ȼ�
	EA=1;				 //�����ж�ʹ��

	myPwm();	//�������

//	P1M1=0x02;
//	P1M2=0x00;
	P10=1;
//	P11=1;

	PAshutdown=0;		  //����ʱ�������Źر�
	Check=GetADCResult(6);//��ص������

	while(1)
	{
		if(Turn==0)
		{
		 	Delay(30);
			if(Turn==0)
			{
//				while(Turn==0);
				if(TurnFlag==0)		 //˵���ǹػ�״̬,�򿪻�
				{
//					SwitchControl=0;	 //������ջ��Ŀ��ƶ�Ϊ�ߵ�ƽ 

					PAshutdown=1;
					SC_Speech(4);
					Delay(200);
					PAshutdown=0;

					ModeControl_1=0;  //�����ģʽ���ƶ�,����ʱ��Ϊ1.5Mģʽ,���������					
			
					if(Check>=0x35a)//���ñȽϵ�ѹ���˴�Ϊ3.3V
					{
						PAshutdown=1;
						SC_Speech(6);//��ѹ������ʾ
						Delay(200);
						PAshutdown=0;
//						poweroverflag=1;			   
					}
					else
					{
						PAshutdown=1;
						SC_Speech(7);//��ѹ��������ʾ
						Delay(200);
						PAshutdown=0;
//						poweroverflag=0;
					}

					StartAll();//�������������Ϳ���ָ��	
					commuFlag=1;//����ͨ��
					TurnFlag=1;
				}
				else
				{	 
//					SwitchControl=0;//�رս��ջ����ƶ�Ϊ�͵�ƽ
					
					PAshutdown=1;
					SC_Speech(5);
					Delay(120);
					PAshutdown=0;

					Moto=1;//ֹͣ�����

//					Check=GetADCResult(6);//��ص������
					if(Check>=0x35a)//���ñȽϵ�ѹ���˴�Ϊ4V
					{
						PAshutdown=1;
						SC_Speech(6);//��ѹ������ʾ
						Delay(120);	
						PAshutdown=0;
//						poweroverflag=1;
					}
					else
					{
						PAshutdown=1;
						SC_Speech(7);//��ѹ��������ʾ
						Delay(120);
						PAshutdown=0;
//						poweroverflag=0;
					}
					commuFlag=0;//�ر�ͨ��

					StopAll();
					Delay3(150);
					StopAll();
					Delay3(150);
					StopAll();
					Delay3(150);

					TurnFlag=0;
					alarmCount2=0;//�屨��������
					alarmFlag2=0;//�屨����־
					alarmCount3=0;//�屨��������
					alarmFlag3=0;//�屨����־
				}
			}
		}
		if(ModeChange==0)
		{
			Delay(20);
			if(ModeChange==0)
			{
//				while(ModeChange==0);
				if(ModeFlag==0&&TurnFlag==1)//����״̬���Ե�ģʽ
				{
					ModeControl_1=1;//�л������ģʽ
					ModeFlag=1;
			
					PAshutdown=1;
					SC_Speech(2);
					Delay(140);
					PAshutdown=0;
				}
				else if(ModeFlag==1&&TurnFlag==1)
				{
					ModeControl_1=0; //�л������ģʽ
					ModeFlag=0;
			
					PAshutdown=1;
					SC_Speech(3);
					Delay(140);
					PAshutdown=0;
				}
			}
		}

		if((alarmFlag2==1)&&(alarmCount2<1))//����2��ʼ��Ӧ�ı���
		{
			alarmCount2++;

			PAshutdown=1;
			SC_Speech(1);
			Delay(160);
			PAshutdown=0;			

			Moto=0;	//����
			Delay(10);
			Moto=1;
			
			PAshutdown=1;
			SC_Speech(1);
			Delay(160);
			PAshutdown=0;

			Moto=0;//����
			Delay(10);
			Moto=1;
		}
		
//		if(alarmCount2>=1)  //���������Ķ���
//		{
//			alarmCount2=0;//�屨��������
//			alarmFlag2=0;//�屨����־
//		}

		if((alarmFlag3==1)&&(alarmCount3<1))//����3��ʼ��Ӧ�ı���
		{
			alarmCount3++;
			
			PAshutdown=1;
			SC_Speech(10);
			Delay(150);
			Moto=0;//����
			Delay(20);
			Moto=1;
	
			SC_Speech(10);
			Delay(150);
			Moto=0;//����
			Delay(20);
			Moto=1;
			PAshutdown=0;
 		}

//		if(alarmCount3>=1) //���������Ķ���	   
//		{
//			alarmCount3=0;//�屨��������
//			alarmFlag3=0;//�屨����־
//		}

		if(Check>=0x377) //��ʾ��س����
		{
			BatteryControl=1;//��©ģʽ������Ϊ����̬	
		}
		else
		{
			BatteryControl=0;//�����û�г����������Ϊ�͵�ƽ
		}

		if(TurnFlag==1)
		{
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

//		if(SwitchControlcount==18000)
//		{
//			SwitchControl=0;
//			SwitchControlcount=0;	
//		}
	}
}

void timeT1() interrupt 3 //��ʱ��1�жϽ�������
{
//	unsigned int newAddr=0;
	TH1=timer1H;//��װ��
	TL1=timer1L;
   
//	if(SwitchControl==1)
//	{
//		SwitchControlcount++;
//	}
//	else
//	{
	if(P11==0)//�������Ϊ�ߵ�ƽ,�е͵�ƽ˵�����ź�
	{
		DataBetween++;
		ComFlag=0;
		if(DataBetween==150)//�͵�ƽ���������ʱ��	
		{
			DataBetween=0;
		}
	}
	else//Ϊ�ߵ�ƽ��
	{
		if(ComFlag==0)//˵����һ���͵�ƽ
		{
			ComFlag=1;
//				RecData<<=1;

			if((DataBetween>60)&&(DataBetween<=100))	//�͵�ƽ������ʱ��С��10ms����Ϊ0
			{
				RecData<<=1;
				RecData &= 0xfe;
				DataTime++;
				T1highcount=0;
			}
			else if((DataBetween>100))//�͵�ƽ������ʱ�����4.5ms����Ϊ1
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

	if(DataTime==8)//˵��һ���ֽڵ������Ѿ�������ȫ
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

	if(receiveFlag==1)	//˵�����յ������ݣ���ʼ����
	{
		receiveFlag=0;	//����ձ�־
		SwitchControl=1;

//		transCode(TxRxBuf,0x1c);//�����յ������ݽ���
 
		switch(myTxRxData[2])//����ָ��
		{
/*
			case ComMode_1://���յ������������͹����ı���1���źţ�˵��������3M�ڣ���������
			{	
//					newAddr=(newAddr|myTxRxData[4])<<8;//�߰�λ
//					newAddr=newAddr+myTxRxData[3];		   //�Ͱ�λ
//					if(PassWord[newAddr]==myTxRxData[5]&&PassWord[newAddr+1]==myTxRxData[6])//������Ƿ������һ��Ȼ��Ե�ס
//					{
//						if(newAddr>=999)
//						{
//							lastAddr=0;
				TestFlag=0;

//					alarmCount2=0;//�屨��������
//					alarmFlag2=0;//�屨����־
//					alarmCount3=0;//�屨��������
//					alarmFlag3=0;//�屨����־
//				SwitchControl=1;
//						}
//						else
//						{
//							 lastAddr+=1;
//							 TestFlag=0;//����������峬ʱ��־
//						}
//				dataFirst=ComMode_1;
//				if(ModeFlag==2||ModeFlag==3)
//				{
//					SC_Speech(0x01);	//�ָ�������������Ӧ��λ����
//				}
			}
			break;
*/				
			case ComMode_2://˵����30m�ڣ��Ѳ�����
			{
				TestFlag=0;//�峬ʱ��־
//					alarmFlag2=1;
				alarmCount2=0;//�屨��������
				alarmFlag2=0;//�屨����־
				alarmCount3=0;//�屨��������
				alarmFlag3=0;//�屨����־
			}
			break;
			
			case ComMode_3:
			{
//					TestFlag=0;//�峬ʱ��־				
				alarmFlag3=1;
				alarmCount2=3;			
			}
			break;
			
			case ComMode_4://����̧���ź�ʹ��
			{
				TestFlag=0;//�峬ʱ��־	
				alarmFlag4=1;//̧�𱨾�
			}
			break;

			case ComMode_5://���������ź�ʹ��
			{
				TestFlag=0;//�峬ʱ��־
				alarmFlag5=1;	//���ر���
			}
			break;
		}
	}
//	}
}

void time0() interrupt 1	//��Ϊ����ϵͳ�Լ���ʱ��
{
	TH0=timer0H;//��װ��
	TL0=timer0L;

	time0Count_3++;

	if(time0Count_3>=60)//����ÿ1S����һ�ε����ݵ�ʱ���־
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

		if(commuFlag==1)//˵��������ͨ��
		{
			ComMode_1_Data();//����ģʽ1�ź�
//			SendData(0xbc);	 //��������
			
			TestFlag++;
			if(TestFlag>=6)//˵���Ѿ�����300M�ˡ��ղ����κ��ź��ˣ�Ҫ������
			{
				TestFlag=7;
				alarmFlag2=1;
				//������Ӧ�������	
			}
 		}

		Check=GetADCResult(6);//��ص������
		time0Count_3=0;
	}
}

void StartAll()	//���Ϳ�ʼ�ź�
{
//	P0M1&=0xfd;	 //����ģʽ	���൱�ط����
//	P0M2&=0xfd;
//	ModeControl_1=0;//���书��ѡ���а�����ȷ��				
//	myPwm();	//�������

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
			if((myTxRxData[i]&0x80)==0x80)//Ϊ1
			{
				P10=0;
				Delay3(110);//��ʱ4.5ms���ϣ����ڶ�ʱ��ռ�����⣬ֻ����������ʱ��ʵ��
			}
			else//Ϊ0�����
			{
				P10=0;
				Delay3(70);//��ʱ2ms�����ڶ�ʱ��ռ�����⣬ֻ����������ʱ��ʵ��
			}
			P10=1;//��̬Ϊ�ߵ�ƽ
			myTxRxData[i]<<=1;
			Delay3(50);//��ʱҪ����2ms
		}
	}
//	codeData(myTxRxData,7);

//	SendNByte(TxRxBuf,28);

//	����
//	SendNByte(myTxRxData,7);

//	PWMCON0=0x00;//�ر�PWM
//	P0M1|=0x02;	 //����ģʽ	���൱�ط����
//	P0M2&=0xfd;
}

void StopAll() //����ֹͣ�ź�
{
//	P0M1&=0xfd;	 //����ģʽ	���൱�ط����
//	P0M2&=0xfd;
//ModeControl_1=0;//���书��ѡ���а�����ȷ��				
//	myPwm();	//�������
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
			if((myTxRxData[i]&0x80)==0x80)//Ϊ1
			{
				P10=0;
				Delay3(110);//��ʱ4.5ms���ϣ����ڶ�ʱ��ռ�����⣬ֻ����������ʱ��ʵ��
			}
			else//Ϊ0�����
			{
				P10=0;
				Delay3(70);//��ʱ2ms�����ڶ�ʱ��ռ�����⣬ֻ����������ʱ��ʵ��
			}
			P10=1;//��̬Ϊ�ߵ�ƽ
			myTxRxData[i]<<=1;
			Delay3(50);//��ʱҪ����2ms
		}
	}
//	codeData(myTxRxData,7);
//	SendNByte(TxRxBuf,28);
//����
//	SendNByte(myTxRxData,7);

//	PWMCON0=0x00;//�ر�PWM
//	P0M1|=0x02;	 //����ģʽ	���൱�ط����
//	P0M2&=0xfd;
}

void ComMode_1_Data()//���ͱ���1
{
//	P0M1&=0xfd;	 //����ģʽ	���൱�ط����
//	P0M2&=0xfd;
	
					
//	myPwm();	//�������
//	ModeTurn=0;
/*
	SendData(0x55);
	SendData(0xf0);
	SendData(0xaa);
	SendData(0x0f);
*/
	unsigned char i,n;

	ModeControl_1=0;//30M���书��

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
			if((myTxRxData[i]&0x80)==0x80)//Ϊ1
			{
				P10=0;
				Delay3(110);//��ʱ4.5ms���ϣ����ڶ�ʱ��ռ�����⣬ֻ����������ʱ��ʵ��
			}
			else//Ϊ0�����
			{
				P10=0;
				Delay3(70);//��ʱ2ms�����ڶ�ʱ��ռ�����⣬ֻ����������ʱ��ʵ��
			}
			P10=1;//��̬Ϊ�ߵ�ƽ
			myTxRxData[i]<<=1;
			Delay3(50);//��ʱҪ����2ms
		}
	}
//	codeData(myTxRxData,7);
//	SendNByte(TxRxBuf,28);
//	����
//	SendNByte(myTxRxData,7);
//	PWMCON0=0x00;//�ر�PWM
//	P0M1|=0x02;	 //����ģʽ	���൱�ط����
//	P0M2&=0xfd;
}


/*
void codeData(unsigned char *doData,unsigned char len)		//���� ,��ƽ1��Ϊ0011����ƽ0��Ϊ1100
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
void transCode(unsigned char *doData,unsigned char len)//���룬�����յ������ݻ�ԭ
{
	unsigned char i,j;
	for(i=0;i<len;i++)
	{
		for(j=0;j<2;j++)
		{
			myTxRxData[i/4]<<=1;

			if((*doData&0x30)==0x30)//˵��Ϊ1
			{
				myTxRxData[i/4]|=0x01;
			}
			else if((*doData&0xc0)==0xc0)  //˵��Ϊ0
			{
				myTxRxData[i/4]&=0xfe;	
			}

			*doData<<=4;
		}
		doData++;
	}
}
*/