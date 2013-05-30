
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

//��·ѭ��
//sbit onePin=P1^6;
//sbit twoPin=P1^7;


//�����ķ��䲿�ֵĿ��ƶ˿�
//sbit PWMout=P0^1;//������ķ�������ڣ�ʹ������PWM
sbit ModeControl_1=P2^6;//�����ģʽ����,0��Ϊ30Mģʽ��1��Ϊ300Mģʽ
//sbit ModeTurn=P2^7;//��������أ�1��Ϊ���ˣ�0��Ϊ����

//���ջ�����
sbit SwitchControl=P1^3;//1��Ч��0�ر�

//���ᴫ����
sbit ReceWave=P0^7;//���ᴫ���������
sbit SensorControl=P2^5;//���ᴫ�������ƶ�
//��ഫ������һ�Σ��൱����ʼ�źţ�һ�θ����弴��һ��
sbit sensor_on=P0^0;

//�����
sbit MagentControl_1=P2^2;
sbit MagentControl_2=P2^3;

//ʰ�������� AD��6��ͨ��Ϊʰ��������������
sbit VoiceControl=P2^4;//ʰ�������ƶ�

sbit upSignal=P0^4;//̧���ź�
sbit downSignal=P0^3;//�����ź�

//��ؿ��� 	AD��1��ͨ��Ϊ��صĵ������ˣ����Ƴ�翪�أ�0Ϊ��磬1Ϊ�����
sbit BatteryControl=P1^2;

unsigned char count=0;//���ݽ��ղ��ֵļ�����

unsigned int lastAddr=0;//��һ�ν��յ��ı���ĵ�ַ

unsigned int time0Count_1=0;//��Ϊ���ᴫ������������֮���ʱ������ʱ
unsigned int time0Count_2=0;//��Ϊ���ᴫ�����ļ�ʱ
unsigned int time0Count_3=0;//��ʱ��T0�ļ���
unsigned int time0Count_4=0;//��Ϊ̧�������ʱ������ʱ
unsigned int time0Count_5=0;//��Ϊ���������ʱ������ʱ

bit SensorFlag=0; //���ᴫ�����ĵ͵�ƽ��־λ
unsigned char  SensorCount=0; //��Ϊ���ᴫ��������ļ���

unsigned char TestFlag=0;	//1��2��3�ֱ�Ϊÿ�ν��յ��������������ݺ�ļ������ڴ��ڵĳɹ�ָ�����ִ�н�ȥ����Ĳ���
                			//�������3�ζ�û�й��㣬��˵�����ڳ���
unsigned char ModeFlag=1;	//ģʽѡ��λ��1����ģʽ1,2����ģʽ2,3��Ϊģʽ3

bit alarmFlag=0;//���������Ŀ�����־
bit alarmFlag2=0;//����������־2
unsigned char alarmCount=0;//���������Ĵ���

bit threeFlag=0;//��·ѭ�����ر�־

bit voiceFlag=0;
bit downUpFlag=0;  //���غ�̧�����־

bit downFlag=0;//���صı�־
bit upFlag=0;//̧��ı�־
bit downFlagSend=0;//���ط��͵ı�־
bit upFlagSend=0;//̧���͵ı�־

//��Ϊ���պͷ��͵Ļ�����
unsigned char TxRxBuf[28]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

//һ��ͷ�ֽڣ�һ����ַ�ֽڣ�һ�������ֽڣ����������ַ�ֽڣ���������
unsigned char myTxRxData[7]={0x00,0x00,0x00,0x00,0x00,0x00,0x00};//��������ͨ�����ݵĻ�����

unsigned int Check2=0;//�������
unsigned char Check1=0;//��ΪAD���ֵ��ʰ������⣬ƽʱ����Ϊ1V���ڣ����ź�ʱΪ1.5V

bit receiveFlag=0;//���յ����ݱ�־
bit commuFlag=0;//����ͨ�ű�־

unsigned char DataBetween=0;//��Ϊ�������ݵ��м����
unsigned char RecData=0;//���յ�������
unsigned char DataTime=0;//��Ϊ���յ����ݵ���λ��������
bit ComFlag=1;//�������ص�һ����־
//unsigned int newAddr=0;
unsigned char T1highcount=0;	   //��ʱ��T1��û���źŵ�����ʱ�򣬶Ըߵ�ƽ������һ������ĳ��ֵ����Datatime��0
//unsigned char T1highcount2=0;		//��P11Ϊ�͵�ƽʱ������ĸߵ�ƽ����

//���ſ��ؿ��ƣ�1Ϊ�򿪹��ţ�0Ϊ�رչ���
sbit PAshutdown=P1^4;

//��������ת�����
unsigned char magnetflag=0;

//����һ������������ʾ�źŽ��պ󣬶೤ʱ��ʹ���ջ��򿪣�������SwitchControl�ĸߵ�ƽʱ�䡣
unsigned int SwitchControlcount=0;

//����һ����ʶλ����ʾ����һ�Σ��ػ�һ��
unsigned char turnflag=0;

//�����ѹ������־λ
unsigned char powerflag=1;

//���ͱ����źű�־λ
unsigned char commode2_flag=0; //���ͱ���2�ı�־λ
//unsigned char commode3_flag=0; //���ͱ���3�ı�־λ

//��������
void ComMode_1_Data(void);	//����ģʽ1����
void ComMode_2_Data(void);//����ģʽ2����
void ComMode_22_Data(void);//����ģʽ2����
void ComMode_3_Data(void);//����ģʽ3����
void ComMode_4_Data(void);//����̧�����
void ComMode_5_Data(void);//���͵��ر���

//void codeData(unsigned char *doData,unsigned char len);		//���� ,��ƽ1��Ϊ0011����ƽ0��Ϊ1100
//void transCode(unsigned char *doData,unsigned char len);//���룬�����յ������ݻ�ԭ

//t=1ʱ���ӳ�100us����
void Delay3(unsigned int t)
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

//init signal�����ͱ����ź�ǰ����ʼ�źţ����ڽ����ջ����Զ������
void initsignal()
{
	unsigned char k,k1;
	unsigned char mystartbuffer=0xb4;
	for(k1=0;k1<8;k1++)
	{
		for(k=0;k<8;k++)
		{
			if((mystartbuffer&0x80)==0x80)//Ϊ1
			{
				P10=0;
				Delay3(300);//��ʱ4.5ms���ϣ����ڶ�ʱ��ռ�����⣬ֻ����������ʱ��ʵ��
			}
			else//Ϊ0�����
			{
				P10=0;
				Delay3(300);//��ʱ2ms�����ڶ�ʱ��ռ�����⣬ֻ����������ʱ��ʵ��
			}
			P10=1;//��̬Ϊ�ߵ�ƽ
			mystartbuffer<<=1;
			Delay3(100);//��ʱҪ����2ms
		}
		mystartbuffer=0xaa;
		Delay3(50);
	}
	P10=1;
//	Delay3(80);
}

void initsignal2()
{
	unsigned char k,k1;
	unsigned char mystartbuffer=0xb4;
	for(k1=0;k1<8;k1++)
	{
		for(k=0;k<8;k++)
		{
			if((mystartbuffer&0x80)==0x80)//Ϊ1
			{
				P10=0;
				Delay3(205);//��ʱ4.5ms���ϣ����ڶ�ʱ��ռ�����⣬ֻ����������ʱ��ʵ��
			}
			else//Ϊ0�����
			{
				P10=0;
				Delay3(205);//��ʱ2ms�����ڶ�ʱ��ռ�����⣬ֻ����������ʱ��ʵ��
			}
			P10=1;//��̬Ϊ�ߵ�ƽ
			mystartbuffer<<=1;
			Delay3(70);//��ʱҪ����2ms
		}
		mystartbuffer=0xaa;
		Delay3(35);
	}
	P10=1;
//	Delay3(80);
}
void main()
{
//	unsigned int newAddr=0;

	noVoice();

//	InitUART();
	InitT0();
	InitT1();
//	TI=0;
//	RI=0;

//	SwitchControl=1;	 //������ջ������ƶ�Ϊ�͵�ƽ

//	onePin=0;
//	twoPin=0;
	sensor_on=0;

//	upSignal=1;
//	downSignal=1;

//	ES=1;
	ET0=1;
	ET1=1;
//	PS=1;
	PT1=1;
	EA=1;
	P10=1;
//	P11=1;

	BatteryControl=0;	//�����ϵ��ʱ����0�������Գ�磬�����û�г����������Ϊ�͵�ƽ
	myPwm();	//�������
	VoiceControl=1;   //�ϵ��ʱ��ʰ������
	PAshutdown=0;		//�����Źر�

	MagentControl_1=0;//�رմ���
	MagentControl_2=1;
	Delay(27);
	MagentControl_1=0;//������̬Ϊ����ģʽ
	MagentControl_2=0;
	magnetflag=0;
	
	Check2=0x3ff;
	commode2_flag=0;
	Check1=GetADCResult(5);		//ʰ�����ļ��
	Check2=GetADCResult(6);		//�������
	powerflag=0;
	turnflag=0;

	SensorControl=0;

	while(1)
	{
//		Delay(500);
		if(Check1>=0x100)//���ñȽϵ�ѹ���˴�Ϊ3V����
		{
			PAshutdown=1;	//ʰ��������ĳ����ѹʱ���򿪹���
		}
		else
		{
			PAshutdown=0;	//ʰ����һ������ĳ����ѹ����رչ���
		}

		if(Check2>=0x377) //��ʾ��س����
		{
			BatteryControl=1;//��©ģʽ������Ϊ����̬	
		}
		else
		{
			BatteryControl=0;//�����û�г����������Ϊ�͵�ƽ
		}
/*
		if(turnflag==1)
		{
			if(Check2<=0x300)
			{
				PAshutdown=1;
				SC_Speech(8);	//��ѹ��������ʾ
				Delay(120);
				PAshutdown=0;
				powerflag=0;
			}
			else if(Check2>=0x390)
			{
				PAshutdown=1;
				SC_Speech(7);	//��ѹ������ʾ
				Delay(120);
				PAshutdown=0;
				powerflag=1;
			}
			turnflag=0;	
		}
		
		if(Check2<=0x300)
		{
//			powerflag=0;
			PAshutdown=1;
			SC_Speech(8);	//��ѹ��������ʾ
			Delay(120);
			PAshutdown=0;
		}
		else if(Check2>=0x390)
		{
//			powerflag=1;
			PAshutdown=1;
			SC_Speech(7);	//��ѹ������ʾ
			Delay(120);
			PAshutdown=0;
		}
*/
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
			else if((DataBetween>100))//�͵�ƽ������ʱ�����10ms����Ϊ1
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
			if(T1highcount>=150)
			{
				DataTime=0;
				ComFlag=1;
				count=0;
			}		
		}
	}
//	P01=~P01;

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

	if(receiveFlag==1)
	{
		receiveFlag=0;
		SwitchControl=1;
//		SwitchControl=1;
//		transCode(TxRxBuf,0x1c);//�����յ������ݽ���	
//		��������
		switch(myTxRxData[2]) //������֡���������д���
		{
			case CmdStart://���ǿ���ָ�ִ�п�������
			{
//��������뿪����ʱ��Ҫ��������
				ModeControl_1=1; 	//�����ģʽ���ƶ�,����ʱΪ30Mģʽ
			    VoiceControl=1;		//����ʱ������ʰ����
				
				if(magnetflag==0)
				{
					MagentControl_1=1;//��������
					MagentControl_2=0;
					Delay(27);
					MagentControl_1=0;//������̬Ϊ����ģʽ
					MagentControl_2=0;
					magnetflag=1;
				}

				SensorControl=0;
				commuFlag=1; //����ͨ��
				turnflag=1;
			}
			break;

			case CmdStop:     //���ǹػ�ָ�ִ�йػ�����
			{
//���������ػ���ʱ��Ҫ��������
				
				VoiceControl=0;//�ر�ʰ����

				if(magnetflag==1)
				{
					MagentControl_1=0;//��������
					MagentControl_2=1;
					Delay(27);
					MagentControl_1=0;//������̬Ϊ����ģʽ
					MagentControl_2=0;
					magnetflag=0;
				}
				SensorControl=0;//�ر����ᴫ����

				commuFlag=0;//�ر�ͨ��

				alarmFlag=0;//�ر�����־λ
				alarmCount=0;//����������������
				turnflag=1;

//				VoiceControl=1;//ʹ������ʱҪ�ر�ʰ����
//				SC_Speech(12);  //
//				Delay(100);
//	
//				if(Check2>=0x3cf)//���ñȽϵ�ѹ���˴�Ϊ4V,��4.2VΪ��׼
//				{
//					SC_Speech(10);  //����������ʾ
//					Delay(100);
//				}
//				else if(Check2<0x3cf&&Check2>=399)
//				{
//					SC_Speech(9);  //����������ʾ
//					Delay(100);
//				}
//				else if(Check2<0x399&&Check2>=0x366)
//				{
//					SC_Speech(8);  //����������ʾ
//					Delay(100);
//				}
//				else if(Check2<0x366)
//				{
//					SC_Speech(7);  //����������ʾ
//					Delay(100);
//				}
//
//				VoiceControl=0;//����ʰ����
			}
			break;

			case ComMode_1:  //�������͹�����ֻ��ģʽ1��˵�������������ģ����ݲ���Ϊ����ĵ�һ�͵ڶ����ֽڣ�Ϊ������ڵ��������Ŀ�ʼ�ֽڵ��Ǹ���ַ��Ȼ���������֡�������������ݷ��ͳ�ȥ
			{
//					commode2_flag=1;
				ComMode_22_Data();//�ظ�ȷ���ź�
				alarmFlag=0;//�ر�����־λ
				alarmCount=0;//����������������
				commode2_flag=0;  //�رձ���2�źŵķ���
//					commode3_flag=0;

//				lastAddr=newAddr;
				SensorControl=0;	//�ر����ᴫ����
				downUpFlag=0; 		//�ص��ء�̧����
							
//				   	if(TestFlag>=3)//���쳣����ָ�ʱ������������
//				  	{
				if(magnetflag==0)
				{
					MagentControl_1=1;//��������
					MagentControl_2=0;
					Delay(27);
					MagentControl_1=0;//������̬Ϊ����ģʽ
					MagentControl_2=0;
					magnetflag=1;
				}
//					}
				TestFlag=0;	
				
				if(ModeFlag==3||ModeFlag==2)
				{
				//�ָ�������������Ӧ��λ����
					ModeFlag=1;
				}
			}
			break;
/*
				case ComMode_1:  //�������͹�����ֻ��ģʽ1��˵�������������ģ����ݲ���Ϊ����ĵ�һ�͵ڶ����ֽڣ�Ϊ������ڵ��������Ŀ�ʼ�ֽڵ��Ǹ���ַ��Ȼ���������֡�������������ݷ��ͳ�ȥ
				{


//							newAddr=(newAddr|myTxRxData[4])<<8;//�߰�λ
//							newAddr=newAddr+myTxRxData[3];		   //�Ͱ�λ

	
	
							if(PassWord[lastAddr+1]==myTxRxData[5]&&PassWord[lastAddr+2]==myTxRxData[6])//������Ƿ������һ��Ȼ��Ե�ס
							{
								
//								if(newAddr==0 &&( (lastAddr-newAddr)>=997 ||(lastAddr-newAddr)==0))
//								{ 
								
//								if(newAddr>=999)
//								{
//									newAddr=0;
//									lastAddr=newAddr;//������ε������ַ��Ϊ�´ε�ַ��У��ʹ��
//								}
//								else 
//								{
//									lastAddr=newAddr;//������ε������ַ��Ϊ�´ε�ַ��У��ʹ��
//									newAddr+=1;//��ַ����һ��
//								}

								ComMode_1_Data(newAddr);//�ظ�ȷ���ź�
								
	
								alarmFlag=0;//�ر�����־λ
								alarmCount=0;//����������������
	
								lastAddr=newAddr;
								SensorControl=0;//�ر����ᴫ����
								
							   if(TestFlag>=3)//���쳣����ָ�ʱ������������
							   {
									MagentControl_1=0;//��������
									MagentControl_2=1;
									Delay(1);
//									MagentControl_1=1;//������̬Ϊ����ģʽ
//									MagentControl_2=1;
								
									MagentControl_1=0;//������̬Ϊ����ģʽ
									MagentControl_2=0;
								}
	
								TestFlag=0;	
								
								if(ModeFlag==3||ModeFlag==2)
								{
								//�ָ�������������Ӧ��λ����
									ModeFlag=1;
								}
							}

							else//��ַ���������
							{
								newAddr=select(myTxRxData[5]);	//���ҵ�һ������ĵ�ַ
								if(newAddr!=1001)//�ҵ������
								{
									if(newAddr>=999)
									{
										newAddr=0;
										lastAddr=newAddr;//������ε������ַ��Ϊ�´ε�ַ��У��ʹ��
									}
									else
									{
										lastAddr=newAddr;//������ε������ַ��Ϊ�´ε�ַ��У��ʹ��
										newAddr+=1;//��ַ����һ��
									}

									ComMode_1_Data(newAddr);//�ظ�ȷ���ź�

									alarmFlag=0;//�ر�����־λ
									alarmCount=0;//����������������
		
									lastAddr=newAddr;
									SensorControl=0;//�ر����ᴫ����
									
								   if(TestFlag>=3)//���쳣����ָ�ʱ������������
								   {
										MagentControl_1=0;//��������
										MagentControl_2=1;
										Delay(1);
										MagentControl_1=0;//������̬Ϊ����ģʽ
										MagentControl_2=0;
									}
		
									TestFlag=0;	
									
									if(ModeFlag==3||ModeFlag==2)
									{
									//�ָ�������������Ӧ��λ����
										ModeFlag=1;
									}
								}
							}
	//							}
	//							else if(newAddr<=1000&&(newAddr-lastAddr<=3))
	//							else if((newAddr-lastAddr)<=3)
	//							{
	//								
	//								TestFlag=0;
	//								lastAddr=newAddr;
	//								SensorControl=1;//�ر����ᴫ����
	//								ComMode_1_Data(newAddr);//�ظ�ȷ���ź�
	//								if(ModeFlag==2)
	//								{
	//								//�ָ�������������Ӧ��λ����
	//									ModeFlag=1;
	//								}
	//							}
	
							}
				}
				break;
*/	
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
		if(commuFlag==1)//˵��������ͨ��
		{
			TestFlag++;

			if(TestFlag>=3&&ModeFlag==1)//˵��û�н��յ������Ѿ���3���ˣ������Ѿ�����3M�����ھ�Ҫ�Ӵ��ʣ��л���ģʽ2,30M�ٿ��ܲ��ܽ��յ�����
			{
				TestFlag=5;
				if(ModeFlag==1)
				{
//					ComMode_2_Data(lastAddr);//�򸽻����ͱ���2
//					SendData(0xac);//��������	
					if(magnetflag==1)
					{
						MagentControl_1=0;//��������
						MagentControl_2=1;
						Delay(27);
						MagentControl_1=0;//������̬Ϊ����ģʽ
						MagentControl_2=0;
						magnetflag=0;
					}
		
					SensorControl=1;//�������ᴫ����
					downUpFlag=1;//�������ء�̧���־
					ModeFlag=2;
	
//					ComMode_2_Data();//�򸽻����ͱ���2
					commode2_flag=1;
				}	
			}

			if(commode2_flag==1)
			{
				ComMode_2_Data();	//�򸽻����ͱ���2	
			}
		}
		time0Count_3=0;
		
		Check1=GetADCResult(5);//ʰ�����ļ��
		
		Check2=GetADCResult(6);//�������
	}

	if(SensorControl==1)//������ᴫ�����Ƿ��
	{
		if(ReceWave==0)//˵���д����������ʼ��ʱ
		{
			time0Count_2++;
			if(time0Count_2>=30)//˵���Ѿ�����0.5S
			{
				time0Count_2 =0;//��ʱ������
				SensorCount++;//���ᴫ�������������1
				alarmFlag2=1;//
			}		
		}
		else if(ReceWave==1&&SensorCount!=0)//˵���Ѿ���һ�����õ�����
		{
			time0Count_1++;
			if(time0Count_1>=100)//����5S
			{
				SensorCount=0;
			}
		}
	}
 
	if(ModeFlag==2&&SensorCount>=1)//���ᴫ�����������Ӧ����
	{
		if(SensorCount==1&&alarmFlag2==1)//���ᴫ����һ�δ���,alarmFlag2���Ʒ���1��
		{					
			VoiceControl=0;//ʹ������ʱҪ�ر�ʰ����
			PAshutdown=1;  //�ȹر�ʰ������Ȼ��򿪹���
			SC_Speech(1);  //�ػ���������
			Delay(140);
			PAshutdown=0;  //������ɺ󣬹رչ��ţ�Ȼ���ʰ����
			VoiceControl=1;//����ʰ����
			alarmFlag2=0;
		}
/*
		if(SensorCount==2&&alarmFlag2==1)//���ᴫ�������δ�����alarmFlag2���Ʒ���1��
		{
			VoiceControl=0;//ʹ������ʱҪ�ر�ʰ����
			PAshutdown=1;
			SC_Speech(2);  //�ػ���������
			Delay(140);
			PAshutdown=0;
			VoiceControl=1;//����ʰ����
			alarmFlag2=0;
		}
*/
		if(SensorCount>=2)//���ᴫ����һ�δ���
		{
//			ComMode_3_Data(); //�򸽻����ͱ���3
			ModeFlag=3;//���ᴫ�����Ѿ���3�δ����ˣ�Ҫ�ı䷢��ģʽ��
			alarmFlag=1;//����������λ
			alarmFlag2=0;
			SensorCount=0; //�����������
			Delay(1);
			commode2_flag=0;
			ComMode_3_Data(); //�򸽻����ͱ���3
//			commode3_flag=1;
		}
	}

	if(ModeFlag==3)
	{
		if(alarmFlag==1)
		{
			VoiceControl=0;//ʹ������ʱҪ�ر�ʰ����
			PAshutdown=1;
			SC_Speech(3);  //�ػ���������
			Delay(120);
			PAshutdown=0;
			VoiceControl=1;//����ʰ����
		}
		if(alarmFlag==1)
		{
			VoiceControl=0;//ʹ������ʱҪ�ر�ʰ����
			PAshutdown=1;
			SC_Speech(4);  //�ػ���������
			Delay(190);
			PAshutdown=0;
			VoiceControl=1;//����ʰ����
		}
		if(ModeFlag==3)
		{
			ComMode_3_Data(); //�򸽻����ͱ���3
		}
		if(alarmFlag==1)
		{
			VoiceControl=0;//ʹ������ʱҪ�ر�ʰ����
			PAshutdown=1;
			SC_Speech(3);  //�ػ���������
			Delay(120);
			PAshutdown=0;
			VoiceControl=1;//����ʰ����
		}
		if(alarmFlag==1)
		{
			VoiceControl=0;//ʹ������ʱҪ�ر�ʰ����
			PAshutdown=1;
			SC_Speech(11);  //�ػ���������
			Delay(190);
			PAshutdown=0;
			VoiceControl=1;//����ʰ����
		}
		if(ModeFlag==3)
		{
			ComMode_3_Data(); //�򸽻����ͱ���3
		}
		if(alarmCount>=20) //���������Ķ���
		{
			alarmCount=0;//�屨��������
			alarmFlag=0;//�屨����־
		}
		alarmCount++;
	}
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
	ModeControl_1=1;//30M���书��				

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
			if((myTxRxData[i]&0x80)==0x80)//Ϊ1
			{
				P10=0;
				Delay3(120);//��ʱ4.5ms���ϣ����ڶ�ʱ��ռ�����⣬ֻ����������ʱ��ʵ��
			}
			else//Ϊ0�����
			{
				P10=0;
				Delay3(80);//��ʱ2ms�����ڶ�ʱ��ռ�����⣬ֻ����������ʱ��ʵ��
			}
			P10=1;//��̬Ϊ�ߵ�ƽ
			myTxRxData[i]<<=1;
			Delay3(50);//��ʱҪ����2ms
		}
	}

//	codeData(myTxRxData,7);
//	SendNByte(TxRxBuf,28);
//	PWMCON0=0x00;//�ر�PWM
//	P0M1|=0x02;	 //����ģʽ	���൱�ط����
//	P0M2&=0xfd;
//	ModeTurn=1;
}

void ComMode_2_Data()//���ͱ���2
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
	ModeControl_1=1;//30M���书��

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
			if((myTxRxData[i]&0x80)==0x80)//Ϊ1
			{
				P10=0;
				Delay3(120);//��ʱ4.5ms���ϣ����ڶ�ʱ��ռ�����⣬ֻ����������ʱ��ʵ��
			}
			else//Ϊ0�����
			{
				P10=0;
				Delay3(80);//��ʱ2ms�����ڶ�ʱ��ռ�����⣬ֻ����������ʱ��ʵ��
			}
			P10=1;//��̬Ϊ�ߵ�ƽ
			myTxRxData[i]<<=1;
			Delay3(50);//��ʱҪ����2ms
		}
	}
//	codeData(myTxRxData,7);	//���б���
//	SendNByte(TxRxBuf,28);
//	PWMCON0=0x00;//�ر�PWM
//	P0M1|=0x02;	 //����ģʽ	���൱�ط����
//	P0M2&=0xfd;
//	ModeTurn=1;
}

void ComMode_22_Data()//���ͱ���2
{
	unsigned char i,n;
	ModeControl_1=1;//30M���书��

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
			if((myTxRxData[i]&0x80)==0x80)//Ϊ1
			{
				P10=0;
				Delay33(120);//��ʱ4.5ms���ϣ����ڶ�ʱ��ռ�����⣬ֻ����������ʱ��ʵ��
			}
			else//Ϊ0�����
			{
				P10=0;
				Delay33(80);//��ʱ2ms�����ڶ�ʱ��ռ�����⣬ֻ����������ʱ��ʵ��
			}
			P10=1;//��̬Ϊ�ߵ�ƽ
			myTxRxData[i]<<=1;
			Delay33(50);//��ʱҪ����2ms
		}
	}
}

void ComMode_3_Data()//���ͱ���3
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
	ModeControl_1=0;//�л�Ϊ300M����

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
			if((myTxRxData[i]&0x80)==0x80)//Ϊ1
			{
				P10=0;
				Delay3(120);//��ʱ4.5ms���ϣ����ڶ�ʱ��ռ�����⣬ֻ����������ʱ��ʵ��
			}
			else//Ϊ0�����
			{
				P10=0;
				Delay3(80);//��ʱ2ms�����ڶ�ʱ��ռ�����⣬ֻ����������ʱ��ʵ��
			}
			P10=1;//��̬Ϊ�ߵ�ƽ
			myTxRxData[i]<<=1;
			Delay3(50);//��ʱҪ����2ms
		}
	}
//	codeData(myTxRxData,7);	//���б���
//	SendNByte(TxRxBuf,28);
//	PWMCON0=0x00;//�ر�PWM
//	P0M1|=0x02;	 //����ģʽ	���൱�ط����
//	P0M2&=0xfd;
//	ModeTurn=1;
	ModeControl_1=1;		//���ͱ���3��ɺ󣬽�ģʽ���л���30m����
}

void ComMode_4_Data()//����̧�����
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
	ModeControl_1=0;//�л�Ϊ300M����

	myTxRxData[0]=CmdHead;
	myTxRxData[1]=MyAddress;
	myTxRxData[2]=ComMode_4;
	myTxRxData[3]=0x00;
	myTxRxData[4]=0x00;
	myTxRxData[5]=0x00;
	myTxRxData[6]=0x00;

	initsignal2();

	for(i=0;i<7;i++)
	{
		for(n=0;n<8;n++)
		{
			if((myTxRxData[i]&0x80)==0x80)//Ϊ1
			{
				P10=0;
				Delay3(120);//��ʱ4.5ms���ϣ����ڶ�ʱ��ռ�����⣬ֻ����������ʱ��ʵ��
			}
			else//Ϊ0�����
			{
				P10=0;
				Delay3(80);//��ʱ2ms�����ڶ�ʱ��ռ�����⣬ֻ����������ʱ��ʵ��
			}
			P10=1;//��̬Ϊ�ߵ�ƽ
			myTxRxData[i]<<=1;
			Delay3(50);//��ʱҪ����2ms
		}
	}
//	codeData(myTxRxData,7);	//���б���
//	SendNByte(TxRxBuf,28);
//	PWMCON0=0x00;//�ر�PWM
//	P0M1|=0x02;	 //����ģʽ	���൱�ط����
//	P0M2&=0xfd;
//	ModeTurn=1;
}

void ComMode_5_Data()//���͵��ر���
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
	ModeControl_1=0;//�л�Ϊ300M����

	myTxRxData[0]=CmdHead;
	myTxRxData[1]=MyAddress;
	myTxRxData[2]=ComMode_5;
	myTxRxData[3]=0x00;
	myTxRxData[4]=0x00;
	myTxRxData[5]=0x00;
	myTxRxData[6]=0x00;

	initsignal2();

	for(i=0;i<7;i++)
	{
		for(n=0;n<8;n++)
		{
			if((myTxRxData[i]&0x80)==0x80)//Ϊ1
			{
				P10=0;
				Delay3(120);//��ʱ4.5ms���ϣ����ڶ�ʱ��ռ�����⣬ֻ����������ʱ��ʵ��
			}
			else//Ϊ0�����
			{
				P10=0;
				Delay3(80);//��ʱ2ms�����ڶ�ʱ��ռ�����⣬ֻ����������ʱ��ʵ��
			}
			P10=1;//��̬Ϊ�ߵ�ƽ
			myTxRxData[i]<<=1;
			Delay3(50);//��ʱҪ����2ms
		}
	}
//	codeData(myTxRxData,7);	//���б���
//	SendNByte(TxRxBuf,28);
//	PWMCON0=0x00;//�ر�PWM
//	P0M1|=0x02;	 //����ģʽ	���൱�ط����
//	P0M2&=0xfd;
//	ModeTurn=1;	
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
