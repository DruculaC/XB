
//#define timer1H 0xff  //���ö�ʱ����ʱ�䣬��ʽ65536-timer*FOSC/12��Ȼ�����������
//#define timer1L 0xa4  //�������������11.0592M�ľ�����Ϊ0.000999s
#define timer1H 0xff  //���ö�ʱ����ʱ�䣬��ʽ(65536-timer*FOSC/12)/256��timerΪ��Ҫ��ʱ��ʱ�䣬����������256����timer1L��ֵ
#define timer1L 0x8e  //�������������13.5M�ľ�����Ϊ100us


void InitT1();//��ʼ����ʱ��01