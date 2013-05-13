


#define HIBYTE(v1)              ((UINT8)((v1)>>8))                      //v1 is UINT16
#define LOBYTE(v1)              ((UINT8)((v1)&0xFF))

//

#define PWMP            0x1d		//频率	 450KHz

#define PWM0_Duty       0x0d		//占空比
#define PWM1_Duty       0x100
#define PWM2_Duty       0x080
#define PWM3_Duty       0x040



typedef bit                   BIT;
typedef unsigned char         UINT8;
typedef unsigned int          UINT16;
typedef unsigned long         UINT32;


typedef enum 
{
    E_CHANNEL0,
    E_CHANNEL1,      
    E_CHANNEL2,      
    E_CHANNEL3,
} E_PWMCNL_SEL;


extern UINT16 PWM_shadow;

void Init_PWM(void);

void PWM_Channel(E_PWMCNL_SEL Channel);	 //选择PWM输出通道


void myPwm();