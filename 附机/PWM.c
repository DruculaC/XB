
#include<N79E81x.h>
#include <intrins.h>
#include"PWM.h"
//#include"UART.h"






UINT16 PWM_shadow;

//-----------------------------------------------------------------------------------------------------------
void Init_PWM(void)
{    
    PWMCON0 = 0xC0;                     // Enable PWM and select auto reload mode    

    PWM_shadow = PWMP;
    PWMPH = HIBYTE(PWM_shadow);         // PWM Period
    PWMPL = LOBYTE(PWM_shadow);                               
}
//void PWM_Pin_Recover_IO(void)
//{
//    PWMCON0 = 0x00;// 
//    PWMCON1 = 0x00;// 
//    PWMCON2 = 0x00;// 
//    PWMPH   = 0x00;// 
//    PWMPL   = 0x00;// 
//    PWM0H   = 0x00;// 
//    PWM0L   = 0x01;// 
//    PWM1H   = 0x00;// 
//    PWM1L   = 0x01;// 
//    PWM2H   = 0x00;// 
//    PWM2L   = 0x01;// 
//    PWM3H   = 0x00;// 
//    PWM3L   = 0x01;// PWMP < PWMn Duty, PWM Pin Output High(no-active)
//    PWMCON0 = 0xD0;// PWM Run
//    _nop_ ();      // NOP
//    PWMCON0 = 0x2F;// PWM Stop
//}
void PWM_Channel(E_PWMCNL_SEL Channel)	 //选择PWM输出通道
{
    switch(Channel)
    {
        case E_CHANNEL0:
            PWM_shadow=PWM0_Duty;
            PWM0H = HIBYTE(PWM_shadow);// PWM0 Duty (P0.1)
            PWM0L = LOBYTE(PWM_shadow);
            break;
        case E_CHANNEL1:
            PWM_shadow=PWM1_Duty;
            PWM1H = HIBYTE(PWM_shadow);// PWM1 Duty (P1.6)
            PWM1L = LOBYTE(PWM_shadow);
            break;
        case E_CHANNEL2:
            PWM_shadow=PWM2_Duty;
            PWM2H = HIBYTE(PWM_shadow);// PWM2 Duty (P1.7)
            PWM2L = LOBYTE(PWM_shadow);
            break;
        case E_CHANNEL3:
            PWM_shadow=PWM3_Duty;
            PWM3H = HIBYTE(PWM_shadow);// PWM3 Duty (P0.0)
            PWM3L = LOBYTE(PWM_shadow);
            break; 
    }
}
//void Enabled_Brake_Function(void)
//{
//    PWMCON1 = 0x30;                    // PWM is running. PWM will be stopped when P0.2 is low level.
//}

void myPwm()
{
	PWM_Channel(E_CHANNEL0);           // Select PWM channel.
    Init_PWM();                        // Enable PWM function and set PWM period.  
    //Enabled_Brake_Function();          // Enable Brake funciton.
}



