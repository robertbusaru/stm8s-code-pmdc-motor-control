/* Includes ------------------------------------------------------------------*/
// PWM MOTOR CONTROL USING TOUCH BUTTON
// STM8S - DISCOVERY 
// R&D Student: BUSARU ROBERT - CRISTIAN 
// Manager: SLAVILA CORNELIU - ALEXANDRU 
#include "stm8s.h"
#include "stm8s_clk.h"
#include "stm8s_conf.h"
#include "tim4millis.h"
#include "stm8s_tim2.h"
#include "stm8s_tim4.h"
#include "stm8s_gpio.h"
#include "stm8s_it.h"
#include "stm8_tsl_api.h"

#define _CLK 1
#define _GPIO 1

// global variables
int MotorSpeed = 0;
int NumberOfStart;
int CheckFlag = 1;

int ButtonCounter = 0;

// tim1 - accelerate to nominal power
int tim1 = 3;
signed int pwm_duty = 1000; // 

// tim2 - decelerate to 0V
int tim2 = 3;
signed int pwm_duty2 = 0;

// tim3 - nominal power

int tim3 = 4;
signed int pwm_duty3 = 1100;

// to make -- liniar acceleration
void delay_us(uint32_t delay_us)
{
  volatile unsigned int num;
  volatile unsigned int t;


  for (num = 0; num < delay_us; num++)
  {
    t = 11;
    while (t != 0)
    {
      t--;
    }
  }
}

// same to delay_us
void delay_ms(uint16_t delay_ms)
{
  volatile unsigned int num;
  for (num = 0; num < delay_ms; num++)
  {
    delay_us(100);
  }
}

void clock_setup(void) 
{
	CLK_DeInit();
	CLK_HSECmd(DISABLE);
	CLK_LSICmd(DISABLE);
	CLK_HSICmd(ENABLE);
	
      while(CLK_GetFlagStatus(CLK_FLAG_HSIRDY) == FALSE);
                
      CLK_ClockSwitchCmd(ENABLE);
      CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);
      CLK_SYSCLKConfig(CLK_PRESCALER_CPUDIV1);
                
      CLK_ClockSwitchConfig(CLK_SWITCHMODE_AUTO, CLK_SOURCE_HSI, 
      DISABLE, CLK_CURRENTCLOCKSTATE_ENABLE);
               
      CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER2, ENABLE);

}

void ExtraCode_Init(void)     //TOUCH BUTTON CHANNEL KEYS ALGORITHM
{

  u8 i;

  /* All keys are implemented and enabled */

  for (i = 0; i < NUMBER_OF_SINGLE_CHANNEL_KEYS; i++)
  {
    sSCKeyInfo[i].Setting.b.IMPLEMENTED = 1;
    sSCKeyInfo[i].Setting.b.ENABLED = 1;
    sSCKeyInfo[i].DxSGroup = 0x01; /* Put 0x00 to disable the DES on these pins */
  }

#if NUMBER_OF_MULTI_CHANNEL_KEYS > 0
  for (i = 0; i < NUMBER_OF_MULTI_CHANNEL_KEYS; i++)
  {
    sMCKeyInfo[i].Setting.b.IMPLEMENTED = 1;
    sMCKeyInfo[i].Setting.b.ENABLED = 1;
    sMCKeyInfo[i].DxSGroup = 0x01; /* Put 0x00 to disable the DES on these pins */
  }
#endif

  enableInterrupts();
}

void motor_control(void)
{	

///////////////////// --- ACCELERATION --- //////////////////

do
{
	if((TSL_GlobalSetting.b.CHANGED) && (TSLState == TSL_IDLE_STATE))
  {
		TIM2_SetCompare1(-1);
    TSL_GlobalSetting.b.CHANGED = 0;
  } 

	TSL_Action();
	
	if(sSCKeyInfo[0].Setting.b.DETECTED)
	{
			TIM2_SetCompare1(pwm_duty);
			pwm_duty--;
			delay_ms(tim1);
	} 
	
	TSL_GlobalSetting.b.CHANGED = 0;
	
} while(pwm_duty >= 0 && pwm_duty <=1000);


///////////////////// --- NOMINAL POWER --- /////////// 

do
{
	if((TSL_GlobalSetting.b.CHANGED) && (TSLState == TSL_IDLE_STATE))
  {
		TIM2_SetCompare1(0);
    TSL_GlobalSetting.b.CHANGED = 0;
  } 

	TSL_Action();
	
	if(sSCKeyInfo[0].Setting.b.DETECTED)
	{
			TIM2_SetCompare1(0);
			pwm_duty3--;
			delay_ms(tim3);
	} 
	
	TSL_GlobalSetting.b.CHANGED = 0;
	
} while(pwm_duty3 >= 1000 && pwm_duty <=1100);


///////////////////// --- BRAKE --- //////////////////

do
{
	if((TSL_GlobalSetting.b.CHANGED) && (TSLState == TSL_IDLE_STATE))
  {
		TIM2_SetCompare1(-1);
    TSL_GlobalSetting.b.CHANGED = 0;
  } 

	TSL_Action();
	
	if(sSCKeyInfo[0].Setting.b.DETECTED)
	{
			TIM2_SetCompare1(pwm_duty2);
			pwm_duty2++;
			delay_ms(tim2);
	} 
	
	TSL_GlobalSetting.b.CHANGED = 0;
	
} while(pwm_duty2 >= 0 && pwm_duty2 <=1000);


if(pwm_duty2 == 1000 && pwm_duty == 0 && pwm_duty3 == 1000 && sSCKeyInfo[0].Setting.b.DETECTED)
	{
		TIM2_SetCompare1(-1);
    TSL_GlobalSetting.b.CHANGED = 0;
		TSL_Action();
	}
		
	pwm_duty2 = 0;
	pwm_duty3 = 1100;
	pwm_duty = 1000;
	TIM2_SetCompare1(-1);

}
///
void brake(void)
{
	TIM2_SetCompare1(-1);
}
///

void main(void)
{
	
	TIM2_DeInit();  // we will use Timer 2 of our microcontroller to generate and controll pwm signals. 
	clock_setup(); // initializing the microcontroller clock
	ExtraCode_Init(); // initializing touch button algorithm to generate number of single channel keys
	GPIO_DeInit(GPIOD); // initializing GPIO to send
	GPIO_Init(GPIOD, GPIO_PIN_4, GPIO_MODE_OUT_PP_HIGH_FAST);
	
	TIM2_OC1Init(TIM2_OCMODE_PWM2, TIM2_OUTPUTSTATE_ENABLE,750, TIM2_OCPOLARITY_HIGH ); 
	TIM2_TimeBaseInit(TIM2_PRESCALER_1, 999);
  TIM2_OC1PreloadConfig(ENABLE);
	TIM2_Cmd(ENABLE);
	
	TSL_Init();
	TSL_Tick_Flags.b.User1_Start_100ms = 1;
	TIM2_SetCompare1(-1);


	while(1)
	{

	motor_control();
	brake();
	
	}

}
