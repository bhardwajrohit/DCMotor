/*
   FILE          : dcmotor.c
   PROJECT       : DC Motor/Stm32f3 Discovery Board/Linux
   PROGRAMMER    : Rohit Bhardwaj
   DESCRIPTION   : This program configures GIPO port A,F to send binary data to control the speed(using PWM)and direction of a DC motor. 
		           Moreover, a motion profile is implemented. When the motor speed is set, it ramps up to the speed, and then down slowly 
				   where it should stop. Timer 1 is configured to generate the PWM and also an Interrupt flag(UIF) to control the time the 
				   motor is turned on for.
	
	The program make use of HAL(Hardware Abstraction Layer) which is C code that implements basic drivers for all the peripherals 
	in the STM32 Family. It makes the code more portable over STM32 Family.
*/

#include <stdint.h>
#include <stdio.h>
#include "stm32f3xx_hal.h"
#include "common.h"

uint32_t enablebreathe = 0;

// FUNCTION      : gpioinit1()
// DESCRIPTION   : The function initialize GPIO pins for the DC motor
// PARAMETERS    : The function accepts an int value
// RETURNS       : returns nothing
void gpioinit(int mode)
{
/* Turn on clocks to I/O */
__GPIOF_CLK_ENABLE();

/* Configure GPIO pins */
GPIO_InitTypeDef  GPIO_InitStruct;
GPIO_InitStruct.Pin = (GPIO_PIN_2 | GPIO_PIN_4);
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
GPIO_InitStruct.Alternate = 0;
HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
return;

}
ADD_CMD("gpioinit",gpioinit,"              Initialize GPIO Pins");


/*Global Handle Structure*/
TIM_HandleTypeDef tim1;
TIM_OC_InitTypeDef sConfig;

// FUNCTION      : dcinit()
// DESCRIPTION   : The function initializes the timer 1 and enables the interrupt
// PARAMETERS    : The function accepts an int value
// RETURNS       : returns nothing
void dcinit(int mode)
{
/* Turn on clocks to I/O */
__GPIOA_CLK_ENABLE();

/* Configure GPIO pins */
GPIO_InitTypeDef  GPIO_InitStruct;
GPIO_InitStruct.Pin = (GPIO_PIN_8 | GPIO_PIN_9);
GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
GPIO_InitStruct.Alternate = 6;
HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

uint32_t rc;

/* Initialize the Timer(PWM) */ 
__TIM1_CLK_ENABLE(); 
tim1.Instance = TIM1; 
tim1.Init.Prescaler     = HAL_RCC_GetPCLK2Freq()*2/1000000; 
tim1.Init.CounterMode   = TIM_COUNTERMODE_UP; 
tim1.Init.Period        = 1000; 
tim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; 
tim1.Init.RepetitionCounter = 0; 

/*Initalize the Timer*/
rc = HAL_TIM_Base_Init(&tim1);
if(rc != HAL_OK) 
 {
  printf("Unable to initalize Timer, rc=%d\n",(unsigned)rc);
  return;
 }

/*Start the timer*/
 rc = HAL_TIM_Base_Start(&tim1);
 if(rc != HAL_OK) 
 {
  printf("Unable to start the timer, rc=%d\n",(unsigned)rc);
  return;
 }

/*Configure output:*/
 
sConfig.OCMode       = TIM_OCMODE_PWM1; 
sConfig.Pulse        = 500; 
sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH; 
sConfig.OCNPolarity  = TIM_OCNPOLARITY_LOW; 
sConfig.OCFastMode   = TIM_OCFAST_DISABLE; 
sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET; 
sConfig.OCNIdleState =TIM_OCNIDLESTATE_RESET;
HAL_TIM_PWM_ConfigChannel(&tim1,&sConfig,TIM_CHANNEL_1);
HAL_TIM_PWM_ConfigChannel(&tim1,&sConfig,TIM_CHANNEL_2);

//Enable the interrupt 
 HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 0, 1);
 HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
 TIM1-> DIER |= 1;
 TIM1->CNT = 0;       /* Reset counter */
}

ADD_CMD("dcinit",dcinit,"              Initialize the Timer & GPIO Pins");


// FUNCTION      : dc()
// DESCRIPTION   : The function set DC motor direction
//                 0 – Brake 
//                 1 – Forward 
//				   2 – Reverse
// PARAMETERS    : The function accepts an int value
// RETURNS       : returns nothing
void dc(int mode)     
{
  if(mode != CMD_INTERACTIVE)
  {
    return;
  }

 uint32_t dir;

 fetch_uint32_arg(&dir);

 HAL_TIM_PWM_Start(&tim1,TIM_CHANNEL_1); 

 if(dir == 0)
 {
   /* Brake */ 
   HAL_GPIO_WritePin(GPIOF,GPIO_PIN_2,0);    //1A
   HAL_GPIO_WritePin(GPIOF,GPIO_PIN_4,0);    //2A
 }

 else if(dir == 1)
  {
   /* Forward */ 
   HAL_GPIO_WritePin(GPIOF,GPIO_PIN_2,1);    //1A
   HAL_GPIO_WritePin(GPIOF,GPIO_PIN_4,0);   //start 2A
  }

 else if(dir == 2)
  {
   /* Reverse*/  
   HAL_GPIO_WritePin(GPIOF,GPIO_PIN_2,0);    //1A
   HAL_GPIO_WritePin(GPIOF,GPIO_PIN_4,1);    //stop 2A
  }
}
ADD_CMD("dc",dc,"                  dc<dir>");

// FUNCTION      : pwm()
// DESCRIPTION   : The function controls the speed of the DC Motor
// PARAMETERS    : The function accepts an int value
// RETURNS       : returns nothing
void pwm(int mode)     
{
  if(mode != CMD_INTERACTIVE)
  {
    return;
  }

 uint32_t value; 

 fetch_uint32_arg(&value);

 TIM1->CCR1 = value;  //Sets the speed of the motor
}

ADD_CMD("pwm",pwm,"                  pwm<value>");


// FUNCTION      : TIM1_UP_TIM16_IRQHandler()
// DESCRIPTION   : The function implements a motion profile, when the motor speed is set, it ramps up to the speed, 
//                 and then down slowly where it should stop.
// PARAMETERS    : nothing
// RETURNS       : returns nothing
void TIM1_UP_TIM16_IRQHandler(void)    //startup_stm32f303xc.s
{
  static int i = 0;
  static int increment = 1;
  TIM1 ->SR &= ~1;//~UIF  here we actually need to reset the flag in timer
 if(enablebreathe == 1)
{
  TIM1->CCR1 = i;//start from zero by setting it to zero
  if (increment) //check if 
 {
   i++;         //increment the value in i
  if(i >= 1000) //when it reaches to maximum ie 1000
  { 
    increment = 0; // change the condition by making it zero
   }  
  }
}
  else if (enablebreathe == 2)  //now start the decrementation 
  {
    TIM1->CCR1 = i;
   if(increment ==0)
   {
     i--;     //decrement it
    if(i == 0) //when it reaches to zero
    {
   HAL_GPIO_WritePin(GPIOF,GPIO_PIN_2,0);    //1A
   HAL_GPIO_WritePin(GPIOF,GPIO_PIN_4,0); //reanalyze it back to 1
    increment = 1;
    }
   } 
  }  
}

// FUNCTION      : monitor()
// DESCRIPTION   : The function fetches the value from the commands
// PARAMETERS    : The function accepts an int value
// RETURNS       : returns nothing
 void monitor(int mode)
{
    if(mode != CMD_INTERACTIVE)
  {
    return;
  }

  fetch_uint32_arg(&enablebreathe);
}

ADD_CMD("enable",monitor,"              enablebreathe");
