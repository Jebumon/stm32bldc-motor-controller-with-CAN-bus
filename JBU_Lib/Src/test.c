/**************************************************************************//**
 * @file     test.c
 * @brief    Testing motor functions.
 * @version  V1.0
 * @date     2. June 2024
 * @developer Jebumon K Thomas
 ******************************************************************************/
 
#include "stdio.h"
#include "main.h"

int testSpeedLoop = 0;
int speed = 10;
int tempDuty = 0;

void testSpeed()
{
	if(testSpeedLoop <= 100 && TIM1-> CCR1 <= 0x000007B0 && TIM1-> CCR1 >= 0x00000700)
		{
		tempDuty = TIM1->CCR1;
		TIM1->CCR1 = tempDuty + speed; //COMPARE VALUE(Dutycycle);
		TIM1->CCR2 = tempDuty + speed; //COMPARE VALUE(Dutycycle);
		TIM1->CCR3 = tempDuty + speed; //COMPARE VALUE(Dutycycle);
		testSpeedLoop++;
		return;
		}
		if(TIM1-> CCR1 >= 0x000007B0){TIM1-> CCR1 = 0x000007B0;}
		if(TIM1-> CCR1 <= 0x00000700){TIM1-> CCR1 = 0x00000700;}
		if(TIM2-> CNT  >  1000){TIM2-> CNT = 0;} // Resetting timer for Starting Interrupt.
		
		testSpeedLoop = 0;
		//speed = speed * -1;
}

void targetSpeed(int dutyCycle){
	
		TIM1->CCR1 = dutyCycle; //COMPARE VALUE(Dutycycle);
		TIM1->CCR2 = dutyCycle; //COMPARE VALUE(Dutycycle);
		TIM1->CCR3 = dutyCycle; //COMPARE VALUE(Dutycycle);
	
}
  


