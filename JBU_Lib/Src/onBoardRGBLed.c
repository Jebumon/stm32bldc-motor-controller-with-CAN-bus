/*****************************************************************************
 * @file     onBoardRGBLed.c
 * @brief    On board status RGB LED source file.
 * @version  V1.0
 * @date     10. April 2024
 * @developer Jebumon K Thomas
 ******************************************************************************/

#include "stdio.h"
#include "main.h"



void outPinConfigRGB()
{
	/*
	//set timer3 ARR value
	//set timer3 CRR2 value
	//set timer3 CCER-> CC2E(channel 2 output enable)
	//set AFIO -> MAPR -> TIM3_REMAP = 0X02 (Remap from GPIOA to GPIOB)
	//set GPIOB-> CRL -> MODE5 = 0X02 (00: Input mode (reset state)
																		 01: Output mode, max speed 10 MHz.
																		 10: Output mode, max speed 2 MHz.
																		 11: Output mode, max speed 50 MHz.)
	
	//								-> CNF5  = 0X02  In output mode (MODE[1:0] >?00)
																		 00: General purpose output push-pull
																		 01: General purpose output Open-drain
																		 10: Alternate function output Push-pull
																		 11: Alternate function output Open-drain
	*/
	
	//AFIO-> MAPR |= 0b000000000000000100000000000; //TIM3_REMAP = 0X02
	AFIO-> MAPR |=(1 << 11);		//TIM3_REMAP = 0X02 (Remap from GPIOA to GPIOB)
	AFIO-> MAPR &= ~(1 << 10);	//TIM3_REMAP = 0X02 (Remap from GPIOA to GPIOB)
	GPIOB-> CRL |=(1 << 21);		//MODE5 = 0X02 (Output mode, max speed 2 MHz.)
	GPIOB-> CRL &= ~(1 << 20);	//MODE5 = 0X02 (Output mode, max speed 2 MHz.)
	GPIOB-> CRL |=(1 << 23);		//CNF5 = 0X02 (Alternate function output Push-pull)
	GPIOB-> CRL &= ~(1 << 22);	//CNF5 = 0X02 (Alternate function output Push-pull)
	TIM3-> ARR  = 90;
	TIM3-> CCR2 = 0;
	TIM3-> CCER |=(1 << 4);			//Timer3 channel 2 output enable	
}



void statusOnRGB(TIM_HandleTypeDef timer, int green, int red, int blue, uint16_t * RGBpwmData)
{
	outPinConfigRGB();
	
	uint32_t color = (green<<16) | (red<<8) | blue;
	
	for (int i= 24; i>= 0; i--)
	{
		if(color&(1<<i))RGBpwmData[i] = 60;
		else RGBpwmData[i] = 30;
	}
	//HAL_TIM_PWM_Start(&timer, 2);
	TIM3-> CR1 |=(1 << 0);			//Timer3 counter enable
	int bitCount = 25;
	while(bitCount >= 1)
	{
		//int counter = TIM3->CNT;
		while(TIM3->CNT < 20)
		{
			TIM3->CCR2 = RGBpwmData[bitCount-2];
			bitCount--;
		}
	}
	TIM3->CCR2 = 0;
	HAL_TIM_Base_Stop(&timer);
	return;
}

