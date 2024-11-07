#ifndef DEFINES_H
#define DEFINES_H

#include "main.h"

//#define TIMER_OUTPUT_ACTIVE_LOW   //Output polarity of Timer channels

#define CAN_SHDN_Pin GPIO_PIN_13
#define CAN_SHDN_GPIO_Port GPIOC

#define Hall_U_Pin GPIO_PIN_0
#define Hall_U_GPIO_Port GPIOA
#define Hall_V_Pin GPIO_PIN_1
#define Hall_V_GPIO_Port GPIOA
#define Hall_W_Pin GPIO_PIN_2
#define Hall_W_GPIO_Port GPIOA

#define LoadCurrent_Pin GPIO_PIN_3
#define LoadCurrent_GPIO_Port GPIOA

#define PWR_HOLD_Pin GPIO_PIN_4
#define PWR_HOLD_GPIO_Port GPIOA

#define V_Batt_Pin GPIO_PIN_5
#define V_Batt_GPIO_Port GPIOA

#define ChargingCurrent_Pin GPIO_PIN_6
#define ChargingCurrent_GPIO_Port GPIOA

#define BuzzerOut_Pin GPIO_PIN_7
#define BuzzerOut_GPIO_Port GPIOA

#define IndicatorLEDs_Pin GPIO_PIN_0
#define IndicatorLEDs_GPIO_Port GPIOB

#define V_Charger_Pin GPIO_PIN_1
#define V_Charger_GPIO_Port GPIOB

 /** 3-Phase MOSFET bridge gate (TIM1) GPIO Configuration **/

#define Phase_U_L_Pin GPIO_PIN_15		//PB15     ------> TIM1_CH3N
#define Phase_U_L_GPIO_Port GPIOB

#define Phase_V_L_Pin GPIO_PIN_14   //PB14     ------> TIM1_CH2N
#define Phase_V_L_GPIO_Port GPIOB

#define Phase_W_L_Pin GPIO_PIN_13   //PB13     ------> TIM1_CH1N
#define Phase_W_L_GPIO_Port GPIOB

#define Phase_U_H_Pin GPIO_PIN_10		//PA10     ------> TIM1_CH3 
#define Phase_U_H_GPIO_Port GPIOA

#define Phase_V_H_Pin GPIO_PIN_9		//PA9      ------> TIM1_CH2
#define Phase_V_H_GPIO_Port GPIOA

#define Phase_W_H_Pin GPIO_PIN_8		//PA8      ------> TIM1_CH1
#define Phase_W_H_GPIO_Port GPIOA
/****************************************************************/

#define StatusLed1_Pin GPIO_PIN_4
#define StatusLed1_GPIO_Port GPIOB
#define StatusLed2_Pin GPIO_PIN_5
#define StatusLed2_GPIO_Port GPIOB

//#define currentHallPosition (GPIOA->IDR & 0x0007)  //Reading hall position from GPIOA->IDR and masking unwanted bits.

#define HALL_STEPS_PER_REVOLUTION  90 //TOTAL HALL STEPS FOR ONE ROTATION = 90.(FOR RPM CALCULATION)

#define CLAMP(x, low, high) (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))

#define CalcRPM(hallStepPeriod) (60 * 1/((hallStepPeriod * HALL_STEPS_PER_REVOLUTION)/1000000)) // RPM calculation


typedef struct Time
{ 
	uint32_t upTimeMilli;
	uint64_t upTimeMicro;
	uint64_t currHallEvent;
	uint64_t prevHallEvent;
	double hallStepPeriodMicro;
}Events;



typedef struct power
{
	float batteryVoltage;
	float chargerVoltage;
	float loadCurrent;
	float chrgCurrent;
	float targetRPM;
	float motorRPM;
}PowerDetails;

//uint8_t HallStatus;  //Current Hall status.



#define MAX_DUTY 0xFFFF  //For fully On lower mosfet.
#define ZERO_DUTY 0x0000 //

//U = channel3
//V = channel2
//W = channel1

#define Phase_U  TIM1->CCR3
#define Phase_V  TIM1->CCR2
#define Phase_W  TIM1->CCR1






#endif
