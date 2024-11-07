/**************************************************************************//**
 * @file     VoltageCurrent.c
 * @brief    Sensing Voltages, Currents source file.
 * @version  V1.0
 * @date     12. April 2024
 * @developer Jebumon K Thomas
 ******************************************************************************/
 
#include "stdio.h"
#include "main.h"


//ADC_HandleTypeDef hadc1Temp;
	
void ADC_Select_LoadCurrentCH(ADC_HandleTypeDef hadc)
{
	ADC_HandleTypeDef hadc1 = hadc;
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  //sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

void ADC_Select_VbattCH(ADC_HandleTypeDef hadc)
{
	ADC_HandleTypeDef hadc1 = hadc;
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

void ADC_Select_CurrentChrgCH(ADC_HandleTypeDef hadc)
{
	ADC_HandleTypeDef hadc1 = hadc;
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

void ADC_Select_VchrgCH(ADC_HandleTypeDef hadc)
{
	ADC_HandleTypeDef hadc1 = hadc;
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}
//----------------------------------------------------------
float SenseLoadCurrent(ADC_HandleTypeDef hadc)
{
	ADC_HandleTypeDef hadc1 = hadc;
	ADC_Select_LoadCurrentCH(hadc1);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1,1);
	int loadCurrent = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	return (3.3/4096) * loadCurrent * 2;
}

float SenseBattVoltage(ADC_HandleTypeDef hadc)
{
	ADC_HandleTypeDef hadc1 = hadc;
	ADC_Select_VbattCH(hadc1);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1,1000);
	int battVoltage = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	return (3.3/4096) * battVoltage * 14.33; //*Voltage divider = 1/14.33
}

float SenseChrgCurrent(ADC_HandleTypeDef hadc)
{
	ADC_HandleTypeDef hadc1 = hadc;
	ADC_Select_CurrentChrgCH(hadc1);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1,1000);
	int chrgCurrent = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	return (3.3/4096) * chrgCurrent * 2;
}

float SenseChrgVoltage(ADC_HandleTypeDef hadc)
{
	ADC_HandleTypeDef hadc1 = hadc;
	ADC_Select_VchrgCH(hadc1);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1,1000);
	int chrgVoltage = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	return (3.3/4096) * chrgVoltage *14.33; //*Voltage divider = 1/14.33
}
