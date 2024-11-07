/**************************************************************************//**
 * @file     VoltageCurrent.h
 * @brief    Sensing Voltages, Currents header file.
 * @version  V1.0
 * @date     12. April 2024
 * @developer Jebumon K Thomas
 ******************************************************************************/



#ifndef __VOLTAGE_CURRENT_H
#define __VOLTAGE_CURRENT_H

#include "main.h"

float SenseLoadCurrent(ADC_HandleTypeDef hadc);
float SenseBattVoltage(ADC_HandleTypeDef hadc);
float SenseChrgVoltage(ADC_HandleTypeDef hadc);
float SenseChrgCurrent(ADC_HandleTypeDef hadc);

#endif

