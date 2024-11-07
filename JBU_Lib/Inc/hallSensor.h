/**************************************************************************//**
 * @file     hallSensor.h
 * @brief    Hall sensor header file
 * @version  V1.0
 * @date     30. June 2024
 * @developer Jebumon K Thomas
 ******************************************************************************/

//TOTAL HALL STEPS FOR ONE ROTATION = 90.

#ifndef __HALLSENSOR_H
#define __HALLSENSOR_H

#include "main.h"
#include "..\..\JBU_Lib\Inc\defines.h"

uint8_t HallStatus;  //Current Hall status.


uint32_t getMicroSec(struct Time);

#endif

