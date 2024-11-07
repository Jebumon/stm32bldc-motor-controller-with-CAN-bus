/**************************************************************************//**
 * @file     powerHold.h
 * @brief    Sensing power switch and Enabling power IC, header file
 * @version  V1.0
 * @date     21. April 2024
 * @developer Jebumon K Thomas
 ******************************************************************************/



#ifndef __POWER_HOLD_H
#define __POWER_HOLD_H

#include "main.h"

void inputConfigPowerHold(void);
void outputConfigPowerHold(void);
int sensePowerHold(void);
void powerHold(void);
void enablePowerHold(void);

#endif

