/*****************************************************************************
 * @file     onBoardRGBLed.h
 * @brief    On board status RGB LED header file.
 * @version  V1.0
 * @date     10. April 2024
 * @developer Jebumon K Thomas
 ******************************************************************************/



#ifndef __ON_BOARD_RGB_LED_H
#define __ON_BOARD_RGB_LED_H

#include "main.h"
#include "stdio.h"

void outPinConfigRGB(void);
void statusOnRGB(TIM_HandleTypeDef timer, int red, int green, int blue, uint16_t * rgbData);

#endif
