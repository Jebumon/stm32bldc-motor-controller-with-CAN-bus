/**************************************************************************//**
 * @file     commutation.h
 * @brief    Commutation sequence header file
 * @version  V1.0
 * @date     10. April 2024
 * @developer Jebumon K Thomas
 ******************************************************************************/

//TOTAL HALL STEPS FOR ONE ROTATION = 90.

#ifndef __COMMUTATOR_H
#define __COMMUTATOR_H

#include "main.h"


const int CommuteTable[8] = {// annotation: for example HU=0 means hall sensor pulls HU down to Ground
	
													    0,
															1,
															2,
															3,
															4,
															5,
															6,
															0, 
	
	
													/*  0,
															2,//
															4,//
															3,//
															6,//
															1,//
															5,//
															0,  */
	
	
	
													/*	0,  //optinal
															1,
															3,
															2,
															5,
															6,
															4,
															0  */
	
	
	
													 /* 0, // hall position [-] - No function (access from 1-6) 
															3, // hall position [1] (HU=1, HV=0, HW=0) -> PWM-position 3
															5, // hall position [2] (HU=0, HV=1, HW=0) -> PWM-position 5
															4, // hall position [3] (HU=1, HV=1, HW=0) -> PWM-position 4
															1, // hall position [4] (HU=0, HV=0, HW=1) -> PWM-position 1
															2, // hall position [5] (HU=1, HV=0, HW=1) -> PWM-position 2
															6, // hall position [6] (HU=0, HV=1, HW=1) -> PWM-position 6
															0, // hall position [-] - No function (access from 1-6) */
	
	



														};




void Commutation(int hallPosition, TIM_HandleTypeDef htim, int direction, int duty);



#endif

