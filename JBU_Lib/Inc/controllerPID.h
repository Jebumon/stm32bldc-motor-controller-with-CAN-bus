/**************************************************************************//**
 * @file     controllerPID.h
 * @brief    PID controller header file.
 * @version  V1.0
 * @date     08. July 2024
 * @developer Jebumon K Thomas
 ******************************************************************************/



#ifndef __CONTROLLER_PID_H
#define __CONTROLLER_PID_H

#include "main.h"

typedef struct ControllerPID{
	
	/*Gain Controls*/
	float Kp;
	float Ki;
	float Kd;
	
	/*Derivative low-pass filter time constant*/
	float tau;
	
	/*Output Limits*/
	float minLim;
	float maxLim;
	
	/*Sample time in seconds*/
	float T;
	
	/*Controller stack*/
	float integrator;
	float prevError;
	float differentiator;
	float prevMeasurement;
	
	/*Output*/
	float output;
	
} PID_Controller;

void  PID_Controller_Init(PID_Controller *pid);
float PID_Controller_Update(PID_Controller *pid, float target, float measurement);


#endif

