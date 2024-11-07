/**************************************************************************//**
 * @file     controllerPID.c
 * @brief    PID Controller source file.
 * @version  V1.0
 * @date     08 July 2024
 * @developer Jebumon K Thomas
 ******************************************************************************/
 
#include "stdio.h"
#include "main.h"
#include "../Inc/controllerPID.h"


void  PID_Controller_Init(PID_Controller *pid){
	
	/*Resetting variables*/
	pid->integrator = 0.0f;
	pid->prevError  = 0.0f;
	
	pid->differentiator  = 0.0f;
	pid->prevMeasurement = 0.0f;
	
	pid->output = 0;

}

float PID_Controller_Update(PID_Controller *pid, float target, float measurement){
	/*Error value*/
	float error = target - measurement;
	
	/*Propotional*/
	float propotional = pid->Kp * error;
	
	
	/*Integral*/
	pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * (error + pid->prevError);
	
	/*Anti-Wind-Up via dynamic integrator clamping*/
	float limMinInt, limMaxInt;
	
	/*Compute integrator limits*/
	if(pid->maxLim > propotional){
		limMaxInt = pid->maxLim - propotional;
		
	}else
	{
		limMaxInt = 0.0f;
	
	}
	
	if(pid->minLim < propotional){
		
		limMinInt = pid->minLim - propotional;
		
	}else{
		
		limMinInt = 0.0f;
		
	}
	
	/*Clamp Integrator*/
	pid->integrator = CLAMP(pid->integrator, limMinInt, limMaxInt);
	
	
	/*Derivative (band-limited differentiator)*/
	
	pid->differentiator = (2.0f * pid->Kd * (measurement - pid->prevMeasurement)  /* Note: derivative on measurement! */
											+ (2.0f * pid->tau - pid->T) * pid->differentiator)
											/ (2.0f * pid->tau + pid->T);
	
	/*Calcutate output*/
	pid->output = propotional + pid->integrator + pid->differentiator;
	
	/*Output limit clamping*/
	pid->output = CLAMP(pid->output, pid->minLim, pid->maxLim);
	
	/*Store error and measurement for next instant*/
	pid->prevError = error;
	pid->prevMeasurement = measurement;
	
	return pid->output;
}




