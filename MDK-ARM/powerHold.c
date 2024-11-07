#include "main.h"
#include "stdio.h"

GPIO_InitTypeDef GPIO_InitStruct = {0};
void inputConfigPowerHold(void)
{
	  /*Configure GPIO pin : PWR_HOLD_Pin */
  GPIO_InitStruct.Pin = PWR_HOLD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PWR_HOLD_GPIO_Port, &GPIO_InitStruct);
}
void outputConfigPowerHold(void)
{
	GPIO_InitStruct.Pin = PWR_HOLD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PWR_HOLD_GPIO_Port, &GPIO_InitStruct);
}
void enablePowerHold(void)
{
	HAL_GPIO_WritePin(PWR_HOLD_GPIO_Port, PWR_HOLD_Pin, GPIO_PIN_SET);
}

int sensePowerHold(void)
{
	int switchStatus = HAL_GPIO_ReadPin(PWR_HOLD_GPIO_Port,PWR_HOLD_Pin);
	if(switchStatus == 1)
	{
		HAL_Delay(1000);
		switchStatus = HAL_GPIO_ReadPin(PWR_HOLD_GPIO_Port,PWR_HOLD_Pin);
		if(switchStatus == 1)
		{
			return switchStatus;
		}
	}
	return switchStatus;
}

void powerHold(void)
{
		HAL_GPIO_WritePin(PWR_HOLD_GPIO_Port, PWR_HOLD_Pin, GPIO_PIN_RESET);
	inputConfigPowerHold();
	int PwrStatus = sensePowerHold();
	if(PwrStatus == 1)
	{
		outputConfigPowerHold();
		enablePowerHold();
	}
	
}
