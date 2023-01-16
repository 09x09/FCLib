/*
 * receiver.c
 *
 *  Created on: Jul 15, 2022
 *      Author: Guest0
 */


#include "main.h"
#include <stdio.h>
#include "peripherals.h"


extern TIM_HandleTypeDef RC_TIM_HANDLE;

uint8_t counter = 0;
uint32_t prev_value;
volatile uint32_t ppm_values[9];

void StartReceiver() {
	HAL_TIM_Base_Start_IT(&RC_TIM_HANDLE);
	HAL_TIM_IC_Start_IT(&RC_TIM_HANDLE, RC_TIM_CHANNEL);
	HAL_Delay(1);
}

void PrintPPM() {
	for (int i = 0; i < 9; i++) {
		printf("Ch %d: %lu ", i, ppm_values[i]);
	}
	printf("\n");
}

void ProcessPPM () {
	__HAL_TIM_CLEAR_IT(&RC_TIM_HANDLE, TIM_IT_CC4);
	__HAL_TIM_CLEAR_FLAG(&RC_TIM_HANDLE, TIM_FLAG_CC4);

	uint32_t captured_value = HAL_TIM_ReadCapturedValue(&RC_TIM_HANDLE, RC_TIM_CHANNEL);
	uint32_t difference = (captured_value - prev_value) % RC_TIMER->ARR;
	prev_value = captured_value;

	if (difference > 4000) {
		counter = 0;

	}

	ppm_values[counter] = difference;
	if (counter == 8) {
		//PrintPPM();
	}
	counter = (counter + 1) % 9;

}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
	{
		ProcessPPM();
	}
}

