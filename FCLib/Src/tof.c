/*
 * tof.c
 *
 *  Created on: Jul 7, 2022
 *      Author: TTM
 */


#include "tof.h"
#include "vl53l1_api.h"
#include "main.h"
#include "peripherals.h"


extern I2C_HandleTypeDef TOF_HANDLE;
VL53L1_Dev_t dev = {.i2c_slave_address = 0x52, .I2cHandle = &TOF_HANDLE};
VL53L1_DEV Dev = &dev;


VL53L1_RangingMeasurementData_t range_data;

void TOF_Enable() {
	HAL_GPIO_WritePin(GPIOH, GPIO_PIN_3, GPIO_PIN_SET);
	HAL_Delay(5); //delay for I2C
}

void TOF_Disable() {
	HAL_GPIO_WritePin(GPIOH, GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_Delay(1);
}




void TOF_Init () {
	VL53L1_Error status;
	status = VL53L1_DataInit(Dev);
	if (status != VL53L1_ERROR_NONE) {
		printf("Data init status: %d", status);
	}

	status  = VL53L1_StaticInit(Dev);
	if (status != VL53L1_ERROR_NONE) {
		printf("Static init status: %d", status);
	}

	status = VL53L1_SetPresetMode(Dev, VL53L1_PRESETMODE_RANGING);
	if (status != VL53L1_ERROR_NONE) {
		printf("Static init status: %d", status);
	}

	VL53L1_StartMeasurement(Dev);
}


void TOF_GetMeasurement() {
	VL53L1_GetRangingMeasurementData(Dev, &range_data);
	VL53L1_ClearInterruptAndStartMeasurement(Dev);
	printf("Distance is: %d mm\n", range_data.RangeMilliMeter);
}

