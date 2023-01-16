/*
 * MMC5983MA.c
 *
 *  Created on: Nov 28, 2022
 *      Author: TTM
 */

#include "../Inc/data_structs.h"
#include "main.h"
#include <stdio.h>
#include "math.h"
#include "../Inc/mmc5983ma.h"

#define MMC5983_HANDLE hi2c2

#define I2C_ADDR 0b0110000
#define XOUT0 0x00
#define XOUT1 0x01
#define YOUT0 0x02
#define YOUT1 0x03
#define ZOUT0 0x04
#define ZOUT1 0x05
#define XYZOUT2 0x06
#define TOUT 0x07
#define STATUS 0x08
#define IC0 0x09
#define IC1 0x0A
#define IC2 0x0B
#define IC3 0x0C
#define PROD_ID 0x2f

extern I2C_HandleTypeDef MMC5983_HANDLE;

int16_t mag_bias[3];
int16_t mag_max[3];
int16_t mag_min[3];
int16_t mag_scale[3];
float mag_fscale[3];


void MMC5983_ClearStatusInterrupt();

void MMC5983_CheckID() {
	uint8_t id[1] = {};
	HAL_I2C_Mem_Read(&MMC5983_HANDLE, I2C_ADDR << 1, PROD_ID, 1, id, 1, 1 );
	printf("Product ID: %u \n", id[0]);
}

void MMC5983_Configure() {
	uint8_t  settings[3];
	uint8_t ic0 = 0b00000100;
	uint8_t ic1 = 0x00;
	uint8_t ic2 = 0b00001101;

	settings[0] = ic0;
	settings[1] = ic1;
	settings[2] = ic2;

	HAL_I2C_Mem_Write(&MMC5983_HANDLE, I2C_ADDR << 1, IC0, 1, settings, 3, 1 );

}

void MMC5983_ClearStatusInterrupt() {
	uint8_t clear_interrupt[1] = {0x03};
	HAL_I2C_Mem_Write(&MMC5983_HANDLE, I2C_ADDR << 1, STATUS, 1, clear_interrupt, 1, 1 );
}

void MMC5983_IntHandler(MagData* m) {
	uint8_t status[1];
	HAL_I2C_Mem_Read(&MMC5983_HANDLE, I2C_ADDR << 1, STATUS, 1, status, 1, 1 );
	if (status[0] & 0x01) {
		MMC5983_ReadMagData(m);
	}

	MMC5983_ClearStatusInterrupt();

}

void MMC5983_ReadMagData(MagData* m) {
	uint8_t data[6] = {};
	HAL_I2C_Mem_Read(&MMC5983_HANDLE, I2C_ADDR << 1, XOUT0, 1, data, 6, 1 );

	m->x = data[0] + ((uint16_t)data[1] << 8);
	m->y = data[2] + ((uint16_t)data[3] << 8);
	m->z = data[4] + ((uint16_t)data[5] << 8);
}

void MMC5983_PrintData(MagData* m) {
	printf("X value: %u, Y value: %u, Z value: %u \n", m->x, m->y, m->z);
}


void MagCalibrate(MagData* destination) {
	  int16_t temp[3] = {destination->x, destination->y, destination->z};

	  for (int i = 0; i < 3; i++) {
		  if (temp[i] > mag_max[i]) {
			  mag_max[i] = temp[i];
		  }

		  else if (temp[i] < mag_min[i]) {
			  mag_min[i] = temp[i];
		  }
	  }

	  // Get hard iron correction
	  mag_bias[0]  = (mag_max[0] + mag_min[0]) / 2; // get average x mag bias in counts
	  mag_bias[1]  = (mag_max[1] + mag_min[1]) / 2; // get average y mag bias in counts
	  mag_bias[2]  = (mag_max[2] + mag_min[2]) / 2; // get average z mag bias in counts
	  //printf("Bias X: %d, Bias Y: %d, Bias Z: %d \n", mag_bias[0], mag_bias[1], mag_bias[2]);

	  // Get soft iron correction estimate
	  mag_scale[0]  = (mag_max[0] - mag_min[0]) / 2; // get average x axis max chord length in counts
	  mag_scale[1]  = (mag_max[1] - mag_min[1]) / 2; // get average y axis max chord length in counts
	  mag_scale[2]  = (mag_max[2] - mag_min[2]) / 2; // get average z axis max chord length in counts
	  //printf("Scale X: %d, Scale Y: %d, Scale Z: %d \n", mag_scale[0], mag_scale[1], mag_scale[2]);

	  mag_fscale[0] = (mag_scale[0] + mag_scale[1] + mag_scale[2])/(3.0 * mag_scale[0]);
	  mag_fscale[1] = (mag_scale[0] + mag_scale[1] + mag_scale[2])/(3.0 * mag_scale[1]);
	  mag_fscale[2] = (mag_scale[0] + mag_scale[1] + mag_scale[2])/(3.0 * mag_scale[2]);
	  //printf("Fscale X: %f, Fscale Y: %f, Fscale Z: %f \n", mag_fscale[0], mag_fscale[1], mag_fscale[2]);
}

void MagCorrection(MagData* destination) {
	destination->fx = (destination->x - mag_bias[0]) * mag_fscale[0];
	destination->fy = (destination->y - mag_bias[1]) * mag_fscale[1];
	destination->fz = (destination->z - mag_bias[2]) * mag_fscale[2];
}

