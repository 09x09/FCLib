/*
 * MMC5983MA.c
 *
 *  Created on: Nov 28, 2022
 *      Author: TTM
 */

#include "../Inc/data_structs.h"
#include "../Inc/peripherals.h"
#include "main.h"
#include <stdio.h>
#include "math.h"
#include "../Inc/mmc5983ma.h"

extern SPI_HandleTypeDef MMC5983_HANDLE;


int32_t mag_bias[3];
int32_t mag_max[3] = {-1,-1,-1};
int32_t mag_min[3] = {-1,-1,-1};
int32_t mag_scale[3];
double mag_fscale[3];


void MMC5983_ClearStatusInterrupt();

void MMC5983_Enable() {
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
	HAL_Delay(1);
}

void MMC5983_Disable() {
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
	HAL_Delay(1);
}


uint8_t MMC5983_ReadRegister(uint8_t addr) {
	uint8_t rr_rx[2]; //receive buffer
	uint8_t rr_tx[2]; //transmit buffer
	rr_tx[0] = (1 << 7) | addr; //sets r/w bit

	HAL_GPIO_WritePin(MMC5983_NSS_PORT, MMC5983_NSS_PIN, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&MMC5983_HANDLE, rr_tx, rr_rx, 2, 1);
	HAL_GPIO_WritePin(MMC5983_NSS_PORT, MMC5983_NSS_PIN, GPIO_PIN_SET);

	return rr_rx[1];
}

void MMC5983_WriteRegister(uint8_t addr, uint8_t value) {
	uint8_t write[2] = {addr, value};

	HAL_GPIO_WritePin(MMC5983_NSS_PORT, MMC5983_NSS_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&MMC5983_HANDLE, write, 2, 1);
	HAL_GPIO_WritePin(MMC5983_NSS_PORT, MMC5983_NSS_PIN, GPIO_PIN_SET);

}

void MMC5983_CheckID() {
	uint8_t id = MMC5983_ReadRegister(PROD_ID);
	printf("Product ID: %u, it should be 48 \n", id);
}

void MMC5983_Configure() {
	uint8_t ic0 = 0b00000101;
	uint8_t ic1 = 0x00;
	uint8_t ic2 = 0b0000101;

	uint8_t ic2_settings[2] = {IC2, ic2};
	uint8_t ic1_settings[2] = {IC1, ic1};
	uint8_t ic0_settings[2] = {IC0, ic0};

	HAL_StatusTypeDef status;

	HAL_GPIO_WritePin(MMC5983_NSS_PORT, MMC5983_NSS_PIN, GPIO_PIN_RESET);
	status = HAL_SPI_Transmit(&MMC5983_HANDLE, ic2_settings, 2, 1);
	HAL_GPIO_WritePin(MMC5983_NSS_PORT, MMC5983_NSS_PIN, GPIO_PIN_SET);
	HAL_Delay(1);

	printf("status ic2: %u \n", status);

	HAL_GPIO_WritePin(MMC5983_NSS_PORT, MMC5983_NSS_PIN, GPIO_PIN_RESET);
	status = HAL_SPI_Transmit(&MMC5983_HANDLE, ic1_settings, 2, 1);
	HAL_GPIO_WritePin(MMC5983_NSS_PORT, MMC5983_NSS_PIN, GPIO_PIN_SET);
	HAL_Delay(1);

	printf("status ic1: %u \n", status);

	HAL_GPIO_WritePin(MMC5983_NSS_PORT, MMC5983_NSS_PIN, GPIO_PIN_RESET);
	status =HAL_SPI_Transmit(&MMC5983_HANDLE, ic0_settings, 2, 1);
	HAL_GPIO_WritePin(MMC5983_NSS_PORT, MMC5983_NSS_PIN, GPIO_PIN_SET);
	HAL_Delay(1);

	printf("status ic0: %u \n", status);
}

void MMC5983_Manual(MagData* m) {
	MMC5983_WriteRegister(IC1, 0x80);
	HAL_Delay(1);
	MMC5983_CheckID();
	MMC5983_WriteRegister(IC0, 0x08);
	MMC5983_WriteRegister(IC0, 0x10);
	HAL_Delay(1);
	//MMC5983_WriteRegister(IC0, 0x0d);
	MMC5983_WriteRegister(IC0, 0x01);
	while(1) {
		uint8_t stat = MMC5983_ReadRegister(STATUS);
		if (stat & 0x01) {
			MMC5983_ReadMagData(m);
			//MMC5983_PrintData(m);
			MMC5983_MagCalibrate(m);
			MMC5983_WriteRegister(STATUS, 0x00);
			MMC5983_WriteRegister(IC0, 0x01);
			MMC5983_MagCorrection(m);
			m->angle = atan2(m->fy, m->fx) * 360/6.28;
			//printf("Angle: %f\n",  m->angle);
			HAL_Delay(5);
		}

	}

}

void MMC5983_ClearStatusInterrupt() {
	uint8_t clear_interrupt[2] = {STATUS,0x03};
	HAL_GPIO_WritePin(MMC5983_NSS_PORT, MMC5983_NSS_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&MMC5983_HANDLE, clear_interrupt, 2, 1);
	HAL_GPIO_WritePin(MMC5983_NSS_PORT, MMC5983_NSS_PIN, GPIO_PIN_SET);
}

void MMC5983_IntHandler(MagData* m) {
	uint8_t status = MMC5983_ReadRegister(STATUS);
	//printf("Status register value: %u \n", status);
	if (status & 0x01) {
		MMC5983_ReadMagData(m);
	}

	MMC5983_ClearStatusInterrupt();

	//start new measurement
//	uint8_t ic0 = 0b00000101;
//	uint8_t ic0_settings[2] = {IC0, ic0};
//	HAL_GPIO_WritePin(MMC5983_NSS_PORT, MMC5983_NSS_PIN, GPIO_PIN_RESET);
//	HAL_SPI_Transmit(&MMC5983_HANDLE, ic0_settings, 2, 1);
//	HAL_GPIO_WritePin(MMC5983_NSS_PORT, MMC5983_NSS_PIN, GPIO_PIN_SET);


}

void MMC5983_ReadMagData(MagData* m) {
	uint8_t tx[7] = {}; //transmit buffer
	uint8_t data[7] = {}; //receive buffer

	tx[0] = (1 << 7) | XOUT0; //set rw bit for spi transmission

	HAL_GPIO_WritePin(MMC5983_NSS_PORT, MMC5983_NSS_PIN, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&MMC5983_HANDLE, tx, data, 7, 1);
	HAL_GPIO_WritePin(MMC5983_NSS_PORT, MMC5983_NSS_PIN, GPIO_PIN_SET);

	m->x = ((uint16_t) data[1] << 8) | data[2];
	m->y = ((uint16_t) data[3] << 8) | data[4];
	m->z = ((uint16_t) data[5] << 8) | data[6];
}

void MMC5983_PrintData(MagData* m) {
	printf("X value: %u, Y value: %u, Z value: %u \n", m->x, m->y, m->z);
}

void MMC5983_GetAngle(MagData *m) {
	if (m->fx == 0 && m->fy == 0) {
		m->angle = atan2(m->y, m->x);
	}

	else {
		m->angle = atan2(m->fy, m->fx);
	}

}


void MMC5983_MagCalibrate(MagData* destination) {
	  int32_t temp[3] = {destination->x, destination->y, destination->z};

	  for (int i = 0; i < 3; i++) {
		  if ((mag_max[i] == -1) || (temp[i] > mag_max[i])) {
			  mag_max[i] = temp[i];
		  }

		  else if ((mag_min[i] == -1) || (temp[i] < mag_min[i])) {
			  mag_min[i] = temp[i];
		  }
	  }

	  // Get hard iron correction
	  mag_bias[0]  = (mag_max[0] + mag_min[0]) / 2; // get average x mag bias in counts
	  mag_bias[1]  = (mag_max[1] + mag_min[1]) / 2; // get average y mag bias in counts
	  mag_bias[2]  = (mag_max[2] + mag_min[2]) / 2; // get average z mag bias in counts
	  //printf("Bias X: %u, Bias Y: %u, Bias Z: %u \n", mag_bias[0], mag_bias[1], mag_bias[2]);

	  // Get soft iron correction estimate
	  mag_scale[0]  = (mag_max[0] - mag_min[0]) / 2; // get average x axis max chord length in counts
	  mag_scale[1]  = (mag_max[1] - mag_min[1]) / 2; // get average y axis max chord length in counts
	  mag_scale[2]  = (mag_max[2] - mag_min[2]) / 2; // get average z axis max chord length in counts
	  //printf("Scale X: %u, Scale Y: %u, Scale Z: %u \n", mag_scale[0], mag_scale[1], mag_scale[2]);

	  mag_fscale[0] = (mag_scale[0] + mag_scale[1] + mag_scale[2])/(3.0 * mag_scale[0]);
	  mag_fscale[1] = (mag_scale[0] + mag_scale[1] + mag_scale[2])/(3.0 * mag_scale[1]);
	  mag_fscale[2] = (mag_scale[0] + mag_scale[1] + mag_scale[2])/(3.0 * mag_scale[2]);
	  //printf("Fscale X: %f, Fscale Y: %f, Fscale Z: %f \n", mag_fscale[0], mag_fscale[1], mag_fscale[2]);
}

void MMC5983_MagCorrection(MagData* destination) {
	destination->fx = (destination->x - mag_bias[0]) * mag_fscale[0];
	destination->fy = (destination->y - mag_bias[1]) * mag_fscale[1];
	destination->fz = (destination->z - mag_bias[2]) * mag_fscale[2];
	destination->angle = atan2(destination->fy, destination->fx);
}

