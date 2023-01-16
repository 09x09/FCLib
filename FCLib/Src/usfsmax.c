/*
 * usfsmax.c
 *
 *  Created on: 29 Nov 2022
 *      Author: TTM
 */

#include "../Inc/data_structs.h"
#include "main.h"
#include <stdio.h>
#include "../Inc/usfsmax.h"

#define USFSMAX_HANDLE hi2c3

#define MAX32660_SLV_ADDR 0x57
#define SENS_ERR_STAT 0x00
#define CALIB_STATUS 0x01
#define FUSION_STATUS 0x03
#define COMBO_DRDY_STAT 0x04
#define G_X_L 0x05
#define A_X_L 0x0b
#define M_X_L 0x11
#define BARO_XL 0x17
#define Q0_BYTE0 0x1a
#define LIN_X_L 0x2a
#define GRAV_X_L 0x30
#define YAW_BYTE0 0x36

extern I2C_HandleTypeDef USFSMAX_HANDLE;


void USFSMAX_Init() {
	uint8_t status = USFSMAX_GetFusionStatus();

}

uint8_t USFSMAX_GetSensErrStatus() {
	uint8_t bytes[1];
	HAL_I2C_Mem_Read(&USFSMAX_HANDLE, MAX32660_SLV_ADDR << 1,SENS_ERR_STAT, 1, bytes, 1, 1 );
	return bytes[0];
}

uint8_t USFSMAX_GetFusionStatus() {
	uint8_t bytes[1];
	HAL_I2C_Mem_Read(&USFSMAX_HANDLE, MAX32660_SLV_ADDR << 1,FUSION_STATUS, 1, bytes, 1, 1 );
	return bytes[0];
}


uint8_t USFSMAX_GetCalibStatus() {
	uint8_t bytes[1];
	HAL_I2C_Mem_Read(&USFSMAX_HANDLE, MAX32660_SLV_ADDR << 1,CALIB_STATUS, 1, bytes, 1, 1 );
	return bytes[0];
}

uint8_t USFSMAX_GetComboDrDy() {
	uint8_t bytes[1];
	HAL_I2C_Mem_Read(&USFSMAX_HANDLE, MAX32660_SLV_ADDR << 1,COMBO_DRDY_STAT, 1, bytes, 1, 1 );
	return bytes[0];
}

void USFSMAX_GetGyroADC(GyroData* g) {
	uint8_t bytes[6];
	HAL_I2C_Mem_Read(&USFSMAX_HANDLE, MAX32660_SLV_ADDR << 1, G_X_L, 1, bytes, 6, 1 );
	g->x = ((int16_t) bytes[1] << 8) | bytes[0];
	g->y = ((int16_t) bytes[3] << 8) | bytes[2];
	g->z = ((int16_t) bytes[5] << 8) | bytes[4];
}


void USFSMAX_GetAccADC(AccelData* a) {
	uint8_t bytes[6];
	HAL_I2C_Mem_Read(&USFSMAX_HANDLE, MAX32660_SLV_ADDR << 1, A_X_L, 1, bytes, 6, 1 );
	a->x = ((int16_t) bytes[1] << 8) | bytes[0];
	a->y = ((int16_t) bytes[3] << 8) | bytes[2];
	a->z = ((int16_t) bytes[5] << 8) | bytes[4];
}


void USFSMAX_GetMagADC(MagData* m) {
	uint8_t bytes[6];
	HAL_I2C_Mem_Read(&USFSMAX_HANDLE, MAX32660_SLV_ADDR << 1, M_X_L, 1, bytes, 6, 1 );
	m->x = ((int16_t) bytes[1] << 8) | bytes[0];
	m->y = ((int16_t) bytes[3] << 8) | bytes[2];
	m->z = ((int16_t) bytes[5] << 8) | bytes[4];
}

void USFSMAX_GetQuat(QuatData* q) {
	uint8_t bytes[16];
	HAL_I2C_Mem_Read(&USFSMAX_HANDLE, MAX32660_SLV_ADDR << 1, Q0_BYTE0, 1, bytes, 16, 1 );
	q->w = USFSMAX_Uint32ToFloat(&bytes[0]);
	q->x = USFSMAX_Uint32ToFloat(&bytes[4]);
	q->y = USFSMAX_Uint32ToFloat(&bytes[8]);
	q->z = USFSMAX_Uint32ToFloat(&bytes[12]);
}


float USFSMAX_Uint32ToFloat(uint8_t* buf) {
	union {
		uint32_t ui32;
		float f;
	} u;

	u.ui32 = ((uint32_t) buf[0]) + ((uint32_t) buf[1] << 8) + ((uint32_t) buf[2] << 16) + ((uint32_t) buf[3] << 24);
	return u.f;
}
