/*
 * usfsmax.c
 *
 *  Created on: 29 Nov 2022
 *      Author: TTM
 *
 * This file contains a port of the USFSMAX library from Gregory Tomasch
 *
 * Copyright (c) 2020 Gregory Tomasch.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal with the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimers.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimers in the
 *     documentation and/or other materials provided with the distribution.
 *  3. The names of Gregory Tomasch and his successors
 *     may not be used to endorse or promote products derived from this Software
 *     without specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * WITH THE SOFTWARE.
 */

#include "main.h"

#include <stdio.h>
#include <string.h>

#include "../Inc/data_structs.h"
#include "../Inc/usfsmax.h"
#include "../Inc/usfsmax_config.h"
#include "../Inc/peripherals.h"

//USFSMAX registers

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
#define FUSION_START_STOP 0x60
#define CALIBRATION_REQUEST 0x61
#define COPRO_CFG_DATA0 0x62
#define COPRO_CFG_DATA1 0x63
#define GYRO_CAL_DATA0 0x64
#define GYRO_CAL_DATA1 0x65
#define ACCEL_CAL_DATA0 0x66
#define ACCEL_CAL_DATA1 0x67
#define ELLIP_MAG_CAL_DATA0 0x68
#define ELLIP_MAG_DATA1 0x69
#define FINE_MAG_CAL_DATA0 0x6a
#define FINE_MAG_CAL_DATA1 0x6b
#define FIRMWARE_ID 0x7f



extern I2C_HandleTypeDef USFSMAX_HANDLE;
extern ImuData imu;


void USFSMAX_Enable() {
	HAL_GPIO_WritePin(GPIOH, GPIO_PIN_3, GPIO_PIN_SET);
	HAL_Delay(5); //delay for I2C
}

void USFSMAX_Disable() {
	HAL_GPIO_WritePin(GPIOH, GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_Delay(1);
}

void USFSMAX_Init() {
	uint8_t id = USFSMAX_GetFirmwareID();
	printf("ID: %u, it should be 4\n", id);

	uint8_t status = USFSMAX_GetFusionStatus();
	if (status == 0) {
		USFSMAX_StopFusion();

		printf("Configuring processor..\n");
		USFSMAX_UploadConfig();
		USFSMAX_StartFusion();

		uint8_t counter;
		//prevent infinite loop
		while (counter < 100) {
			status = USFSMAX_GetFusionStatus();
			if (status & 0x01) {
				break;
			}
			counter += 1;
		}

		if (counter < 100) {
			printf("Fusion running\n");
		}

		else {
			printf("Fusion failed to start \n");
		}
	}

	status = USFSMAX_GetSensErrStatus();
	if (status == 0) {
		printf("No errors \n");
	}

	else {
		printf("Error status: %u\n", status);
	}

	if (ENABLE_DHI_CORRECTOR) {
		uint8_t bytes[1];
		if (USE_2D_DHI_CORRECTOR) {
			bytes[0] = 0x50;
			HAL_I2C_Mem_Read(&USFSMAX_HANDLE, MAX32660_SLV_ADDR << 1,CALIBRATION_REQUEST, 1, bytes, 1, 1 );
		}

		else {
			bytes[0] = 0x10;
			HAL_I2C_Mem_Read(&USFSMAX_HANDLE, MAX32660_SLV_ADDR << 1,CALIBRATION_REQUEST, 1, bytes, 1, 1 );
		}
	}


}

void USFSMAX_UploadConfig() {
	uint8_t config_byte[1] = {0x08};
	HAL_I2C_Mem_Write(&USFSMAX_HANDLE, MAX32660_SLV_ADDR << 1, FUSION_START_STOP, 1, config_byte, 1, 1 );
	HAL_Delay(1000);

	uint8_t cfg_buff[sizeof(USFSMAX_config)];
	USFSMAX_config config = {.cal_points = CAL_POINTS,
							 .a_scale = ACC_SCALE,
							 .a_odr = ACC_ODR,
							 .a_lpf = LSM6DSR_ACC_DLPF_CFG,
							 .a_hpf = LSM6DSR_ACC_DHPF_CFG,
							 .g_scale = GYRO_SCALE,
							 .g_odr = GYRO_ODR,
							 .g_lpf = LSM6DSR_GYRO_DLPF_CFG,
							 .g_hpf = LSM6DSR_GYRO_DHPF_CFG,
							 .m_scale = MAG_SCALE,
							 .m_odr = MAG_ODR,
							 .m_lpf = MMC5983MA_MAG_LPF,
							 .m_hpf = MMC5983MA_MAG_HPF,
							 .p_scale = BARO_SCALE,
							 .p_odr = BARO_ODR,
							 .p_lpf = LPS22HB_BARO_LPF,
							 .p_hpf = LPS22HB_BARO_LPF,
							 .aux1_scale = AUX1_SCALE,
							 .aux1_odr = AUX1_ODR,
							 .aux1_lpf = AUX1_LPF,
							 .aux1_hpf = AUX1_HPF,
							 .aux2_scale = AUX2_SCALE,
							 .aux2_odr = AUX2_ODR,
							 .aux2_lpf = AUX2_LPF,
							 .aux2_hpf = AUX2_HPF,
							 .aux3_scale = AUX3_SCALE,
							 .aux3_odr = AUX3_ODR,
							 .aux3_lpf = AUX3_LPF,
							 .aux3_hpf = AUX3_HPF,
							 .m_v = M_V,
							 .m_h = M_H,
							 .m_dec = MAG_DECLINATION,
							 .quat_div = QUAT_DIV,
							 .kp_def = KP_DEF_USFSMAX
							};

	memcpy(cfg_buff, &config, sizeof(USFSMAX_config));
	HAL_I2C_Mem_Write(&USFSMAX_HANDLE, MAX32660_SLV_ADDR << 1, COPRO_CFG_DATA0, 1, &cfg_buff[0], 30, 10);
	HAL_Delay(100);
	HAL_I2C_Mem_Write(&USFSMAX_HANDLE, MAX32660_SLV_ADDR << 1, COPRO_CFG_DATA1, 1, &cfg_buff[30], sizeof(USFSMAX_config) - 30, 10);
	HAL_Delay(100);
}

uint8_t USFSMAX_GetFirmwareID() {
	uint8_t bytes[1];
	HAL_I2C_Mem_Read(&USFSMAX_HANDLE, MAX32660_SLV_ADDR << 1,FIRMWARE_ID, 1, bytes, 1, 5);
	HAL_Delay(100);
	return bytes[0];
}

uint8_t USFSMAX_GetSensErrStatus() {
	uint8_t bytes[1];
	HAL_I2C_Mem_Read(&USFSMAX_HANDLE, MAX32660_SLV_ADDR << 1,SENS_ERR_STAT, 1, bytes, 1, 5);
	HAL_Delay(100);
	return bytes[0];
}

uint8_t USFSMAX_GetFusionStatus() {
	uint8_t bytes[1];
	HAL_I2C_Mem_Read(&USFSMAX_HANDLE, MAX32660_SLV_ADDR << 1,FUSION_STATUS, 1, bytes, 1, 5);
	HAL_Delay(10);
	return bytes[0];
}


uint8_t USFSMAX_GetCalibStatus() {
	uint8_t bytes[1];
	HAL_I2C_Mem_Read(&USFSMAX_HANDLE, MAX32660_SLV_ADDR << 1,CALIB_STATUS, 1, bytes, 1, 5);
	HAL_Delay(100);
	return bytes[0];
}

uint8_t USFSMAX_GetComboDRdy() {
	uint8_t bytes[1];
	HAL_I2C_Mem_Read(&USFSMAX_HANDLE, MAX32660_SLV_ADDR << 1,COMBO_DRDY_STAT, 1, bytes, 1, 5);
	return bytes[0];
}

void USFSMAX_StopFusion() {
	uint8_t bytes[1] = {0x00};
	HAL_I2C_Mem_Write(&USFSMAX_HANDLE, MAX32660_SLV_ADDR << 1, FUSION_START_STOP, 1, bytes, 1, 5);
	HAL_Delay(100);
}

void USFSMAX_StartFusion() {
	uint8_t bytes[1] = {((0x01 | OUTPUT_EULER_ANGLES << 1) | SCALED_SENSOR_DATA << 2)};
	HAL_I2C_Mem_Write(&USFSMAX_HANDLE, MAX32660_SLV_ADDR << 1, FUSION_START_STOP, 1, bytes, 1, 5);
	HAL_Delay(100);
}

void USFSMAX_GetGyroADC(GyroData* g) {
	uint8_t bytes[6];
	HAL_I2C_Mem_Read(&USFSMAX_HANDLE, MAX32660_SLV_ADDR << 1, G_X_L, 1, bytes, 6, 5);
	g->x = ((int16_t) bytes[1] << 8) | bytes[0];
	g->y = ((int16_t) bytes[3] << 8) | bytes[2];
	g->z = ((int16_t) bytes[5] << 8) | bytes[4];
}


void USFSMAX_GetAccelADC(AccelData* a) {
	uint8_t bytes[6];
	HAL_I2C_Mem_Read(&USFSMAX_HANDLE, MAX32660_SLV_ADDR << 1, A_X_L, 1, bytes, 6, 5);
	a->x = ((int16_t) bytes[1] << 8) | bytes[0];
	a->y = ((int16_t) bytes[3] << 8) | bytes[2];
	a->z = ((int16_t) bytes[5] << 8) | bytes[4];
}

void USFSMAX_GetGyroAccelADC(ImuData* imu) {
	uint8_t bytes[12];
	HAL_I2C_Mem_Read(&USFSMAX_HANDLE, MAX32660_SLV_ADDR << 1, G_X_L, 1, bytes, 12, 5);
	imu->g.x = ((int16_t) bytes[1] << 8) | bytes[0];
	imu->g.y = ((int16_t) bytes[3] << 8) | bytes[2];
	imu->g.z = ((int16_t) bytes[5] << 8) | bytes[4];
	imu->a.x = ((int16_t) bytes[7] << 8) | bytes[6];
	imu->a.y = ((int16_t) bytes[9] << 8) | bytes[8];
	imu->a.z = ((int16_t) bytes[11] << 8) | bytes[10];
}

void USFSMAX_GetMagADC(MagData* m) {
	uint8_t bytes[6];
	HAL_I2C_Mem_Read(&USFSMAX_HANDLE, MAX32660_SLV_ADDR << 1, M_X_L, 1, bytes, 6, 5 );
	m->x = ((int16_t) bytes[1] << 8) | bytes[0];
	m->y = ((int16_t) bytes[3] << 8) | bytes[2];
	m->z = ((int16_t) bytes[5] << 8) | bytes[4];
}

void USFSMAX_GetBaroADC(BaroData* b) {
	uint8_t bytes[3];
	HAL_I2C_Mem_Read(&USFSMAX_HANDLE, MAX32660_SLV_ADDR << 1, BARO_XL, 1, bytes, 3, 5 );
	b->val = ((int32_t) bytes[2] << 16)| ((int32_t) bytes[1] << 8) | bytes[0];
}

void USFSMAX_GetMagBaroADC(MagData* m, BaroData* b){
	uint8_t bytes[9];
	HAL_I2C_Mem_Read(&USFSMAX_HANDLE, MAX32660_SLV_ADDR << 1, M_X_L, 1, bytes, 9, 5 );
	m->x = ((int16_t) bytes[1] << 8) | bytes[0];
	m->y = ((int16_t) bytes[3] << 8) | bytes[2];
	m->z = ((int16_t) bytes[5] << 8) | bytes[4];
	b->val = ((int32_t) bytes[8] << 16)| ((int32_t) bytes[7] << 8) | bytes[6];
}

void USFSMAX_GetQuat(QuatData* q) {
	uint8_t bytes[16];
	HAL_I2C_Mem_Read(&USFSMAX_HANDLE, MAX32660_SLV_ADDR << 1, Q0_BYTE0, 1, bytes, 16, 5 );
	q->w = USFSMAX_Uint32ToFloat(&bytes[0]);
	q->x = USFSMAX_Uint32ToFloat(&bytes[4]);
	q->y = USFSMAX_Uint32ToFloat(&bytes[8]);
	q->z = USFSMAX_Uint32ToFloat(&bytes[12]);
}

void USFSMAX_GetEuler(EulerData* e) {
	uint8_t bytes[12];
	HAL_I2C_Mem_Read(&USFSMAX_HANDLE, MAX32660_SLV_ADDR << 1, YAW_BYTE0, 1, bytes, 12, 5 );
	e->x = USFSMAX_Uint32ToFloat(&bytes[0]);
	e->y = USFSMAX_Uint32ToFloat(&bytes[4]);
	e->z = USFSMAX_Uint32ToFloat(&bytes[8]);
}


void USFSMAX_FetchData(ImuData* imu, uint8_t drdy_status) {
	//TODO: get current time here
	if (drdy_status & 0x01) {
		USFSMAX_GetGyroADC(&imu->g);
	}

	if (drdy_status & 0x02) {
		USFSMAX_GetAccelADC(&imu->a);
	}

	if (drdy_status & 0x04) {
		USFSMAX_GetMagADC(&imu->m);
	}

	if (drdy_status & 0x08) {
		USFSMAX_GetMagBaroADC(&imu->m, &imu->b);
	}

	if (drdy_status & 0x10) {
		if (OUTPUT_EULER_ANGLES) {
			USFSMAX_GetEuler(&imu->e);
		}

		else {
			USFSMAX_GetQuat(&imu->q);
		}
	}
}



float USFSMAX_Uint32ToFloat(uint8_t* buf) {
	union {
		uint32_t ui32;
		float f;
	} u;

	u.ui32 = ((uint32_t) buf[0]) + ((uint32_t) buf[1] << 8) + ((uint32_t) buf[2] << 16) + ((uint32_t) buf[3] << 24);
	return u.f;
}

void PrintDataIMU(ImuData* imu) {
	printf("Accel - x: %d, y: %d, z: %d \n", imu->a.x, imu->a.y, imu->a.z);
	printf("Gyro - x: %d, y: %d, z: %d \n", imu->g.x, imu->g.y, imu->g.z);
	printf("Mag - x: %d, y: %d, z: %d \n", imu->m.x, imu->m.y, imu->m.z);
	printf("Baro - %ld\n", imu->b.val);
	printf("Quat - w: %f, x: %f, y: %f, z: %f \n", imu->q.w, imu->q.x, imu->q.y, imu->q.z);
	printf("\n\n\n");
}
