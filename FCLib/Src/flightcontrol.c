/*
 * flightcontrol.c
 *
 *  Created on: 2 Dec 2022
 *      Author: TTM
 *
 */


#include "math.h"
#include <stdio.h>
#include "main.h"

#include "flightcontrol.h"
#include "data_structs.h"
#include "USFSMAX.h"
#include "tof.h"
#include "mmc5983ma.h"
#include "tests.h"
#include "led.h"
#include "peripherals.h"
#include "gps.h"
#include "motor.h"
#include "receiver.h"

#define RC_CH1 ppm_values[1]
#define RC_CH2 ppm_values[2]
#define RC_CH3 ppm_values[3]
#define RC_CH4 ppm_values[4]
#define RC_CH5 ppm_values[5]
#define RC_CH6 ppm_values[6]
#define RC_CH7 ppm_values[7]
#define RC_CH8 ppm_values[8]

#define MID_VAL 1511.0
#define MIN_VAL 994
#define NORM_DENOM 515
#define PI 3.14
#define TWO_PI 6.28
#define SERVO_OFFSET 10

extern uint32_t ppm_values[9];
uint16_t servo_value = 1500;

//threshold variables for landing check
#define GYRO__THRESHOLD 1

int tof_data_flag = 0;
int mag_flag = 0;
ImuData imu;
MagData mag;

double x_coeff = 1/96.489;
double y_coeff = 94.489/96.489;
double prev_y = 0.0;
double curr_y = 0.0;
double prev_angle = 0.0;
double angular_vel = 0.0;

uint32_t prev_time = 0;

//flags
uint8_t in_flight = 0;
uint8_t is_landed = 0;

//uint8_t FC_CheckLandedStatus() {
//	if (imu.g.z < GYRO_THRESOLD && imu.g.x < GYRO_THRESHOLD && imu.g.y < GYRO_THRESHOLD && in_flight) {
//		return 1;
//	}
//
//	return 0;
//}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_9) // If The INT Source Is EXTI Line9 (A9 Pin)
    {
    	tof_data_flag = 1;
    	TOF_GetMeasurement();
    }

    if(GPIO_Pin == GPIO_PIN_10) // If The INT Source Is EXTI Line10 (B10 Pin)
	{
    	mag_flag = 1;
    	MMC5983_IntHandler(&mag);
	}
}

void FC_Setup() {
	HAL_Delay(1000);
	StartReceiver();
	InitServo(1);
	HAL_Delay(1);
	MMC5983_Enable();
	MMC5983_WriteRegister(IC1, 0x80);
	HAL_Delay(10);
	MMC5983_CheckID();
	MMC5983_WriteRegister(IC1, 0x02);
	MMC5983_WriteRegister(IC0, 0x08);
	MMC5983_WriteRegister(IC0, 0x10);
	MMC5983_WriteRegister(IC0, 0x04);
	MMC5983_WriteRegister(IC2, 0b00001101);
	HAL_Delay(1);

	while(1) {
		if (mag_flag == 1) {
			MMC5983_PrintData(&mag);
			mag_flag = 0;
		}
	}

}


int FC_Loop() {
	prev_time = HAL_GetTick();
	//MMC5983_WriteRegister(IC0, 0x01);
	while (1) {
		//MMC5983_WriteRegister(IC0, 0x01);

		check_status: ;

		uint8_t stat = MMC5983_ReadRegister(STATUS);
		if (stat & 0x01) {
			MMC5983_ReadMagData(&mag);
			MMC5983_WriteRegister(STATUS, 0x00);
			MMC5983_PrintData(&mag);
			//MMC5983_WriteRegister(IC0, 0x01);

//			if (RC_CH6 > 1200) {
//				LED_EnableLed(LED_RED_HANDLE);
//				MMC5983_MagCalibrate(&mag);
//			}
//
//			else {
//				LED_DisableLed(LED_RED_HANDLE);
//				//MMC5983_MagCorrection(&mag);
//				MMC5983_GetAngle(&mag);
//				if (!isnan(mag.angle)) {
//					double filtered_val = filter(mag.angle);
//					printf("%f\n",  angular_vel);
//
//					FC_PassiveController();
//					FC_UpdateFlightStatus();
//					if (is_landed) {
//						LED_EnableLed(LED_BLUE_HANDLE);
//						return 0;
//					}
//				}

			}

		else {
			goto check_status;
		}

	}
}

void FC_UpdateFlightStatus() {
	if (in_flight && angular_vel < 0.1) {
		is_landed = 1;
		in_flight = 0;
	}

	if (!in_flight && angular_vel > 31.4 && angular_vel < 50) {
		in_flight = 1;
	}

}

double filter(double angle) {
	curr_y = x_coeff * (prev_angle + angle) + y_coeff*prev_y;
	uint32_t curr_time = HAL_GetTick();

	//crossing from pi to -pi will cause huge angular velocity values, therefore only update it if (angle-prev_angle) is below a threshold
	if (fabs(angle - prev_angle) < 3.14) {
		angular_vel = (angle - prev_angle)/((double)(curr_time - prev_time)/1000.0);  //dont need to worry about tick overflow since it's less than 49 days
	}

	prev_angle = angle;
	prev_time = curr_time;
	prev_y = curr_y;
	return curr_y;
}

int GetSineSign(double value) {
	if (value < 0) {
		return -GetSineSign(-value);
	}

	if (value > 0 && value < PI) {
		return 1;
	}

	else {
		return -1;
	}
}

void FC_PassiveController() {
	double cont_pitch = (RC_CH2 - MID_VAL)/NORM_DENOM;
	double cont_roll = (RC_CH1 - MID_VAL)/NORM_DENOM;
	double amp = sqrt(pow(cont_pitch, 2) + pow(cont_roll, 2));
	double dir = atan2(cont_pitch, cont_roll);
	double scale = (RC_CH5 - MIN_VAL)/5;
	double yaw_offset = (RC_CH2 - MID_VAL)/NORM_DENOM * PI;
	double servo_val;

	if (amp > 1) {
		amp = 1;
	}

	if (RC_CH6 < 1200) {
		int sign = GetSineSign(mag.angle + dir + yaw_offset);
		servo_val = RC_CH3 + sign*amp*scale;

		//printf("%u \n", (uint16_t)servo_val);
		SetServoMicroseconds(1, (uint16_t) servo_val);
	}

	else {
		SetServoMicroseconds(1,1500);
	}
}


