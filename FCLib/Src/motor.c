/*
 * motor.c
 *
 *  Created on: Jul 5, 2022
 *      Author: Guest0
 */


#include "main.h"
#include "../Inc/motor.h"
#include "../Inc/peripherals.h"

extern TIM_HandleTypeDef SERVO_TIM_HANDLE;
extern TIM_HandleTypeDef MOTOR_TIM_HANDLE;


void InitServo(uint8_t servo) {
	if (servo == 1) {
		HAL_TIM_PWM_Start(&SERVO_TIM_HANDLE, TIM_CHANNEL_1);
	}

	if (servo == 2) {
		HAL_TIM_PWM_Start(&SERVO_TIM_HANDLE, TIM_CHANNEL_2);
	}

}

void InitMotor(uint8_t motor) {
	if (motor == 1) {
		HAL_TIM_PWM_Start(&MOTOR_TIM_HANDLE, TIM_CHANNEL_1);
	}

	if (motor == 2) {
		HAL_TIM_PWM_Start(&MOTOR_TIM_HANDLE, TIM_CHANNEL_4);
	}
}

void SetServoMicroseconds(uint8_t servo, uint16_t value) {
	if (value <= 2000) {
		if (servo == 1) {
			SERVO_TIMER->CCR1 = value;
		}

		if (servo == 2) {
			SERVO_TIMER->CCR2 = value;
		}
	}
}

void SetMotorMicroseconds(uint8_t motor, uint16_t value) {
	if (value <= 2000) {
		if (motor == 1) {
			MOTOR_TIMER->CCR1 = value;
		}

		if (motor == 2) {
			MOTOR_TIMER->CCR4 = value;
		}
	}
}

