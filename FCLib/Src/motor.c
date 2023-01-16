/*
 * motor.c
 *
 *  Created on: Jul 5, 2022
 *      Author: Guest0
 */


#include "main.h"
#include "../Inc/motor.h"

extern TIM_HandleTypeDef htim1;
//extern TIM_HandleTypeDef htim8;
//extern uint32_t ppm_values[9];

void InitServo() {
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

}

//void InitMotor(uint8_t motor) {
//	if (motor == 1) {
//		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//	}
//
//	if (motor == 2) {
//		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
//	}
//}

void SetServoMicroseconds(uint16_t value) {
	TIM1->CCR3 = value;
}

//void SetMotorMicroseconds(uint8_t motor, uint16_t value) {
//	if (motor == 1) {
//		TIM1->CCR1 = value;
//	}
//
//	if (motor == 2) {
//		TIM1->CCR4 = value;
//	}
//}


//void ServoTest(uint8_t servo) {
//	SetServoMicroseconds(servo, ppm_values[1]);
//}
//
//void MotorTest(uint8_t motor) {
//	SetMotorMicroseconds(motor, ppm_values[1]);
//}
