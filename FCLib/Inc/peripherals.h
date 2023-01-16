/*
 * peripherals.h
 *
 *  Created on: Dec 8, 2022
 *      Author: TTM
 */

#ifndef FCLIB_INC_PERIPHERALS_H_
#define FCLIB_INC_PERIPHERALS_H_


//GPS
#define GPS_HANDLE huart8
#define GPS_DMA_HANDLE hdma_uart8_rx

//MMC5983
#define MMC5983_HANDLE hspi6
#define MMC5983_NSS_PIN GPIO_PIN_8
#define MMC5983_NSS_PORT GPIOG

//USFSMAX
#define USFSMAX_HANDLE hi2c2

//LED
#define LED_BLUE_HANDLE GPIO_PIN_13
#define LED_RED_HANDLE GPIO_PIN_14
#define LED_GREEN_HANDLE GPIO_PIN_15

//RC
#define RC_TIMER TIM5
#define RC_TIM_HANDLE htim5
#define RC_TIM_CHANNEL TIM_CHANNEL_4

//SERVO
#define SERVO_TIM_HANDLE htim8
#define SERVO_TIMER TIM8

//motor
#define MOTOR_TIM_HANDLE htim8
#define MOTOR_TIMER TIM8

//TOF
#define TOF_HANDLE hi2c3

#endif /* FCLIB_INC_PERIPHERALS_H_ */
