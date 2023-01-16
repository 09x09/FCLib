/*
 * led.c
 *
 *  Created on: 2 Dec 2022
 *      Author: TTM
 *
 *  Only for STM32F4 eval board, remove for H7 board
 */

#include "main.h"
#include "../Inc/led.h"
#include "peripherals.h"

/*
 * Input parameters:
 * 		- LED_BLUE
 * 		- LED_RED
 * 		- LED_GREEN
 *
 */

void LED_ToggleLed(uint16_t led) {
	HAL_GPIO_TogglePin(GPIOJ, GPIO_PIN_3);
}

void LED_EnableLed(uint16_t led) {
	HAL_GPIO_WritePin(GPIOJ, led, GPIO_PIN_RESET);
}

void LED_DisableLed(uint16_t led) {
	HAL_GPIO_WritePin(GPIOJ, led, GPIO_PIN_SET);
}



