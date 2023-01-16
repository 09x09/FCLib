/*
 * led.h
 *
 *  Created on: 2 Dec 2022
 *      Author: TTM
 *
 *  Only for STM32F4 eval board, remove for H7 board
 */

#ifndef FCLIB_INC_LED_H_
#define FCLIB_INC_LED_H_


void LED_ToggleLed(uint16_t led_num);
void LED_EnableLed(uint16_t led_num);
void LED_DisableLed(uint16_t led_num);
void LED_EnableAll();
void LED_DisableAll();

#endif /* FCLIB_INC_LED_H_ */
