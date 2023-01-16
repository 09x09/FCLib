/*
 * motor.h
 *
 *  Created on: Jul 5, 2022
 *      Author: Guest0
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

/*
 * Initialises the timer channels for the selected servo
 * Servo 1: TIM8 CH1
 * Servo 2: TIM8 CH2
 */
void InitServo();

/*
 * Initialises the timer channels for the selected motor
 * Motor 1: TIM1 CH1
 * Motor 2: TIM1 CH4
 */
void InitMotor();

/*
 * Sets the CCR register for the selected servo
 */
void SetServoMicroseconds(uint16_t value);

/*
 * Sets the CCR register for the selected motor
 */
void SetMotorMicroseconds(uint16_t value);

/*
 * Sets servo pwm output value to RC Ch1
 */
void ServoTest();

/*
 * Sets ESC pwm value to RC Ch1
 */
void MotorTest();

#endif /* INC_MOTOR_H_ */
