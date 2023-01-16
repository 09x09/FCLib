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
void InitServo(uint8_t servo);

/*
 * Initialises the timer channels for the selected motor
 * Motor 1: TIM1 CH1
 * Motor 2: TIM1 CH4
 */
void InitMotor(uint8_t motor);

/*
 * Sets the CCR register for the selected servo
 */
void SetServoMicroseconds(uint8_t servo, uint16_t value);

/*
 * Sets the CCR register for the selected motor
 */
void SetMotorMicroseconds(uint8_t motor, uint16_t value);


#endif /* INC_MOTOR_H_ */
