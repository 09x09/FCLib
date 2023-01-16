/*
 * tof.h
 *
 *  Created on: Jul 1, 2022
 *      Author: Guest0
 */

#ifndef INC_TOF_H_
#define INC_TOF_H_

/*
 * Enables the load switch for both PNI and TOF
 */
void TOF_Enable();

/*
 * Disables the load switch for PNI and TOF
 */
void TOF_Disable();

/*
 * Calls the bare driver functions VL53L1_DataInit(), VL53L1_StaticInit(), VL53L1_SetPresetMode()
 * and VL53L1_StartMeasurement()
 */
void TOF_Init ();

/*
 * Calls the bare driver functions VL53L1_GetRangingMeasurementData() and VL53L1_ClearInterruptAndStartMeasurement()
 * This function is called in HAL_GPIO_EXTI_Callback
 */
void TOF_GetMeasurement();

#endif /* INC_TOF_H_ */
