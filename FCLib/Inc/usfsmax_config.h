/*
 * usfsmax_config.h
 *
 *  Created on: 1 Dec 2022
 *      Author: TTM
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

#ifndef FCLIB_INC_USFSMAX_CONFIG_H_
#define FCLIB_INC_USFSMAX_CONFIG_H_

#define CAL_POINTS 2048
#define ACC_ODR 0x07 //834Hz
#define GYRO_ODR 0x07 //834Hz
#define MAG_ODR 0x05 //100H
#define BARO_ODR 0x04 //50Hz
#define AUX1_ODR 0x00
#define AUX2_ODR 0x00
#define AUX3_ODR 0x00
#define QUAT_DIV 0x07

#define ACC_SCALE 0x01 //16g
#define GYRO_SCALE 0x01 //4000 dps
#define MAG_SCALE 0x00
#define BARO_SCALE 0x00
#define AUX1_SCALE 0x00
#define AUX2_SCALE 0x00
#define AUX3_SCALE 0x00

#define LSM6DSR_GYRO_DLPF_CFG 0x03
#define LSM6DSR_GYRO_DHPF_CFG 0x00
#define LSM6DSR_ACC_DLPF_CFG 0x07
#define LSM6DSR_ACC_DHPF_CFG 0x00
#define MMC5983MA_MAG_LPF 0x00
#define MMC5983MA_MAG_HPF 0x00
#define LPS22HB_BARO_LPF 0x0C
#define LPS22HB_BARO_HPF 0x00
#define AUX1_LPF 0x00
#define AUX1_HPF 0x00
#define AUX2_LPF 0x00
#define AUX2_HPF 0x00
#define AUX3_LPF 0x00
#define AUX3_HPF 0x00

#define KP_DEF_USFSMAX 0.5f

//mag constants for SUTD
#define M_V -9616.2f
#define M_H 41054.1f
#define MAG_DECLINATION 0.0809f

//other settings
#define ENABLE_DHI_CORRECTOR 0x01
#define USE_2D_DHI_CORRECTOR 0x00
#define OUTPUT_EULER_ANGLES  0x00 //0x01 - euler, 0x00 - quaternion
#define SCALED_SENSOR_DATA 0x01 //scaled

//scaling settings
#define DPS_PER_COUNT 0.140f
#define MMC5983MA_UT_PER_COUNT 0.006103515625f
#define G_PER_COUNT 0.0004880f

#endif /* FCLIB_INC_USFSMAX_CONFIG_H_ */
