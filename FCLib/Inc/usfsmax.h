/*
 * usfsmax.h
 *
 *  Created on: Nov 29, 2022
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

#ifndef INC_USFSMAX_H_
#define INC_USFSMAX_H_

typedef struct USFSMAX_config {

	 uint16_t cal_points;
	  uint8_t  a_scale;
	  uint8_t  a_odr;
	  uint8_t  a_lpf;
	  uint8_t  a_hpf;
	  uint8_t  g_scale;
	  uint8_t  g_odr;
	  uint8_t  g_lpf;
	  uint8_t  g_hpf;
	  uint8_t  m_scale;
	  uint8_t  m_odr;
	  uint8_t  m_lpf;
	  uint8_t  m_hpf;
	  uint8_t  p_scale;
	  uint8_t  p_odr;
	  uint8_t  p_lpf;
	  uint8_t  p_hpf;
	  uint8_t  aux1_scale;
	  uint8_t  aux1_odr;
	  uint8_t  aux1_lpf;
	  uint8_t  aux1_hpf;
	  uint8_t  aux2_scale;
	  uint8_t  aux2_odr;
	  uint8_t  aux2_lpf;
	  uint8_t  aux2_hpf;
	  uint8_t  aux3_scale;
	  uint8_t  aux3_odr;
	  uint8_t  aux3_lpf;
	  uint8_t  aux3_hpf;
	  float    m_v;
	  float    m_h;
	  float    m_dec;
	  uint8_t  quat_div;
	  float kp_def;

} USFSMAX_config;

void USFSMAX_Enable();
void USFSMAX_Disable();
void USFSMAX_Init();
uint8_t USFSMAX_GetSensErrStatus();
uint8_t USFSMAX_GetFusionStatus();
uint8_t USFSMAX_GetCalibStatus();
uint8_t USFSMAX_GetComboDRdy();

void USFSMAX_GetGyroADC(GyroData* g);
void USFSMAX_GetAccelADC(AccelData* a);
void USFSMAX_GetGyroAccelADC(ImuData* imu);
void USFSMAX_GetMagADC(MagData* m);
void USFSMAX_GetMagBaroADC(MagData* m, BaroData* b);
void USFSMAX_GetQuat(QuatData* q);


float USFSMAX_Uint32ToFloat(uint8_t* buf);
uint8_t USFSMAX_GetFirmwareID();
void USFSMAX_UploadConfig();
void USFSMAX_StopFusion();
void USFSMAX_StartFusion();
void USFSMAX_FetchData(ImuData* imu, uint8_t drdy_status);
void PrintDataIMU(ImuData* imu);


#endif /* INC_USFSMAX_H_ */
