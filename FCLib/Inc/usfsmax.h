/*
 * usfsmax.h
 *
 *  Created on: Nov 29, 2022
 *      Author: TTM
 */

#ifndef INC_USFSMAX_H_
#define INC_USFSMAX_H_

uint8_t USFSMAX_GetSensErrStatus();
uint8_t USFSMAX_GetFusionStatus();
uint8_t USFSMAX_GetCalibStatus();
uint8_t USFSMAX_GetComboDrDy();
void USFSMAX_GetGyroADC(GyroData* g);
void USFSMAX_GetAccADC(AccelData* a) ;
void USFSMAX_GetMagADC(MagData* m);
void USFSMAX_GetQuat(QuatData* q);
float USFSMAX_Uint32ToFloat(uint8_t* buf);


#endif /* INC_USFSMAX_H_ */
