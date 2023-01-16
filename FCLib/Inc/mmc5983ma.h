/*
 * mmc5983ma.h
 *
 *  Created on: Nov 28, 2022
 *      Author: TTM
 */

#ifndef INC_MMC5983MA_H_
#define INC_MMC5983MA_H_

#include "data_structs.h"

#define XOUT0 0x00
#define XOUT1 0x01
#define YOUT0 0x02
#define YOUT1 0x03
#define ZOUT0 0x04
#define ZOUT1 0x05
#define XYZOUT2 0x06
#define TOUT 0x07
#define STATUS 0x08
#define IC0 0x09
#define IC1 0x0A
#define IC2 0x0B
#define IC3 0x0C
#define PROD_ID 0x2f


uint8_t MMC5983_ReadRegister(uint8_t addr);
void MMC5983_CheckID();
void MMC5983_Configure();
void MMC5983_ReadMagData(MagData* m);
void MMC5983_PrintData(MagData* m);
void MMC5983_IntHandler(MagData* m);
void MMC5983_Enable();
void MMC5983_Disable();
void MMC5983_GetAngle(MagData *m);
void MMC5983_MagCalibrate(MagData* destination);
void MMC5983_MagCorrection(MagData* destination) ;

void MMC5983_Manual(MagData* m);
void MMC5983_WriteRegister(uint8_t addr, uint8_t value);

#endif /* INC_MMC5983MA_H_ */
