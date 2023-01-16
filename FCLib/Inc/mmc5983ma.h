/*
 * mmc5983ma.h
 *
 *  Created on: Nov 28, 2022
 *      Author: TTM
 */

#ifndef INC_MMC5983MA_H_
#define INC_MMC5983MA_H_

#include "data_structs.h"

void MMC5983_CheckID();
void MMC5983_Configure();
void MMC5983_ReadMagData(MagData* m);
void MMC5983_PrintData(MagData* m);
void MMC5983_IntHandler(MagData* m);


#endif /* INC_MMC5983MA_H_ */
