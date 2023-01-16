/*
 * sd.h
 *
 *  Created on: 5 Dec 2022
 *      Author: TTM
 */

#ifndef FCLIB_INC_SD_H_
#define FCLIB_INC_SD_H_

FRESULT SD_Init();
void SD_Write(char* txt, uint16_t length);

#endif /* FCLIB_INC_SD_H_ */
