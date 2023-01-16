/*
 * sensor_structs.h
 *
 *  Created on: 29 Nov 2022
 *      Author: TTM
 */

#ifndef INC_DATA_STRUCTS_H_
#define INC_DATA_STRUCTS_H_

#include "main.h"

typedef struct AccelData {

	int16_t x;
	int16_t y;
	int16_t z;

} AccelData;

typedef struct GyroData {

	int16_t x;
	int16_t y;
	int16_t z;

} GyroData;


typedef struct MagData {

	uint16_t x;
	uint16_t y;
	uint16_t z;
	float angle;
	float fx;
	float fy;
	float fz;

} MagData;


typedef struct QuatData {

	float w;
	float x;
	float y;
	float z;

} QuatData;


#endif /* INC_DATA_STRUCTS_H_ */
