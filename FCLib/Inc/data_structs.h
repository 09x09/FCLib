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
	double angle;
	double fx;
	double fy;
	double fz;

} MagData;


typedef struct QuatData {

	double w;
	double x;
	double y;
	double z;

} QuatData;

typedef struct EulerData {

	double x;
	double y;
	double z;

} EulerData;

typedef struct BaroData {

	int32_t val;

} BaroData;

typedef struct ImuData {

	AccelData a;
	GyroData g;
	MagData m;
	QuatData q;
	BaroData b;
	EulerData e;

} ImuData;


typedef struct GPSData {
	double timestamp;
	double latitude;
	double longitude;
	double msl_altitude;
	double geoid_sep;
	double hdop;

	double true_heading;
	double mag_heading;
	double speed_knots;
	double speed_kmh;

	char ew[2];
	char ns[2];

	uint8_t num_satellite;
	uint8_t fix_indicator;


} GPSData;

#endif /* INC_DATA_STRUCTS_H_ */
