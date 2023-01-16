/*
 * gps.h
 *
 *  Created on: Jul 21, 2022
 *      Author: Guest0
 */

#ifndef INC_GPS_H_
#define INC_GPS_H_

#include "data_structs.h"

#define GPS_OK 0
#define GPS_NO_CONNECTION 1
#define GPS_ERROR 2

void GPS_EnableGPS();
void GPS_DisableGPS();
void GPS_GetNMEAmsg();
void GPS_ConfigGPS();
void GPS_PrintMsg();
void GPS_ParseMsg (GPSData* d);
uint8_t GPS_CheckGPSStatus();
void GPS_ParseGGA(char*s, GPSData* d);
void GPS_ParseVTG(char*s, GPSData* d);

#endif /* INC_GPS_H_ */
