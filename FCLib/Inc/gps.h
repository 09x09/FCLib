/*
 * gps.h
 *
 *  Created on: Jul 21, 2022
 *      Author: Guest0
 */

#ifndef INC_GPS_H_
#define INC_GPS_H_



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

void EnableGPS();
void DisableGPS();
void GetNMEAmsg();
void ConfigGPS();
void PrintMsg();
void ParseMsg (GPSData* d);
uint8_t CheckGPSStatus();
void ParseGGA(char*s, GPSData* d);
void ParseVTG(char*s, GPSData* d);

#endif /* INC_GPS_H_ */
