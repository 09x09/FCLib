/*
 * GPS.c
 *
 *  Created on: Jul 21, 2022
 *      Author: Guest0
 */

#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "gps.h"

extern UART_HandleTypeDef huart8;
extern DMA_HandleTypeDef hdma_uart8_rx;

#define MAX_SENTENCE_LENGTH 82
#define SENTENCE_COUNT 5
#define MAX_NMEA_LENGTH MAX_SENTENCE_LENGTH*SENTENCE_COUNT

uint8_t callback_done_flag = 0;

uint8_t gps_buffer[MAX_NMEA_LENGTH];
char raw_msg[MAX_NMEA_LENGTH];

GPSData data;

void EnableGPS() {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	HAL_Delay(1);
}

void DisableGPS() {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_Delay(1);
}

#define GPS_OK 0;
#define GPS_NO_CONNECTION 1;
#define GPS_ERROR 2;

uint8_t CheckGPSStatus (GPSData* g) {
	if (g->num_satellite == 0) {
		return GPS_NO_CONNECTION;
	}

	if (g->longitude == 0 && g->latitude == 0) {
		return GPS_ERROR;
	}

	return GPS_OK;
}

void GetNMEAmsg() {
	HAL_UARTEx_ReceiveToIdle_DMA(&huart8, gps_buffer, MAX_NMEA_LENGTH);
	__HAL_DMA_DISABLE_IT(&hdma_uart8_rx, DMA_IT_HT);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	memcpy(raw_msg, gps_buffer, Size);
	callback_done_flag = 1;
	HAL_UARTEx_ReceiveToIdle_DMA(&huart8, gps_buffer, MAX_NMEA_LENGTH);
	__HAL_DMA_DISABLE_IT(&hdma_uart8_rx, DMA_IT_HT);
}


void PrintMsg() {
	while (!callback_done_flag) {

	}

	ParseMsg(&data);

	printf("Time: %f, Lat: %f%c, Long: %f%c, Alt: %f, Speed %f, Mag head: %f, True head: %f, num sat: %u, HDOP: %f \n", \
			data.timestamp, data.latitude, data.ns[0], data.longitude, data.ew[0], data.msl_altitude, data.speed_kmh, data.mag_heading, data.true_heading, data.num_satellite, data.hdop);
	callback_done_flag = 0;
}


const char sentence_end[2] = "\n";
const char gga[4] = "GGA";
const char vtg[4] = "VTG";

#define FIND_AND_NUL(s, p, c) ( \
   (p) = strchr(s, c), \
   *(p) = '\0', \
   ++(p), \
   (p))


void ParseMsg (GPSData* d) {
	char *curr_ptr;
	curr_ptr = strtok(raw_msg, sentence_end);
	while (curr_ptr != NULL){
		if (strstr(curr_ptr, gga) != NULL) {
			printf("%s ", curr_ptr);
			ParseGGA(curr_ptr, d);
			goto next_sentence;
		}

		if (strstr(curr_ptr, vtg) != NULL) {
			printf("%s\n", curr_ptr);
			ParseVTG(curr_ptr,d);
			goto next_sentence;
		}

		next_sentence:
		curr_ptr = strtok(NULL, sentence_end);
	}

	callback_done_flag = 0;

}


void ParseGGA (char* s, GPSData* g) {
	char gga_msg[100];
	strcpy(gga_msg, s);

	char* msg_id = gga_msg;
	char* msg_time = FIND_AND_NUL(msg_id, msg_time, ',');
	char* latitude =  FIND_AND_NUL(msg_time, latitude, ',');
	char* ns = FIND_AND_NUL(latitude, ns, ',');
	char* longitude = FIND_AND_NUL(ns, longitude, ',');
	char* ew =  FIND_AND_NUL(longitude, ew, ',');
	char* fix_indicator =  FIND_AND_NUL(ew, fix_indicator, ',');
	char* num_satellite =  FIND_AND_NUL(fix_indicator, num_satellite, ',');
	char* hdop =  FIND_AND_NUL(num_satellite, hdop, ',');
	char* msl_altitude =   FIND_AND_NUL(hdop, msl_altitude, ',');
	char* units1 =  FIND_AND_NUL(msl_altitude, units1,  ',');
	char* geoid_sep =  FIND_AND_NUL(units1, geoid_sep, ',');
	char* units2 =  FIND_AND_NUL(geoid_sep, units2, ',');
	char* age_diff_correction =  FIND_AND_NUL(units2, age_diff_correction, ',');
	char* diff_station_id =  FIND_AND_NUL(age_diff_correction, diff_station_id, ',');
	char* checksum =  FIND_AND_NUL(checksum, diff_station_id, ',');

	double num_lat = atof(latitude);
	double num_long = atof(longitude);

    //stuff struct with values

	g->timestamp = atof(msg_time);
	g->latitude = floor(num_lat/100) + (num_lat-floor(num_lat/100)*100)/60;
	g->longitude = floor(num_long/100) + (num_long-floor(num_long/100)*100)/60;
	g->msl_altitude = atof(msl_altitude);
	g->geoid_sep = atof(geoid_sep);
	g->hdop = atof(hdop);

	g->fix_indicator = (uint8_t) atoi(fix_indicator);
	g->num_satellite = (uint8_t) atoi(num_satellite);

	strcpy(g->ew, ew);
	strcpy(g->ns, ns);


}


void ParseVTG(char* s, GPSData* g) {
	char vtg_msg[100];
	strcpy(vtg_msg, s);

	char* msg_id = vtg_msg;
	char* true_heading = FIND_AND_NUL(msg_id, true_heading, ',');
	char* ref1 = FIND_AND_NUL(true_heading, ref1, ',');
	char* mag_heading = FIND_AND_NUL(msg_id, mag_heading, ',');
	char* ref2 = FIND_AND_NUL(mag_heading, ref2, ',');
	char* speed_knots = FIND_AND_NUL(ref2, speed_knots, ',');
	char* unit1 = FIND_AND_NUL(speed_knots, unit1, ',');
	char* speed_kmh = FIND_AND_NUL(unit1, speed_kmh, ',');
	char* unit2 = FIND_AND_NUL(speed_kmh, unit2, ',');
	char* mode = FIND_AND_NUL(unit2, mode, ',');
	char* checksum = FIND_AND_NUL(mode, checksum, ',');

	g->true_heading = atof(true_heading);
	g->mag_heading = atof(mag_heading);
	g->speed_knots = atof(speed_knots);
	g->speed_kmh = atof(speed_kmh);

}
