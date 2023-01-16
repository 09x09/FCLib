/*
 * tests.c
 *
 *  Created on: Jan 3, 2023
 *      Author: TTM
 */

#include <stdio.h>
//#include "fatfs.h"
#include "math.h"

#include "data_structs.h"
#include "USFSMAX.h"
#include "tof.h"
#include "mmc5983ma.h"
#include "tests.h"
#include "led.h"
#include "peripherals.h"
#include "gps.h"
#include "motor.h"
#include "receiver.h"
//#include "sd.h"

ImuData imu;
MagData mag;
int tof_data_flag = 0;
int mag_flag = 0;

void Test_StartTests() {
	//Test_USFSMAX();
	//Test_TOF();
	HAL_Delay(1000);
	LED_EnableLed(LED_BLUE_HANDLE);
	HAL_Delay(1000);
	LED_DisableLed(LED_BLUE_HANDLE);
	HAL_Delay(1000);
	LED_EnableLed(LED_RED_HANDLE);
	HAL_Delay(1000);
	LED_DisableLed(LED_RED_HANDLE);

	Test_GPS();
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_9) // If The INT Source Is EXTI Line9 (A9 Pin)
    {
    	tof_data_flag = 1;
    	TOF_GetMeasurement();
    }

    if(GPIO_Pin == GPIO_PIN_10) // If The INT Source Is EXTI Line10 (B10 Pin)
	{
    	mag_flag = 1;
    	MMC5983_IntHandler(&mag);
	}
}

void Test_TOF() {
	printf("Testing TOF...\n");
	TOF_Enable();
	HAL_Delay(100);
	TOF_Init();
	HAL_Delay(100);

	printf("Please move the board around to check TOF output\n");

	while (1) {

	}

	TOF_Disable();
	printf("End Test TOF...\n");

}


void Test_USFSMAX() {
	printf("Testing USFSMAX...\n");
	USFSMAX_Enable();
	HAL_Delay(1000);
	USFSMAX_Init();
	HAL_Delay(100);

	printf("Please move the board around to check USFSMAX output\n");

	for (uint8_t i = 0; i< 100; i++) {
		uint8_t stat = USFSMAX_GetComboDRdy();
		USFSMAX_FetchData(&imu, stat);
		PrintDataIMU(&imu);
		HAL_Delay(50);
	}

	HAL_Delay(100);
	USFSMAX_Disable();
	printf("Finish testing USFSMAX...\n");
}

void Test_MMC5983() {
	printf("Testing MMC5983MA in interrupt mode...\n");
	MMC5983_Enable();
	HAL_Delay(100);
	MMC5983_CheckID();
	MMC5983_Configure();
	HAL_Delay(1);
	printf("Please move the board around to check MMC5983MA output\n");


	while (1) {
		if (mag_flag == 1) {
			MMC5983_PrintData(&mag);
		}
		mag_flag = 0;
	}

	printf("Ending idle time \n");
	MMC5983_Disable();
	HAL_Delay(100);
	printf("End test MMC5983MA \n");

}

void Test_MMC5983_2() {
	MMC5983_Enable();
	HAL_Delay(1);
	MMC5983_Manual(&mag);

}

void Test_LED() {
	printf("Testing LED..\n");
	HAL_Delay(2000);
	LED_EnableLed(LED_GREEN_HANDLE);
	HAL_Delay(2000);
	LED_DisableLed(LED_GREEN_HANDLE);
	HAL_Delay(500);

	LED_EnableLed(LED_BLUE_HANDLE);
	HAL_Delay(2000);
	LED_DisableLed(LED_BLUE_HANDLE);
	HAL_Delay(500);

	LED_EnableLed(LED_RED_HANDLE);
	HAL_Delay(2000);
	LED_DisableLed(LED_RED_HANDLE);
	HAL_Delay(500);
	printf("End test LED\n");
}


void Test_Servo() {
	printf("Testing servos 1 in 5s, ensure servo 1 is conected\n");
	InitServo(1);
	HAL_Delay(1000);
	SetServoMicroseconds(1, 1000);
	HAL_Delay(1000);
	SetServoMicroseconds(1, 2000);
	HAL_Delay(1000);
	SetServoMicroseconds(1, 1000);
	HAL_Delay(500);
	SetServoMicroseconds(2, 0);
	HAL_Delay(1000);

	printf("Servo 1 done. Starting servo 2 test in 5s, ensure servo 2 is connected");
	HAL_Delay(5000);
	InitServo(2);
	HAL_Delay(1000);
	SetServoMicroseconds(2, 1000);
	HAL_Delay(1000);
	SetServoMicroseconds(2, 2000);
	HAL_Delay(1000);
	SetServoMicroseconds(2, 1000);
	HAL_Delay(500);
	SetServoMicroseconds(2, 0);
	HAL_Delay(1000);

	printf("Ending servo test...\n\n\n");
}

void Test_Motor() {
	printf("Testing servos 1 in 5s, ensure motor 1 is conected\n");
	HAL_Delay(5000);
	InitMotor(1);
	HAL_Delay(1000);
	SetMotorMicroseconds(1, 1000);
	HAL_Delay(1000);
	SetMotorMicroseconds(1, 1300);
	HAL_Delay(1000);
	SetMotorMicroseconds(1, 0);
	HAL_Delay(1000);

	printf("Servo 1 done. Starting servo 2 test in 5s, ensure motor 2 is connected");
	HAL_Delay(5000);
	InitMotor(2);
	HAL_Delay(1000);
	SetMotorMicroseconds(2, 1000);
	HAL_Delay(1000);
	SetMotorMicroseconds(2, 1300);
	HAL_Delay(1000);
	SetMotorMicroseconds(2, 0);
	HAL_Delay(1000);

	printf("Ending motor test...\n\n\n");
}

//void Test_SD() {
//	printf("Testing micro sd...\n");
//	FRESULT sd_status = SD_Init();
//	char header[] = "Time, Lat, Long, Altitude, Num Sat\n";
//	if (sd_status == FR_OK) {
//	  SD_Write(&header[0], sizeof(header));
//	}
//}

void Test_Receiver() {
	printf("Starting receiver.. rc values will appear shortly \n");
	StartReceiver();

	uint32_t start = HAL_GetTick();
	uint32_t curr = HAL_GetTick();
	while (curr - start < 3000) {
		curr = HAL_GetTick();
	}

	printf("Ending receiver test..\n\n\n");
}

/*
 * GPS connection required, move to outdoor space
 * Red LED will light up when a connection is made
 */
void Test_GPS() {
	extern GPSData data;

	HAL_Delay(1000);

	GPS_EnableGPS();

	GPS_GetNMEAmsg();

	uint8_t gps_status = GPS_CheckGPSStatus(&data);
	while(1) {
		if (gps_status == GPS_OK) {
			LED_EnableLed(LED_BLUE_HANDLE);
		}

		else {
			LED_DisableLed(LED_BLUE_HANDLE);
		}

		GPS_PrintMsg();
	}
}
