/*
 * sd.c
 *
 *  Created on: Dec 2, 2022
 *      Author: TTM
 */

#include "main.h"
#include "fatfs.h"
#include <stdio.h>
#include <string.h>

#define WRITE_LEN_MAX 512

char fname[20];
BYTE write_buffer[WRITE_LEN_MAX];
FIL fil;
FATFS FatFs;
FRESULT fres;

void SD_Enable() {
	HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_7, GPIO_PIN_SET);
	HAL_Delay(1);
}

void SD_Disable() {
	HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_7, GPIO_PIN_RESET);
	HAL_Delay(1);
}

FRESULT SD_Init() {
	fres = f_mount(&FatFs, "", 1); //1=mount now
	  if (fres != FR_OK) {
		printf("f_mount error (%i)\r\n", fres);
		while(1);
	  }

	  //Let's get some statistics from the SD card
	  DWORD free_clusters, free_sectors, total_sectors;

	  FATFS* getFreeFs;

	  fres = f_getfree("", &free_clusters, &getFreeFs);
	  if (fres != FR_OK) {
		printf("f_getfree error (%i)\r\n", fres);
		return fres;
	  }

	  //Formula comes from ChaN's documentation
	  total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
	  free_sectors = free_clusters * getFreeFs->csize;

	  printf("SD card stats:\r\n%10lu KiB total drive space.\r\n%10lu KiB available.\r\n", total_sectors / 2, free_sectors / 2);

	  FRESULT fr;
	  FILINFO fno;
	  int filenum = 0;

	  for (filenum = 0; filenum < 1000; filenum ++) {
		  sprintf(fname, "Data%d.csv", filenum);
		  fr = f_stat(fname, &fno);
		  if (fr == FR_OK) {
			  printf("File %d exists, increment number \n", filenum);
		  }

		  if (fr == FR_NO_FILE) {
			  printf("Filename = Data%d.csv\n", filenum);
			  break;
		  }
	  }

	  return FR_OK;
}


void SD_Write(char* txt, uint16_t length) {
	fres = f_open(&fil, fname, FA_OPEN_APPEND | FA_WRITE);

	//Copy in a string
	uint16_t remaining = length;
	while (remaining > WRITE_LEN_MAX ) {
		strncpy((char*)write_buffer, &txt[length-remaining], WRITE_LEN_MAX);
		UINT bytesWrote;
		fres = f_write(&fil, write_buffer, WRITE_LEN_MAX, &bytesWrote);
		remaining -= WRITE_LEN_MAX;
	}

	strncpy((char*)write_buffer, &txt[length-remaining], remaining);
	UINT bytesWrote;
	fres = f_write(&fil, write_buffer, remaining, &bytesWrote);

	//Be a tidy kiwi - don't forget to close your file!
	f_close(&fil);
}
