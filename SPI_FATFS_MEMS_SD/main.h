#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

//******************************************************//
//**********************INCLUDS*************************//
//******************************************************//
#include "ff.h"
#include "sd.h"
#include "ff_gen_drv.h"
#include "user_diskio.h" /* defines USER_Driver as external */
#include "w25qxx.h"
#include "w25qxxConf.h"
#include "pdm2pcm_glo.h"
//*******************************************************//
//***********************DEFINES*************************//
//*******************************************************//
#define WAV_SIZE 96000
#define I2S_SAMPLE_RATE (16000) //Частота дискритизации 16кГЦ
#define I2S_SAMPLE_BITS (16) //Биты канала i2c 16-бит
#define RECORD_TIME (10) //Время записи
#define I2S_CHANNEL_NUM (1)
#define FLASH_RECORD_SIZE (I2S_CHANNEL_NUM * I2S_SAMPLE_RATE * I2S_SAMPLE_BITS / 8 * RECORD_TIME) //Память, которую занимает запись
#define PCM_buff_size 128;
#define	INTERNAL_BUFF_SIZE 80
#define NUM_READ 16
#define	DEC_FACTOR 80
//******************************************************//		
//**********************FUNCTIONS***********************//
//******************************************************//
void Error_Handler(void);		// 
void RCC_Init(void);			//initialization reset and clock controller
void SPI_Init(void);			//initialization spi bus for sd card and flash
void I2S_MICRO_init(void);		//initialisation spi and i2s buses for pdm microphone
void I2S_DMA_Init(void);		//initialisatoin dma controller for i2s bus
void pdm2pcm_init(void);		//initialisation library of pdm filter
	
//******************************************************//
//**********************TYPEDEFS************************//
//******************************************************//
GPIO_InitTypeDef spi_sd_gpio;
GPIO_InitTypeDef spi_flash_gpio;
SPI_HandleTypeDef spi1;
Diskio_drvTypeDef  USER_Driver;
static PDM_Filter_Handler_t Filter_H;
static PDM_Filter_Config_t Filter_C;
//******************************************************//	
//*********************VARIABLES************************//
//******************************************************//
uint8_t retUSER;    /* Return value for USER */
char USERPath[4];   /* USER logical drive path */
FATFS USERFatFS;    /* File system object for USER logical drive */
FIL USERFile;       /* File object for USER */
uint32_t byteswritten, bytesread;
uint8_t result;
FATFS SDFatFs;
FATFS *fs;
FIL MyFile;


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
