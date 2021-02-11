#include "main.h"
#include <stm32f4xx_hal.h>
#include <stm32_hal_legacy.h>

const int headerSize = 44;
//******************************************************//	
//*****************VARIABLES****************************//
//******************************************************//
static uint16_t InternalBuffer[INTERNAL_BUFF_SIZE];
static uint16_t PCM_buff_sd[WAV_SIZE / 2];
static uint16_t InternalBuffer1[INTERNAL_BUFF_SIZE];
uint8_t recording_flag;
uint32_t count;
uint16_t ba_count;
FRESULT f;

void SysTick_Handler(void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}

void wavHeader(char* header, int wavSize)
{
	header[0] = 'R';
	header[1] = 'I';
	header[2] = 'F';
	header[3] = 'F';
	unsigned int fileSize = wavSize + headerSize - 8;
	header[4] = (char)(fileSize & 0xFF);
	header[5] = (char)((fileSize >> 8) & 0xFF);
	header[6] = (char)((fileSize >> 16) & 0xFF);
	header[7] = (char)((fileSize >> 24) & 0xFF);
	header[8] = 'W';
	header[9] = 'A';
	header[10] = 'V';
	header[11] = 'E';
	header[12] = 'f';
	header[13] = 'm';
	header[14] = 't';
	header[15] = ' ';
	header[16] = 0x10;
	header[17] = 0x00;
	header[18] = 0x00;
	header[19] = 0x00;
	header[20] = 0x01;
	header[21] = 0x00;
	header[22] = 0x01;
	header[23] = 0x00;
	header[24] = 0x80;
	header[25] = 0x3E;
	header[26] = 0x00;
	header[27] = 0x00;
	header[28] = 0x00;
	header[29] = 0x7D;
	header[30] = 0x00;
	header[31] = 0x00;
	header[32] = 0x02;
	header[33] = 0x00;
	header[34] = 0x10;
	header[35] = 0x00;
	header[36] = 'd';
	header[37] = 'a';
	header[38] = 't';
	header[39] = 'a';
	header[40] = (char)(wavSize & 0xFF);
	header[41] = (char)((wavSize >> 8) & 0xFF);
	header[42] = (char)((wavSize >> 16) & 0xFF);
	header[43] = (char)((wavSize >> 24) & 0xFF);
}


void DMA1_Stream3_IRQHandler(void)
{
	static uint16_t PCM_buff[16]; 							// tmp buffer for PCM data
	uint16_t* write_buf; 									//pointer for RAW data which must be filtered
	if((DMA1->LISR)&DMA_LISR_TCIF3)
	{
		DMA1->LIFCR = DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTEIF3 | DMA_LIFCR_CDMEIF3 | DMA_LIFCR_CFEIF3;
		if ((DMA1_Stream3->CR & DMA_SxCR_CT) == 0)			//get number of current buffer
			{
				write_buf = (uint16_t*)InternalBuffer;
			}
		else
		{
			write_buf = (uint16_t*)InternalBuffer1;
		}	
		if (recording_flag != 1)
		{
			PDM_Filter(write_buf, PCM_buff, &Filter_H);
			for (int i = 0; i < 16; i++)
			{
				if (ba_count < 1024)
				{
					ba_count++;
				}
				else
				{
					PCM_buff_sd[count] = PCM_buff[i];
					count++;
				}
				
			}
		}
		
		if (count == WAV_SIZE / 2)
		{
			count = 0;
			recording_flag = 1;
		}

	}
}




int main(void)
{
	HAL_Init();
	RCC_Init();
	SPI_Init();
	pdm2pcm_init();
	I2S_MICRO_init();
	recording_flag = 0;
	count = 0;
	ba_count = 0;
	I2S_DMA_Init();

	for (;;)
	{
		if (recording_flag)
		{
			recording_flag = 0;
			
			
			NVIC_DisableIRQ(DMA1_Stream3_IRQn);
			retUSER = FATFS_LinkDriver(&USER_Driver, USERPath);
			disk_initialize(SDFatFs.drv);
			char header[44];
			wavHeader(header, WAV_SIZE);
			// запись шапки wav
			if(f = f_mount(&SDFatFs, (TCHAR const*)USERPath, 0) != FR_OK)
			{
				Error_Handler();
			}
			else
			{
				if (f = f_open(&MyFile, "221.wav", FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
				{
					Error_Handler();
				}
				else
				{
					f = f_write(&MyFile, header, headerSize, (void*)&byteswritten);
					if ((byteswritten == 0) || (f != FR_OK))
					{
						Error_Handler();
					}
					for (int i = 0; i <= WAV_SIZE; i += 512)
					{
						f = f_write(&MyFile, (uint8_t *)&PCM_buff_sd[i/2], 512, (void*)&byteswritten);
						if ((byteswritten == 0) || (f != FR_OK))
						{
							Error_Handler();
						}
					}					
					f_close(&MyFile);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
				}
			}
		}

	}
}
//*****************ERROR*****************************//
void Error_Handler(void)
{
	NVIC_SystemReset();
}
//*****************SPI_INIT*****************************//
void SPI_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	/**SPI1 GPIO Configuration
	PA5     ------> SPI1_SCK
	PA6     ------> SPI1_MISO
	PA7     ------> SPI1_MOSI
	*/
	GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_5 | GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	//	GPIO_InitStruct.Pin = GPIO_PIN_4;
	//	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	//	GPIO_InitStruct.Pull = GPIO_NOPULL;
	//	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	//	GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
	//	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		spi_flash_gpio.Pin = GPIO_PIN_4;
	spi_flash_gpio.Mode = GPIO_MODE_OUTPUT_PP;
	spi_flash_gpio.Pull = GPIO_NOPULL;
	spi_flash_gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOA, &spi_flash_gpio);

	spi_sd_gpio.Pin = GPIO_PIN_3;
	spi_sd_gpio.Mode = GPIO_MODE_OUTPUT_PP;
	spi_sd_gpio.Pull = GPIO_NOPULL;
	spi_sd_gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOA, &spi_sd_gpio);

	__HAL_RCC_SPI1_CLK_ENABLE();

	spi1.Instance = SPI1;
	spi1.Init.Mode = SPI_MODE_MASTER;
	spi1.Init.Direction = SPI_DIRECTION_2LINES;
	spi1.Init.DataSize = SPI_DATASIZE_8BIT;
	spi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	spi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	spi1.Init.NSS = SPI_NSS_SOFT;
	spi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
	spi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	spi1.Init.TIMode = SPI_TIMODE_DISABLE;
	spi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	spi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&spi1) != HAL_OK)
	{
		Error_Handler();
	}
	//	__HAL_SI_ENABLE(&spi1);
	//	SPI1->CR2 |= SPI_CR2_RXDMAEN; 				// DMA interrupt enable
	//	SPI1->CR2 |= SPI_CR2_TXDMAEN;  				// DMA interrupt enable
	//	DMA_SD_Init();
}
//*****************RCC_INIT*****************************//
void RCC_Init(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	*/
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the CPU, AHB and APB busses clocks
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 25;
	RCC_OscInitStruct.PLL.PLLN = 192;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
		| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
	{
		Error_Handler();
	}
	
	__HAL_RCC_GPIOC_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
}




void I2S_MICRO_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* Peripheral clock enable */
	__HAL_RCC_SPI2_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	/**I2S2 GPIO Configuration
	PB10     ------> I2S2_CK
	PB15     ------> I2S2_SD
	PB12     ------> I2S2_WS
	*/
	GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_12 | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	RCC->PLLI2SCFGR &= ~RCC_PLLI2SCFGR_PLLI2SM;
	RCC->PLLI2SCFGR &= ~RCC_PLLI2SCFGR_PLLI2SN;
	RCC->PLLI2SCFGR &= ~RCC_PLLI2SCFGR_PLLI2SR;
	RCC->PLLI2SCFGR |= RCC_PLLI2SCFGR_PLLI2SM_0 | RCC_PLLI2SCFGR_PLLI2SM_3 | RCC_PLLI2SCFGR_PLLI2SM_4;           // M = 25
	RCC->PLLI2SCFGR |= RCC_PLLI2SCFGR_PLLI2SN_6 | RCC_PLLI2SCFGR_PLLI2SN_7;       // N = 192
	RCC->PLLI2SCFGR |= RCC_PLLI2SCFGR_PLLI2SR_1;          // R = 2
	//I2S_CLK =  MHz
	RCC->CR |= RCC_CR_PLLI2SON;
	while (!(RCC->CR & RCC_CR_PLLI2SRDY)) ;
	
	SPI2->I2SCFGR |= SPI_I2SCFGR_I2SMOD; 		// i2s mod selected
	SPI2->I2SCFGR |= SPI_I2SCFGR_I2SCFG; 		// master-receive
	SPI2->I2SCFGR |= SPI_I2SCFGR_I2SSTD_0; 		// LSB philips standart
	SPI2->I2SCFGR &= ~SPI_I2SCFGR_CKPOL; 		// I2S clock steady state is low level
	SPI2->I2SCFGR &= ~SPI_I2SCFGR_DATLEN; 		// 16-bit data length
	SPI2->I2SCFGR &= ~SPI_I2SCFGR_CHLEN; 		// 16-bit wide
	SPI2->I2SPR &= ~SPI_I2SPR_I2SDIV;
	SPI2->I2SPR &= ~SPI_I2SPR_ODD; 				// ODD = 0
	SPI2->I2SPR |= 38;

	SPI2->CR2 |= SPI_CR2_RXDMAEN; 				// DMA interrupt enable

	//SPI2->I2SPR |= SPI_I2SPR_MCKOE;				// clock output enable

	SPI2->I2SCFGR |= SPI_I2SCFGR_I2SE; 			// I2S enable
}

void I2S_DMA_Init(void)
{
	// DMA1 stream4 channel 0(spi2 RX)
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;   // enable DMA1

	DMA1_Stream3->PAR = (uint32_t)&(SPI2->DR);
	DMA1_Stream3->M0AR = (uint32_t)InternalBuffer;
	DMA1_Stream3->M1AR = (uint32_t)InternalBuffer1;
	DMA1_Stream3->NDTR = INTERNAL_BUFF_SIZE;
	//DMA1_Stream4->CR |= DMA_SxCR_CHSEL_0 | DMA_SxCR_CHSEL_1;   
	DMA1_Stream3->CR &= ~DMA_SxCR_CHSEL;      // channel 0 selected
	DMA1_Stream3->CR |= DMA_SxCR_DBM;
	DMA1_Stream3->CR |= DMA_SxCR_PL_1;
	DMA1_Stream3->CR |= DMA_SxCR_MSIZE_0;
	DMA1_Stream3->CR |= DMA_SxCR_PSIZE_0;
	DMA1_Stream3->CR |= DMA_SxCR_MINC;
	DMA1_Stream3->CR &= ~DMA_SxCR_DIR;
	DMA1_Stream3->CR |= DMA_SxCR_TCIE;
	DMA1_Stream3->CR |= DMA_SxCR_CIRC;
	DMA1_Stream3->CR |= DMA_SxCR_EN;

	NVIC_EnableIRQ(DMA1_Stream3_IRQn);
	NVIC_SetPriority(DMA1_Stream3_IRQn, 6);
}

void pdm2pcm_init(void)
{
	__HAL_RCC_CRC_CLK_ENABLE();
	CRC->CR = CRC_CR_RESET;
	
	Filter_H.high_pass_tap = 1932735282;
	//Filter_H.high_pass_tap = 2122358088;
	Filter_H.in_ptr_channels = 1;
	Filter_H.out_ptr_channels = 1;
	Filter_H.endianness = PDM_FILTER_ENDIANNESS_BE;
	Filter_H.bit_order = PDM_FILTER_BIT_ORDER_LSB;
	PDM_Filter_Init(&Filter_H);
	
	Filter_C.decimation_factor = PDM_FILTER_DEC_FACTOR_80;
	Filter_C.mic_gain = 30;
	Filter_C.output_samples_number = 16;
	
	PDM_Filter_setConfig((PDM_Filter_Handler_t *)&Filter_H, &Filter_C);
}


DWORD get_fattime(void)
{
	
	return 0;
	
}