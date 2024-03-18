/*
 * ssd1306.c
 *
 *  Created on: Mar 18, 2024
 *      Author: zaccko
 */

#include "stm32f1xx.h"

// Simple video RAM
static uint8_t ssd1306_vram[SSD1306_ROWS/8][SSD1306_COLUMNS] = {0};


void delay(void) {
	for (uint32_t i = 0; i < 250000 / 2; i++)
		;
}


//SPI2 (doesnt need AFIO remap)
//PB15 SPI2_MOSI
//pb14 SPI2_MISO
//pb13 SPI2_SCK
//pb14 SPI2_NSS

void SPI_GPIOINITS() {
	// Initialize GPIO pins for SPI (outputmode pupd, fast speed)
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;

	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinDirection = GPIO_DIR_OUT_FAST;

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//MISO not used
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	//GPIO_Init(&SPIPins);

	//SCK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);

}

void SPI2_Init() {
	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_BUSConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER
	;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV128;
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_HIGH;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_HIGH;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DI;
	SPI2Handle.SPIConfig.SPI_LSBFIRST = SPI_MSBFIRST_MODE;

	SPI_Init(&SPI2Handle);
}

/*
 * Initialize the SSD1306 display module
 *
 */
void ssd1306_init(void) {
	GPIO_Handle_t ScreenReset, ScreenDCToggle;
	GPIO_PCLK_CTRL(GPIOB, ENABLE);
	//Handle GPIOS for Screen
	//Reset
	ScreenReset.pGPIOx = GPIOB;
	ScreenReset.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	ScreenReset.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT_PP;
	ScreenReset.GPIO_PinConfig.GPIO_PinDirection = GPIO_DIR_OUT_MEDIUM;
	GPIO_Init(&ScreenReset);

	//Data/Command Toggle
	ScreenDCToggle.pGPIOx = GPIOB;
	ScreenDCToggle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	ScreenDCToggle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT_PP;
	ScreenDCToggle.GPIO_PinConfig.GPIO_PinDirection = GPIO_DIR_OUT_MEDIUM;
	GPIO_Init(&ScreenDCToggle);

	//2.Init SPI2
	SPI_GPIOINITS();
	SPI2_Init();
	SPI_SSOEConfig(SPI2, ENABLE);
	SPI_PeripheralControl(SPI2, ENABLE);

	// Reset Screen
	GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_7, SET);
	delay();
	GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_7, RESET);
	delay();
	GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_7, SET);
	delay();

	//3.Init Commands
	//Enter COmmand mode
	GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_8, RESET);
	delay();

	uint8_t initInstructions[] = {
	SSD1306_SETDISPLAY_OFF, // display off
			SSD1306_SETDISPLAYCLOCKDIV, 0x80, // set clock command and param
			SSD1306_SETMULTIPLEX, (SSD1306_ROWS - 1), // number of display rows in multiplexer
			SSD1306_VERTICALOFFSET, 0, //no vertical display offset
			SSD1306_SETSTARTLINE | 0x00, // RAM start line 0
			SSD1306_SETCHARGEPUMP, 0x14, // charge pump on
			SSD1306_SETADDRESSMODE, 0x00, // horizontal addressmode
			SSD1306_COLSCAN_DESCENDING, // flip columns
			SSD1306_COMSCAN_ASCENDING, //dont flip pages
			SSD1306_SETCOMPINS, 0x02, // set comm pins sequential mode
			SSD1306_SETCONTRAST, 0x00, //set minimal contrast
			SSD1306_SETPRECHARGE, 0xF1, //set precharge period
			SSD1306_SETVCOMLEVEL, 0x40, // set vcom deselect level
			SSD1306_ENTIREDISPLAY_OFF, //use RAM contents for display
			SSD1306_SETINVERT_OFF, //set inversion off
			SSD1306_SCROLL_DEACTIVATE, //deactivate scroll
			SSD1306_SETDISPLAY_ON, //set display on
			};
	for (int i = 0; i < sizeof(initInstructions) / sizeof(initInstructions[0]);
			i++) {
		SPI_SendData(SPI2, &initInstructions[i], 1);
	}
}


/*
 * ! Draw a single pixel to the display
 * !
 * ! \param x: x coordinate of pixel to write to [0:SSD1306_COLUMNS-1]
 * ! \param y: y coordinate of pixel to write to [0:SSD1306_ROWS-1]
 * ! \param value: value to write to the pixel [0:1]
 * !
 * ! \return 0 if successful, error code if failed:
 * !         1: x value out of range
 * !         2: y value out of range
 */
uint16_t ssd1306_drawPixel(uint16_t x, uint16_t y, uint8_t value) {
	// ensure pixel location is valid
	if (x >= SSD1306_COLUMNS)
		return 1;
	if (y >= SSD1306_ROWS)
		return 2;

	const uint8_t page = y >> 3;
	uint8_t configMsg[] = {
			SSD1306_SETPAGERANGE, page, page, SSD1306_SETCOLRANGE, x, x };

	for (int n = 0; n < sizeof(configMsg) / sizeof(configMsg[0]); n++) {
		SPI_SendData(SPI2, &configMsg[n], 1);
	}

	//write to vram
	if(value) ssd1306_vram[page][x] |= 0x01 << (y & 0x07);
	else ssd1306_vram[page][x] &= ~(0x01 << (y & 0x07));

	//Enter Data mode
	GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_8, SET);
	delay();
	uint8_t dataMsg[] = { ssd1306_vram[page][x]};

	//for (int n = 0; n < sizeof(dataMsg) / sizeof(dataMsg[0]); n++) {
	SPI_SendData(SPI2, dataMsg, sizeof(dataMsg));
	//}

	return 0;
}


