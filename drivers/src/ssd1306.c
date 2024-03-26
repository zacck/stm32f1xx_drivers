/*
 * ssd1306.c
 *
 *  Created on: Mar 18, 2024
 *      Author: zaccko
 */

#include "stm32f1xx.h"
#include "string.h"
#include "font.h"

// Simple video RAM
//static uint8_t ssd1306_vram[SSD1306_ROWS / 8][SSD1306_COLUMNS] = { 0 };
static uint8_t ssd1306_vram[VRAM_SIZE];
unsigned int _pIndex;



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
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV64;
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
void SSD1306_Init(void) {
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

	uint8_t initInstructions[] = { SSD1306_CMD_START,          // start commands
			SSD1306_SETDISPLAY_OFF,         // turn off display
			SSD1306_SETDISPLAYCLOCKDIV,     // set clock:
			0x80,                           //   Fosc = 8, divide ratio = 0+1
			SSD1306_SETMULTIPLEX,           // display multiplexer:
			0x3F,             				//   number of display rows
			SSD1306_VERTICALOFFSET,         // display vertical offset:
			1, 0x00,                        //   no offset
			SSD1306_SETSTARTLINE, 0,    	// RAM start line 0
			SSD1306_SETCHARGEPUMP,          // charge pump:
			0x14,                           //   charge pump ON (0x10 for OFF)
			SSD1306_SETADDRESSMODE,         // addressing mode:
			0x00,                           //   horizontal mode
			SSD1306_COLSCAN_DESCENDING, 0,    // flip columns
			SSD1306_COMSCAN_DESCENDING, 0,     // don't flip rows (pages)
			SSD1306_SETCOMPINS,             // set COM pins
			0x12,                           //   sequential pin mode
			SSD1306_SETCONTRAST,            // set contrast
			1, 0x7F,                           //   minimal contrast
			SSD1306_SETPRECHARGE,           // set precharge period
			0xF1,                           //   phase1 = 15, phase2 = 1
			SSD1306_SETVCOMLEVEL,           // set VCOMH deselect level
			0x20,                           //   ????? (0,2,3)
			SSD1306_ENTIREDISPLAY_OFF,      // use RAM contents for display
			SSD1306_SETINVERT_OFF,          // no inversion
			SSD1306_SCROLL_DEACTIVATE,      // no scrolling
			SSD1306_SETDISPLAY_ON,          // turn on display (normal mode)
			};
	int inst_size = (sizeof(initInstructions) / sizeof(initInstructions[0]));

	SPI_SendData(SPI2, initInstructions, inst_size);

	delay();
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

	_pIndex = x + ((y >> 3) << 7);
	//write to vram
	if (value)
		ssd1306_vram[_pIndex++] |= 0x01 << (y & 0x07);
	else
		ssd1306_vram[_pIndex++] &= ~(0x01 << (y & 0x07));

	return 0;
}

/*
 * Update the position this is to ensure that character is not divided at the end of a row

 * ! \return 0 if successful, error code if failed:
 * !         1: x value out of range
 * !         2: y value out of range
 */
uint8_t ssd1306_updatePosition(void) {

	uint8_t y = _pIndex >> 7;
	uint8_t x = _pIndex - (y << 7);
	uint8_t x_new = x + CHARS_COLS_LENGTH + 1;

	if(x_new > SSD1306_END_COLUMN_ADDR){			//check position
		if(y > SSD1306_END_PAGE_ADDR){				//if more than allowed pages
			return 2;								// return  out of range for y
		} else if(y < (SSD1306_END_PAGE_ADDR-1)) {	// if x reacges the end but still in page range
			_pIndex = ((++y) << 7);					// update
		}
	}

	return 0;
}


void SSD1306_DrawString(char *str){
	int i = 0;
	while(str[i] != '\0'){
		ssd1306_drawChar(str[i++]);
	}
}


/*
 * Draws a given character on the screen
 * param character: character to draw
 *
 */
uint8_t ssd1306_drawChar(char character){
	uint8_t i = 0;

	if(ssd1306_updatePosition() ==  2 /*Y range error*/){
			return 2;
	}
	while(i < CHARS_COLS_LENGTH) {
		ssd1306_vram[_pIndex++] = *(&FONTS[character-32][i++]);
	}
	_pIndex++;

	return 0;
}

/*
 * Sets the cursor positioon to the x y coordinates
 *
 *
 */
void ssd1306_setCursorPosition(uint8_t x, uint8_t y){
	_pIndex = x + (y << 7);
}


/*
 * Clear the display
 * */

void ssd1306_clearDisplay(void){
	memset(ssd1306_vram, 0, sizeof ssd1306_vram);
	ssd1306_display();
}




/*
 * Writes the content of the ram to the screen
 * */
void ssd1306_display(void) {
	//const uint8_t page = y >> 3;
	uint8_t configMsg[] = { SSD1306_CMD_START,
	SSD1306_SETPAGERANGE, 0x0, 0xFF,
			SSD1306_SETCOLRANGE, 0x0, 0x7F };
	SPI_SendData(SPI2, configMsg, sizeof configMsg);

	//Enter Data mode
	GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_8, SET);
	delay();

	SPI_SendData(SPI2, ssd1306_vram, sizeof(ssd1306_vram));

}




