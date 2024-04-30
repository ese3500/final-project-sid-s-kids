/*
 * LCD_GFX.c
 *
 * Created: 9/20/2021 6:54:25 PM
 *  Author: You
 */ 

#include "LCD_GFX.h"
#include "ST7735.h"

/******************************************************************************
* Local Functions
******************************************************************************/



/******************************************************************************
* Global Functions
******************************************************************************/

/**************************************************************************//**
* @fn			uint16_t rgb565(uint8_t red, uint8_t green, uint8_t blue)
* @brief		Convert RGB888 value to RGB565 16-bit color data
* @note
*****************************************************************************/
uint16_t rgb565(uint8_t red, uint8_t green, uint8_t blue)
{
	return ((((31*(red+4))/255)<<11) | (((63*(green+2))/255)<<5) | ((31*(blue+4))/255));
}

/**************************************************************************//**
* @fn			void LCD_drawPixel(uint8_t x, uint8_t y, uint16_t color)
* @brief		Draw a single pixel of 16-bit rgb565 color to the x & y coordinate
* @note
*****************************************************************************/
void LCD_drawPixel(uint8_t x, uint8_t y, uint16_t color) {
	LCD_setAddr(x,y,x,y);
	SPI_ControllerTx_16bit(color);
}

/**************************************************************************//**
* @fn			void LCD_drawChar(uint8_t x, uint8_t y, uint16_t character, uint16_t fColor, uint16_t bColor)
* @brief		Draw a character starting at the point with foreground and background colors
* @note
*****************************************************************************/
void LCD_drawChar(uint8_t x, uint8_t y, uint16_t character, uint16_t fColor, uint16_t bColor){
	uint16_t row = character - 0x20;		//Determine row of ASCII table starting at space
	int i, j;
	if ((LCD_WIDTH-x>7)&&(LCD_HEIGHT-y>7)){
		for(i=0;i<5;i++){
			uint8_t pixels = ASCII[row][i]; //Go through the list of pixels
			for(j=0;j<8;j++){
				if ((pixels>>j)&1==1){
					LCD_drawPixel(x+i,y+j,fColor);
				}
				else {
					LCD_drawPixel(x+i,y+j,bColor);
				}
			}
		}
	}
}

// void LCD_drawChar(uint8_t x, uint8_t y, uint16_t character, uint16_t fg, uint16_t bg) {
// 	if (x + 5 > LCD_WIDTH || y + 8 > LCD_HEIGHT) return;  // Boundary check
// 
// 	uint16_t row = character - 0x20;  // Determine row of ASCII table starting at space
// 	LCD_setAddr(x, y, x+5, y+8);  // Set address window for a 5x8 character
// 
// 	clear(LCD_PORT, LCD_DC);
// 	SPI_ControllerTx(ST7735_RAMWR);  // Command to write to RAM
// 	set(LCD_PORT, LCD_DC);
// 	clear(LCD_PORT, LCD_TFT_CS);  // Begin SPI transmission
// 
// 	for (int i = 0; i < 5; i++) {
// 		uint8_t pixels = ASCII[row][i];  // Get column data
// 		for (int j = 0; j < 8; j++) {
// 			uint16_t pixelColor = (pixels & (1 << j)) ? fg : bg;
// 			SPI_ControllerTx_16bit(pixelColor);  // Send pixel color
// 		}
// 	}
// 
// 	set(LCD_PORT, LCD_TFT_CS);  // End SPI transmission
// }

// void LCD_drawChar(uint8_t x, uint8_t y, uint16_t character, uint16_t fg, uint16_t bg) {
// 	if (x + 5 > LCD_WIDTH || y + 8 > LCD_HEIGHT) return;  // Boundary check
// 
// 	uint16_t row = character - 0x20;  // Determine row of ASCII table starting at space
// 	LCD_setAddr(x, y, x+4, y+7);  // Set address window for a 5x8 character
// 
// 	clear(LCD_PORT, LCD_DC);
// 	SPI_ControllerTx(ST7735_RAMWR);  // Command to write to RAM
// 	set(LCD_PORT, LCD_DC);
// 	clear(LCD_PORT, LCD_TFT_CS);  // Begin SPI transmission
// 
// 	for (int col = 0; col < 5; col++) {
// 		uint8_t pixels = ASCII[row][col];  // Get column data
// 		for (int bit = 0; bit < 8; bit++) {
// 			uint16_t pixelColor = (pixels & (1 << bit)) ? fg : bg;
// 			SPI_ControllerTx_16bit(pixelColor);  // Send pixel color
// 		}
// 	}
// 
// 	set(LCD_PORT, LCD_TFT_CS);  // End SPI transmission
// }



/******************************************************************************
* LAB 4 TO DO. COMPLETE THE FUNCTIONS BELOW.
* You are free to create and add any additional files, libraries, and/or
*  helper function. All code must be authentically yours.
******************************************************************************/

/**************************************************************************//**
* @fn			void LCD_drawCircle(uint8_t x0, uint8_t y0, uint8_t radius,uint16_t color)
* @brief		Draw a colored circle of set radius at coordinates
* @note			Used this article on Midpoint Circle Algorithm: https://en.wikipedia.org/wiki/Midpoint_circle_algorithm
*****************************************************************************/
void LCD_drawCircle(uint8_t x0, uint8_t y0, uint8_t radius,uint16_t color)
{
	int8_t x = radius;
    int8_t y = 0;
    int16_t d = 1 - radius;

    while (x >= y)
    {
        LCD_drawLine(x0 - x, y0 + y, x0 + x, y0 + y, color);
        LCD_drawLine(x0 - y, y0 + x, x0 + y, y0 + x, color);
        LCD_drawLine(x0 - x, y0 - y, x0 + x, y0 - y, color);
        LCD_drawLine(x0 - y, y0 - x, x0 + y, y0 - x, color);

        if (d <= 0)
        {
            y += 1;
            d += 2*y + 1;
        }
        if (d > 0)
        {
            x -= 1;
            d -= 2*x + 1;
        }
    }
}


/**************************************************************************//**
* @fn			void LCD_drawLine(short x0,short y0,short x1,short y1,uint16_t c)
* @brief		Draw a line from and to a point with a color
* @note			Used this article on Bresenham's Line Algorithm: https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
*****************************************************************************/
void LCD_drawLine(short x0,short y0,short x1,short y1,uint16_t c)
{
	short dx;
    short dy;
    short d;
    short ystep;
    short temp;
    
    if (abs(y1 - y0) > abs(x1 - x0)) { // steep case
        temp = x0;
        x0 = y0;
        y0 = temp;
        temp = x1;
        x1 = y1;
        y1 = temp;
    }
    
    if (x0 > x1) {
        temp = x0;
        x0 = x1;
        x1 = temp;
        temp = y0;
        y0 = y1;
        y1 = temp;
    }
    
    dx = x1 - x0;
    dy = abs(y1 - y0);
    d = dx / 2;
    
    if (y0 < y1) {
        ystep = 1;
    } else {
        ystep = -1;
    }

    while (x0 <= x1) {
        if (abs(y1 - y0) > abs(x1 - x0)) { // steep case
            LCD_drawPixel(y0, x0, c);
        } else {
            LCD_drawPixel(x0, y0, c);
        }
        
        d -= dy;
        if (d < 0) {
            y0 += ystep;
            d += dx;
        }
        
        x0++;
    }
}

/**************************************************************************//**
* @fn			void LCD_drawBlock(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1,uint16_t color)
* @brief		Draw a colored block at coordinates
* @note
*****************************************************************************/
void LCD_drawBlock(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1,uint16_t color)
{
	if (x0 > x1) { 
        uint8_t tmp = x0; 
        x0 = x1; 
        x1 = tmp; 
    }
    if (y0 > y1) { 
        uint8_t tmp = y0; 
        y0 = y1; 
        y1 = tmp; 
    }

    LCD_setAddr(x0, y0, x1, y1);
    clear(LCD_PORT, LCD_DC);
    SPI_ControllerTx(ST7735_RAMWR);
    set(LCD_PORT, LCD_DC);
    clear(LCD_PORT, LCD_TFT_CS);
    uint32_t totalPixels = (uint32_t)(x1 - x0 + 1) * (uint32_t)(y1 - y0 + 1);
    
	for (int i = 0; i < totalPixels; i++) {
        SPI_ControllerTx_16bit_stream(color);
    }
    set(LCD_PORT, LCD_TFT_CS);
}

/**************************************************************************//**
* @fn			void LCD_setScreen(uint16_t color)
* @brief		Draw the entire screen to a color
* @note
*****************************************************************************/
void LCD_setScreen(uint16_t color) 
{
    LCD_setAddr(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1);
    
    clear(LCD_PORT, LCD_DC);
    SPI_ControllerTx(ST7735_RAMWR);
    set(LCD_PORT, LCD_DC);
    clear(LCD_PORT, LCD_TFT_CS);
    uint32_t totalPixels = LCD_WIDTH * (uint32_t)LCD_HEIGHT;

    for (int i = 0; i < totalPixels; i++) {
        SPI_ControllerTx_16bit_stream(color);
    }
    
    set(LCD_PORT, LCD_TFT_CS);
}

/**************************************************************************//**
* @fn			void LCD_drawString(uint8_t x, uint8_t y, char* str, uint16_t fg, uint16_t bg)
* @brief		Draw a string starting at the point with foreground and background colors
* @note
*****************************************************************************/
void LCD_drawString(uint8_t x, uint8_t y, char* str, uint16_t fg, uint16_t bg) {
	uint8_t start = x;
	while (*str) {
		switch (*str) {
			case '\n':
			y += 8;
			x = start;
			break;
			case '\r':
			x = start;
			break;
			default:
			LCD_drawChar(x, y, *str, fg, bg);
			x += 5;
			break;
		}
		str++;
	}
}

void display_three_digits(int number, uint8_t x, uint8_t y, uint16_t color, uint16_t bgColor) {
	if (number < 0) number = 0;
	if (number > 999) number = 999;

	char digits[4];
	digits[0] = (number / 100) % 10 + '0'; // Hundreds
	digits[1] = (number / 10) % 10 + '0';  // Tens
	digits[2] = number % 10 + '0';         // Units
	digits[3] = '\0';                      // Null terminator

	uint8_t width = 5 * 3;  // Each character 5 pixels wide, three characters total
	uint8_t height = 8;     // Each character 8 pixels tall

	LCD_setAddr(x, y, x + width, y + height);
	clear(LCD_PORT, LCD_DC);
	SPI_ControllerTx(ST7735_RAMWR);
	set(LCD_PORT, LCD_DC);
	clear(LCD_PORT, LCD_TFT_CS);

	for (int i = 0; i < 3; i++) {
		uint8_t index = digits[i] - '0';
		index += 16;

		for (int col = 0; col < 5; col++) {
			uint8_t pixels = ASCII[index][col];
			for (int row = 0; row < 8; row++) {
				uint16_t pixelColor = (pixels & (1 << row)) ? color : bgColor;
				SPI_ControllerTx_16bit_stream(pixelColor);
			}
		}
	}

	set(LCD_PORT, LCD_TFT_CS);
}