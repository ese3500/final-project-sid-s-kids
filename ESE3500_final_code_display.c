#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <stdio.h>
#include "uart.h"
#include <util/delay.h>
#include <stdint.h>
#include "ASCII_LUT.h"
#include "LCD_GFX.h"
#include "ST7735.h"
#define MID_POINT 512

#define BAUD_RATE 9600
#define BAUD_PRESCALER (((F_CPU / (BAUD_RATE * 16UL))) - 1)
#define PI 3.14159265358979323846
#define BUFFER_SIZE 20
#define SNAKE_LENGTH 30

typedef struct {
	uint8_t x;
	uint8_t y;
} Point;

Point snake[SNAKE_LENGTH];

void initSnake() {
	for (int i = 0; i < SNAKE_LENGTH; i++) {
		snake[i].x = 0;
		snake[i].y = i;
	}
}

void drawSnakePath() {
	static int dx = 1, dy = 0;
	static uint8_t headIndex = 0;
	
	Point newHead = {
		snake[headIndex].x + dx,
		snake[headIndex].y + dy
	};

	headIndex = (headIndex + 1) % SNAKE_LENGTH;
	snake[headIndex] = newHead;

	LCD_drawPixel(newHead.x, newHead.y, WHITE);

	int tailIndex = (headIndex + 1) % SNAKE_LENGTH;
	LCD_drawPixel(snake[tailIndex].x, snake[tailIndex].y, BLACK);

	// Check boundaries and change direction if needed
	if ((dx == 1 && newHead.x >= LCD_WIDTH - 1) ||  // right
	(dx == -1 && newHead.x <= 0) ||            // left
	(dy == 1 && newHead.y >= LCD_HEIGHT - 1) || // bottom
	(dy == -1 && newHead.y <= 0)) {             // top
		// change direction clockwise
		if (dx == 1) {
			dx = 0; dy = 1; // turn down
			} else if (dy == 1) {
			dx = -1; dy = 0; // turn left
			} else if (dx == -1) {
			dx = 0; dy = -1; // turn up
			} else if (dy == -1) {
			dx = 1; dy = 0;  // turn right
		}
	}
}

void ADC_init() {
	ADMUX = (1 << REFS0);
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // adc0, adc1, adc2
}

uint16_t read_ADC(uint8_t channel) {
	ADMUX = (ADMUX & 0xF8) | (channel & 0x07);
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));
	return ADC;
}

void int_to_string(int num, char* str) {
	char* ptr = str;
	int temp;

	if (num < 0) {
		num = -num;
		*ptr++ = '-';
	}

	char* start = ptr;
	do {
		temp = num % 10;
		*ptr++ = temp + '0';
		num /= 10;
	} while (num);

	*ptr-- = '\0';

	char ch;
	while (start < ptr) {
		ch = *start;
		*start++ = *ptr;
		*ptr-- = ch;
	}
}

int main(void) {
	ADC_init();
	lcd_init();
 	initSnake();
	sei();
	LCD_rotate(1);
	LCD_setScreen(BLACK);
	LCD_drawString(LCD_WIDTH/2-40, LCD_HEIGHT/2-20, "|V_in| = ", WHITE, BLACK);
	LCD_drawString(LCD_WIDTH/2-40, LCD_HEIGHT/2, "Target = ", WHITE, BLACK);
	LCD_drawString(LCD_WIDTH/2-40, LCD_HEIGHT/2+20, "<V_out> = ", WHITE, BLACK);

	while (1) {
		int vin = read_ADC(0);
		int vout = read_ADC(1);
		int target = read_ADC(2);
		char buffer[20];
		int voltageIn = (int)(vin * 15.0 * 1000 / 1023.0);
		int_to_string(voltageIn, buffer);
		LCD_drawString(LCD_WIDTH/2+20, LCD_HEIGHT/2-20, buffer, WHITE, BLACK);
		int_to_string((int) (vout * 7.5 * 1000.0/1023.0), buffer);
		LCD_drawString(LCD_WIDTH/2+20, LCD_HEIGHT/2, buffer, WHITE, BLACK);
		int_to_string((int)(target * 7.5 * 1000.0/1023.0), buffer);
		LCD_drawString(LCD_WIDTH/2+20, LCD_HEIGHT/2+20, buffer, WHITE, BLACK);
		drawSnakePath();
	}
}