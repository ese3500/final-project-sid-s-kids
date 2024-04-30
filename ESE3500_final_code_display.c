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
    uint8_t x;  // x-coordinate of the point
    uint8_t y;  // y-coordinate of the point
} Point;

Point snake[SNAKE_LENGTH];  // Array of Points to store the position of each segment of the snake

// Initializes the snake with all segments in a vertical line
void initSnake() {
    for (int i = 0; i < SNAKE_LENGTH; i++) {
        snake[i].x = 0;  // Set all x-coordinates to 0
        snake[i].y = i;  // Set y-coordinates to 0, 1, 2, ..., SNAKE_LENGTH-1
    }
}

// Draws the snake and handles movement and boundary conditions
void drawSnakePath() {
    static int dx = 1, dy = 0;  // Direction vector for snake's head movement
    static uint8_t headIndex = 0;  // Current index of the snake's head in the array

    // Calculate the new head position based on the current head position and direction
    Point newHead = {
        snake[headIndex].x + dx,
        snake[headIndex].y + dy
    };

    // Update the head index to point to the next element in the snake array
    headIndex = (headIndex + 1) % SNAKE_LENGTH;
    snake[headIndex] = newHead;  // Place new head in the snake array

    // Draw the new head of the snake
    LCD_drawPixel(newHead.x, newHead.y, WHITE);

    // Calculate the index of the tail in the snake array
    int tailIndex = (headIndex + 1) % SNAKE_LENGTH;
    // Erase the tail segment of the snake
    LCD_drawPixel(snake[tailIndex].x, snake[tailIndex].y, BLACK);

    // Check boundaries to change direction if needed
    if ((dx == 1 && newHead.x >= LCD_WIDTH - 1) ||  // right boundary
        (dx == -1 && newHead.x <= 0) ||             // left boundary
        (dy == 1 && newHead.y >= LCD_HEIGHT - 1) || // bottom boundary
        (dy == -1 && newHead.y <= 0)) {             // top boundary
        // Change direction clockwise
        if (dx == 1) {
            dx = 0; dy = 1;  // Turn down
        } else if (dy == 1) {
            dx = -1; dy = 0;  // Turn left
        } else if (dx == -1) {
            dx = 0; dy = -1;  // Turn up
        } else if (dy == -1) {
            dx = 1; dy = 0;   // Turn right
        }
    }
}

// Initialize ADC hardware settings
void ADC_init() {
    ADMUX = (1 << REFS0);  // Set reference voltage to AVcc
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Enable ADC and set prescaler to 128
}

// Read the ADC value from a specified channel
uint16_t read_ADC(uint8_t channel) {
    ADMUX = (ADMUX & 0xF8) | (channel & 0x07);  // Set ADC channel keeping the reference voltage setting
    ADCSRA |= (1 << ADSC);  // Start ADC conversion
    while (ADCSRA & (1 << ADSC));  // Wait until ADC conversion is complete
    return ADC;  // Return the ADC result
}

// Convert an integer to a string
void int_to_string(int num, char* str) {
    char* ptr = str;
    int temp;

    // Handle negative numbers
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

    *ptr-- = '\0';  // Null-terminate the string

    char ch;
    // Reverse the string to display correctly
    while (start < ptr) {
        ch = *start;
        *start++ = *ptr;
        *ptr-- = ch;
    }
}

int main(void) {
    ADC_init();  // Initialize ADC
    lcd_init();  // Initialize the LCD display
    initSnake();  // Initialize the snake game
    sei();  // Enable global interrupts
    LCD_rotate(1);  // Set LCD rotation
    LCD_setScreen(BLACK);  // Clear the LCD to black

    // Draw initial static text on the LCD
    LCD_drawString(LCD_WIDTH/2-40, LCD_HEIGHT/2-20, "|V_in| = ", WHITE, BLACK);
    LCD_drawString(LCD_WIDTH/2-40, LCD_HEIGHT/2, "Target = ", WHITE, BLACK);
    LCD_drawString(LCD_WIDTH/2-40, LCD_HEIGHT/2+20, "<V_out> = ", WHITE, BLACK);

    while (1) {
        // Read voltages and update display continuously
        int vin = read_ADC(0);  // Read input voltage
        int vout = read_ADC(1);  // Read output voltage
        int target = read_ADC(2);  // Read target voltage
        char buffer[20];  // Buffer for string conversion
        int voltageIn = (int)(vin * 15.0 * 1000 / 1023.0);  // Calculate voltage in millivolts
        int_to_string(voltageIn, buffer);
        LCD_drawString(LCD_WIDTH/2+20, LCD_HEIGHT/2-20, buffer, WHITE, BLACK);
        int_to_string((int) (vout * 7.5 * 1000.0/1023.0), buffer);
        LCD_drawString(LCD_WIDTH/2+20, LCD_HEIGHT/2, buffer, WHITE, BLACK);
        int_to_string((int)(target * 7.5 * 1000.0/1023.0), buffer);
        LCD_drawString(LCD_WIDTH/2+20, LCD_HEIGHT/2+20, buffer, WHITE, BLACK);
        drawSnakePath();  // Update the snake's position and draw it
    }
}