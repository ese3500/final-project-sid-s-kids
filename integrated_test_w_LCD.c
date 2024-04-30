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
#define F_CPU 16000000UL
#define BAUD_RATE 9600
#define BAUD_PRESCALER (((F_CPU / (BAUD_RATE * 16UL))) - 1)
#define PI 3.14159265358979323846
#define BUFFER_SIZE 20
#define SNAKE_LENGTH 30


volatile uint32_t totalTicks = 0;
volatile bool lastReadingAboveMid = false;

volatile float frequencyBuffer[BUFFER_SIZE] = {0};
volatile int bufferIndex = 0;
volatile float frequencySum = 0;
volatile int frequencyCount = 0;
volatile float averageFrequency = 60;

volatile int count = 0;

volatile uint8_t zero_detected = 0;
volatile uint16_t time_elapsed = 0;

volatile bool outputFlag = 0;

void ADC_init() {
	ADMUX = (1 << REFS0);
	ADCSRA = (1 << ADEN) | (7 << ADPS0);
}

uint16_t read_ADC(uint8_t channel) {
	ADMUX = (ADMUX & 0xF8) | (channel & 0x07);
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));
	return ADC;
}

void initialize_timers() {
	// Set WGM12 for CTC mode and CS11, CS10 for prescaler 64
	TCCR1B |= (1 << WGM12) | (1 << CS11);
	
	OCR1A = 249;
		
	TIMSK1 |= (1 << OCIE1A);
		
	sei();
}
		 
void initTimer3() {
	TCCR3A = 0;
	TCCR3B = 0;
	TCCR3B |= (1 << CS31); // CS31 set for a prescaler of 8
	TCNT3 = 0;
	TIMSK3 |= (1 << TOIE3); // Enable overflow interrupt
}

ISR(TIMER3_OVF_vect) {
	totalTicks += 0x10000; // Increment by 65536 on each overflow
}

ISR(TIMER1_COMPA_vect) {
	if (zero_detected && !outputFlag) {
		time_elapsed++; // Increment time elapsed
 		if (zero_detected && time_elapsed >= ((960*8/averageFrequency * firing_angle_rad/(2*PI)))) { // 960 is a neccessary change from 1000 due to delays, 8 is prescaler
			PORTD |= (1 << PORTD3);
			_delay_ms(5);
			PORTD &= ~(1 << PORTD3);

		zero_detected = 0;
		time_elapsed = 0;
		outputFlag = 1;
		}
	}
	else if (zero_detected && outputFlag) {
		time_elapsed++; // Increment time elapsed
		if (zero_detected && time_elapsed >= ((960*8/averageFrequency * firing_angle_rad/(2*PI)))) { // 960 is a neccessary change from 1000 due to delays, 8 is prescaler
			PORTD |= (1 << PORTD2);
			
			_delay_ms(5);
			PORTD &= ~(1 << PORTD2);
			
			zero_detected = 0;
			time_elapsed = 0;
			outputFlag = 0;
		}
	}
}

void setFrequency(float newFrequency) {
	frequencySum -= frequencyBuffer[bufferIndex];
	frequencyBuffer[bufferIndex] = newFrequency;
	frequencySum += newFrequency;

	bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;

	if (frequencyCount < BUFFER_SIZE) {
		frequencyCount++;
	}
	averageFrequency = frequencySum / frequencyCount;
}

typedef struct {
	float Kp;
	float Ki;
	float Kd;

	float target;
	float integral;
	float prevError;
	float Vin;
	float f;

	float controlOutput;
	float alpha;
} PIDController;

void PID_Init(PIDController* pid, float Kp, float Ki, float Kd, float target) {
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->target = target;
	pid->integral = 0.0f;
	pid->prevError = 0.0f;
	pid->Vin = 0.0f;
	pid->f = 0.0f;
	pid->controlOutput = 0.0f;
	pid->alpha = 0.0f;
}

void PID_Update(PIDController* pid, float actualValue, float vin) {
	float error = pid->target - actualValue;
	pid->integral += error;
	float derivative = error - pid->prevError;

	pid->controlOutput = (pid->Kp * error) + (pid->Ki * pid->integral) + (pid->Kd * derivative);
	pid->prevError = error;
	pid->Vin = vin;

	pid->alpha = acos(pid->controlOutput / pid->Vin);
}

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

int main(void) {
	ADC_init();
	initTimer3();
	initialize_timers();
	UART_init(BAUD_PRESCALER);
	lcd_init();
	initSnake();
	sei();
	LCD_rotate(1);
	LCD_setScreen(BLACK);
	LCD_drawString(LCD_WIDTH/2-40, LCD_HEIGHT/2-14, "|V_in| = ", WHITE, BLACK);
	LCD_drawString(LCD_WIDTH/2-40, LCD_HEIGHT/2, "f_in = ", WHITE, BLACK);
	LCD_drawString(LCD_WIDTH/2-40, LCD_HEIGHT/2+14, "<V_out> = ", WHITE, BLACK);

	uint16_t adcValue = 0;
	uint32_t lastCapture = 0;
	uint32_t currentTicks;
	
	DDRD |= (1 << DDD3);
	DDRD |= (1 << DDD2);
	
	PIDController pid;
	PID_Init(&pid, 1.0f, 1.0f, 1.0f, 2.0f);

	while (1) {
		adcValue = read_ADC(0);
		bool currentReadingAboveMid = (adcValue >= MID_POINT);
		bool currentReadingBelowMid = (adcValue < MID_POINT);
		if ((!lastReadingAboveMid && currentReadingAboveMid) || (lastReadingAboveMid && currentReadingBelowMid)) {
			// Zero crossing detected
			currentTicks = totalTicks + TCNT3;
			if (currentTicks >= lastCapture) {
				zero_detected = 1;
				uint32_t elapsedTicks = currentTicks - lastCapture;
				setFrequency((float)(F_CPU / 8.0f) / (2*elapsedTicks)); // frequency
				lastCapture = currentTicks;
				char buffer[20];
				sprintf(buffer, "%u \n", (int) averageFrequency);
				UART_putstring(buffer);
				sprintf(buffer, "%u \n", pid.Vin);
				LCD_drawString(LCD_WIDTH/2+20, LCD_HEIGHT/2-14, buffer, WHITE, BLACK);
				sprintf(buffer, "%u \n", (int) averageFrequency);
				LCD_drawString(LCD_WIDTH/2+20, LCD_HEIGHT/2, buffer, WHITE, BLACK);
				sprintf(buffer, "%u \n", pid.controlOutput);
				LCD_drawString(LCD_WIDTH/2+20, LCD_HEIGHT/2+14, buffer, WHITE, BLACK);
			}
		}
		lastReadingAboveMid = currentReadingAboveMid;
		float actualVoltage = 0.0f;
		PID_Update(&pid, actualVoltage, 5.0f);
		drawSnakePath();
	}
}
