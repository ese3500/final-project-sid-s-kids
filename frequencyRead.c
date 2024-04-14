#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <stdio.h>
#include "uart.h"

#define MID_POINT 512
#define F_CPU 16000000UL
#define BAUD_RATE 9600
#define BAUD_PRESCALER (((F_CPU / (BAUD_RATE * 16UL))) - 1)

volatile uint32_t totalTicks = 0;
volatile bool lastReadingAboveMid = false;
volatile float frequency = 0.0;

void initADC() {
	ADMUX = (1 << REFS0);
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // prescaler = 128
}

uint16_t readADC0() {
	ADMUX = (ADMUX & 0xF0) | 0;
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));
	return ADC;
}

void initTimer3() {
	TCCR3A = 0; // Set entire TCCR3A register to 0
	TCCR3B = 0; // Same for TCCR3B
	TCCR3B |= (1 << CS30); // No prescaling
	TCNT3 = 0; // Initialize counter value to 0
	TIMSK3 |= (1 << TOIE3); // Enable overflow interrupt
}

ISR(TIMER3_OVF_vect) {
	totalTicks += 0x10000; // increment by 65536 on each overflow
}

int main(void) {
	initADC();
	initTimer3();
	UART_init(BAUD_PRESCALER);
	sei();

	uint16_t adcValue = 0;
	uint32_t lastCapture = 0;
	uint32_t currentTicks;

	while (1) {
		adcValue = readADC0();
		bool currentReadingAboveMid = (adcValue >= MID_POINT);
		if (!lastReadingAboveMid && currentReadingAboveMid) {
			currentTicks = totalTicks + TCNT3;
			if (currentTicks >= lastCapture) {
				uint32_t elapsedTicks = currentTicks - lastCapture;
				frequency = (float)F_CPU / elapsedTicks;
				lastCapture = currentTicks;
				char buffer[20];
				sprintf(buffer, "%u \n", (int) frequency);
				UART_putstring(buffer);
			}
		}
		lastReadingAboveMid = currentReadingAboveMid;
	}
}