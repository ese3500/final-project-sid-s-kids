#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <stdio.h>
#include "uart.h"
#include <util/delay.h>

#define MID_POINT 512
#define F_CPU 16000000UL
#define BAUD_RATE 9600
#define BAUD_PRESCALER (((F_CPU / (BAUD_RATE * 16UL))) - 1)
#define PI 3.14159265358979323846
#define BUFFER_SIZE 20

volatile uint32_t totalTicks = 0;
volatile bool lastReadingAboveMid = false;

volatile float frequencyBuffer[BUFFER_SIZE] = {0};  // Buffer to store last 5 frequencies
volatile int bufferIndex = 0;                       // Current index for the circular buffer
volatile float frequencySum = 0;                    // Sum of the frequencies in the buffer
volatile int frequencyCount = 0;                    // Current count of frequencies stored
volatile float averageFrequency = 60;               // Start with a default frequency

volatile float firing_angle_rad = PI/2; // Default firing angle of 0 radians
volatile int count = 0;


volatile uint8_t zero_detected = 0;
volatile uint16_t time_elapsed = 0;

// Initialize ADC
void initADC() {
	ADMUX = (1 << REFS0); // Use AVcc as the reference
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Enable ADC, prescaler = 128
}

// Read ADC value from ADC0
uint16_t readADC0() {
	ADMUX = (ADMUX & 0xF0) | 0; // ADC0
	ADCSRA |= (1 << ADSC); // Start conversion
	while (ADCSRA & (1 << ADSC)); // Wait for conversion to complete
	return ADC;
}

void initialize_timers() {
	// Set WGM12 for CTC mode and CS11, CS10 for prescaler 64
	TCCR1B |= (1 << WGM12) | (1 << CS11);
		
	// Set TOP value for 2ms interrupt
	OCR1A = 249;
		
	// Enable Timer1 compare match A interrupt
	TIMSK1 |= (1 << OCIE1A);
		
	// Enable global interrupts
	sei();
}
		 
// Initialize Timer3
void initTimer3() {
	TCCR3A = 0; // Set entire TCCR3A register to 0
	TCCR3B = 0; // Same for TCCR3B
	TCCR3B |= (1 << CS31); // CS31 set for a prescaler of 8
	TCNT3 = 0; // Initialize counter value to 0
	TIMSK3 |= (1 << TOIE3); // Enable overflow interrupt
}

// Timer3 Overflow Interrupt Service Routine
ISR(TIMER3_OVF_vect) {
	totalTicks += 0x10000; // Increment by 65536 on each overflow
}

ISR(TIMER1_COMPA_vect) {
	if (zero_detected) {
		time_elapsed++; // Increment time elapsed
 		if (zero_detected && time_elapsed >= ((960*8/averageFrequency * firing_angle_rad/(2*PI)))) { // 960 is a neccessary change from 1000 due to delays, 8 is prescaler
			// Perform the action (pull PD3 high)
			PORTD |= (1 << PORTD3);
		
		// Reset the pin to low after a delay
			_delay_ms(5);
			PORTD &= ~(1 << PORTD3);
		
		// Reset flags and time elapsed
		zero_detected = 0;
		time_elapsed = 0;
		}
	}
}

void setFrequency(float newFrequency) {
	// Subtract the oldest frequency value from the sum and replace it
	frequencySum -= frequencyBuffer[bufferIndex];
	frequencyBuffer[bufferIndex] = newFrequency;
	frequencySum += newFrequency;

	// Update the buffer index
	bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;

	// Update the count of frequencies stored (max BUFFER_SIZE)
	if (frequencyCount < BUFFER_SIZE) {
		frequencyCount++;
	}

	// Calculate the new average frequency
	averageFrequency = frequencySum / frequencyCount;
}

int main(void) {
	initADC();
	initTimer3();
	initialize_timers();
	UART_init(BAUD_PRESCALER);
	sei(); // Enable global interrupts

	uint16_t adcValue = 0;
	uint32_t lastCapture = 0;
	uint32_t currentTicks;
	
	DDRD |= (1 << DDD3);

	while (1) {
		adcValue = readADC0();
		bool currentReadingAboveMid = (adcValue >= MID_POINT);
		if (!lastReadingAboveMid && currentReadingAboveMid) {
			// Zero crossing detected
			currentTicks = totalTicks + TCNT3;
			if (currentTicks >= lastCapture) {
				zero_detected = 1;
				uint32_t elapsedTicks = currentTicks - lastCapture;
				setFrequency((float)(F_CPU / 8.0f) / elapsedTicks); // frequency
				lastCapture = currentTicks;
				char buffer[20];
				sprintf(buffer, "%u \n", (int) averageFrequency);
				UART_putstring(buffer);
			}
		}
		lastReadingAboveMid = currentReadingAboveMid;
	}
}