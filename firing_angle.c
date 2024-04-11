/*
 * main.c
 *
 * Created: 4/2/2024 2:44:14 PM
 *  Author: Owen
 * 
 * Fires a 10ms pulse a firing angle after a 0 is detected on an input. This will be updated to use the ADC or i2c when we are doing voltage measurements in the future. 
 */ 

// #include <xc.h>


#define F_CPU 16000000UL
#define BAUD_RATE 9600
#define BAUD_PRESCALER (((F_CPU / (BAUD_RATE * 16UL))) - 1)

#include <avr/io.h>
#include <avr/interrupt.h>

#include <avr/delay.h>
#define PI 3.14159265358979323846

volatile uint16_t frequency = 60;
volatile float firing_angle_rad = PI; // Default firing angle of 0 radians
volatile int count = 0;


volatile uint8_t zero_detected = 0;
volatile uint16_t time_elapsed = 0;

void initialize_timers() {
	// Configure Timer1 for CTC mode with prescaler 64
	TCCR1B |= (1 << WGM12) | (1 << CS11) | (1 << CS10);

	// Set TOP value for 1ms interrupt
	OCR1A = 249;

	// Enable Timer1 compare match A interrupt
	TIMSK1 |= (1 << OCIE1A);

	// Enable global interrupts
	sei();
}

ISR(INT0_vect) {
	zero_detected = 1; // Set flag to indicate zero detected
	time_elapsed = 0; // Reset time elapsed
}

ISR(TIMER1_COMPA_vect) {
	if (zero_detected) {
		time_elapsed++; // Increment time elapsed
		if (zero_detected && time_elapsed >= (1137/frequency * firing_angle_rad/(2*PI))) { // 1000 ms delay
			// Perform the action (pull PD3 high)
			PORTD |= (1 << PORTD3);

			// Reset the pin to low after a delay
			_delay_ms(10);
			PORTD &= ~(1 << PORTD3);

			// Reset flags and time elapsed
			zero_detected = 0;
			time_elapsed = 0;
		}
	}
}

int main() {
	// Configure PD2 (INT0) as input with pull-up resistor enabled
	DDRD &= ~(1 << DDD2);
	PORTD |= (1 << PORTD2);

	// Configure PD3 as output
	DDRD |= (1 << DDD3);

	// Enable external interrupt INT0 on falling edge
	EICRA |= (1 << ISC01);
	EIMSK |= (1 << INT0);

	// Initialize timers
	initialize_timers();

	while (1) {

	}

	return 0;
}