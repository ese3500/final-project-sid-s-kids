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
#include "uart.h"

#include <avr/delay.h>
#define PI 3.14159265358979323846

volatile int frequency = 60;
volatile float firing_angle_rad = PI; // Default firing angle of 0 radians
volatile int count = 0;

volatile int period_measurement_1 = 0;
volatile int period_measurement_2 = 0;
volatile int period_overflow = 0;

volatile uint8_t zero_detected = 0;
volatile uint16_t time_elapsed = 0;

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
	cli();
	DDRB &= ~(1<<DDB0);
	DDRB |= (1<<DDB1) | (1<<DDB2) | (1<<DDB5);
	
	TCCR1B |= (1<<ICES1);
	// Prescale the clock by 256 (instead of one overflow every 4ms, we get one ever 1024 ms)
	TCCR1B |= (1<<CS12);
	TCCR1B &= ~(1<<CS11) & ~(1<<CS10);
	
	TIMSK1 |= (1<<ICIE1);
	
	//Overflow Interrupt
	TIMSK1 |= (1 << TOIE1);
	
	TIFR1 |= (1<<ICF1);
	sei();
}

// ISR(INT0_vect) {
// 	zero_detected = 1; // Set flag to indicate zero detected
// 	time_elapsed = 0; // Reset time elapsed
// }

ISR(TIMER1_OVF_vect) {
	period_overflow++;
}

ISR(TIMER1_CAPT_vect) {
	if(zero_detected == 1) {
		period_measurement_1 = period_measurement_2;
		period_measurement_2 = ICR1;
		int period_difference = period_measurement_2 - period_measurement_1;
		
		char buffer[50];
		// Correctly account for overflows
		frequency = (int)((0xFFFF/1024)/period_difference);
		
		//frequency = (int) ((0xFFFF/8) / (2*(period_measurement_2 - period_measurement_1 + (0xFFFF)*period_overflow)));
		sprintf(buffer, "%i, %i, %i, %i \n", period_measurement_2, period_measurement_1, period_overflow, frequency);
		UART_putstring(buffer);
		period_overflow = 0;
	}
	TCCR1B ^= (1<<ICES1);
}

ISR(TIMER1_COMPA_vect) {
	if (zero_detected) {
		time_elapsed++; // Increment time elapsed
		if (zero_detected && time_elapsed >= ((940*8)/frequency * firing_angle_rad/(2*PI))) { //1137
			// Perform the action (pull PD3 high)
			PORTD |= (1 << PORTD3);

			// Reset the pin to low after a delay
			_delay_ms(3);
			PORTD &= ~(1 << PORTD3);

			// Reset flags and time elapsed
			zero_detected = 0;
			time_elapsed = 0;
		}
	}
}

int main() {
	UART_init(BAUD_PRESCALER);
	// Configure PD2 (INT0) as input with pull-up resistor enabled
	DDRD &= ~(1 << DDD2);
	PORTD |= (1 << PORTD2);

	// Configure PD3 as output
	DDRD |= (1 << DDD3);

	// Enable external interrupt INT0 on falling edge
// 	EICRA |= (1 << ISC01);
// 	EIMSK |= (1 << INT0);

	// Initialize timers
	initialize_timers();
	ADC_init();
	
	float lastVoltage = 5;
	while (1) {
		float actualVoltage = (read_ADC(0)/1023.0f)*5;
		if(lastVoltage <= 2.5 && actualVoltage >= 2.5) {
			zero_detected = 1;
			period_measurement_1 = period_measurement_2;
			period_measurement_2 = TCNT1;
			char buffer[50];
			uint32_t period_diff =
			(0x10000 - period_measurement_1 + period_measurement_2);
			
			
		}
		lastVoltage = actualVoltage;
	}

	return 0;
}