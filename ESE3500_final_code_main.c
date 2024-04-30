#define F_CPU 16000000UL
#define BAUD_RATE 9600
#define BAUD_PRESCALER (((F_CPU / (BAUD_RATE * 16UL))) - 1)

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "uart.h"
#include <stdio.h>
#include <string.h>
#include "ASCII_LUT.h"
#include "LCD_GFX.h"
#include "ST7735.h"

uint16_t timer_overflows = 0;
uint16_t start_time = 0, end_time = 0;

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

void initialize(void) {
	TIMSK1 |= (1 << TOIE1);
	
	// buzzer as output
	DDRD |= (1 << DDD5);

	// fast PWM mode
	TCCR0A |= (1 << WGM01) | (1 << WGM00);
	TCCR0B |= (1 << WGM02);
	
	TCCR0A |= (1 << COM0A0);
	TCCR0A &= ~(1 << COM0A1);
	TCCR0A |= (1 << COM0B1);
	TCCR0A &= ~(1 << COM0B0);
	
	OCR0A = 100;

	TCCR0B |= (1 << CS01);
	TCCR0B &= ~(1 << CS02);
	
	sei();
}

ISR(TIMER1_OVF_vect) {
	timer_overflows++;
}

typedef struct {
	float Kp;
	float Ki;
	float Kd;

	float target;
	float prevError;
	float Vout;

	float controlOutput;
	float D;
} PIDController;

void PID_Init(PIDController* pid, float Kp, float Ki, float Kd, float target) {
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->target = target;
	pid->prevError = 0.0f;
	pid->Vout = 0.0f;
	pid->controlOutput = 0.0f;
	pid->D = 0.5f;
}

// Update PID Controller - Call this function at a regular interval
void PID_Update(PIDController* pid, float vout, float target) {
	pid->target = target;
	float error = pid->target - vout;
	float derivative = error - pid->prevError;

	pid->prevError = error;
	pid->Vout = vout;

	if (vout > target + 50) {
		pid->D-= pid->Kp*(vout/6-target) + pid->Kd*derivative;
	} else if (vout < target - 50) {
		pid->D+= pid->Kp*(target-vout/6) + pid->Kd*derivative;
	} else {
		pid->D+=0;
	}
	
	if(pid->D > 0.9) {
		pid->D = 0.9;
	}
	if(pid->D < 0.05) {
		pid->D = 0.05;
	}
}

int main(void) {
	initialize();
	ADC_init();
	PIDController pid;
	PID_Init(&pid, 0.00001f, 0, 0.001f, 10.0f);
	while (1) {
		double pot_value = read_ADC(2);
		int vin = read_ADC(0) * 3;
		int vout = read_ADC(1) * 6;
		PID_Update(&pid, vout, pot_value);
		OCR0B = (uint8_t)(pid.D * OCR0A);
	}
}