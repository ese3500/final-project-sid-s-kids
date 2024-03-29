#include <stdint.h>
#include <avr/io.h>
#include <util/delay.h>

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

	pid->alpha = cos(pid->controlOutput / pid->Vin);
}

void PWM_init() {
	DDRB |= (1 << PORTB1);
	TCCR1A = (1 << COM1A1) | (1 << WGM11) | (1 << WGM10);
	TCCR1B = (1 << WGM12) | (1 << CS11);
}

void set_PWM(uint8_t dutyCycle) {
	OCR1A = dutyCycle;
}

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

uint8_t scaleToUint8(float value) {
	if (value < 0.0f) value = 0.0f;
	if (value > 6.0f) value = 6.0f;

	return (uint8_t)((value / 6.0f) * 255.0f);
}

int main(void) {
	PIDController pid;
	PID_Init(&pid, 0.1f, 0.01f, 0.01f, 5.0f);
	ADC_init();
	PWM_init();
	while (1) {
		float actualVoltage = (read_ADC(0)/1023.0f)*5;
		PID_Update(&pid, actualVoltage, 5);
		uint8_t dutyCycle = scaleToUint8(pid.controlOutput);
		set_PWM(dutyCycle);
		_delay_ms(100);
	}

	return 0;
}