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

// Update PID Controller - Call this function at a regular interval
void PID_Update(PIDController* pid, float actualValue, float vin) {
    float error = pid->target - actualValue; 
    pid->integral += error;
    float derivative = error - pid->prevError;

    pid->controlOutput = (pid->Kp * error) + (pid->Ki * pid->integral) + (pid->Kd * derivative);
    pid->prevError = error;
    pid->Vin = vin;

    pid->alpha = cos(controlOutput / Vin);
}

void PWM_init() {
    DDRB |= (1 << PB1); // Set PB1 as output
    TCCR1A = (1 << COM1A1) | (1 << WGM11) | (1 << WGM10); // Fast PWM, 8-bit
    TCCR1B = (1 << WGM12) | (1 << CS11); // Prescaler = 8
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

int main(void) {
    PIDController pid;
    PID_Init(&pid, 1.0f, 1.0f, 1.0f, 2.0f);
    ADC_init();
    PWM_init();
    while (1) {
        float actualVoltage = (read_ADC(0)/1023f)*5;
        PID_Update(&myPID, actualVoltage, 5);
        uint8_t dutyCycle = (uint8_t) pid->controlOutput;
        set_PWM(dutyCycle);
        _delay_us(100);
    }

    return 0;
}