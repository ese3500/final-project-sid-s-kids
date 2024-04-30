// With Professor McGill-Gardner's permission, I have commented this code using ChatGPT. The specific prompt used was:

// "Please help me comment the following code for a pong game in C. 
// Please be very careful not to change any of the code I have written at all (no optimizations, no fixes of any errors you may perceive, 
// nothing of that sort). The structure of the comments should be as follows: before any method, 
// an explanation of the overall purpose of the method should be given, plus an explanation of the arguments to the method 
// (if there are any). Then, within a method, explain any blocks or lines that aren't directly clear in their purpose. 
// If there is embedded C code setting pins, registers, or Atmega 328PB specific code, please do write next to these lines what 
// the block is setting or why. Please do not over-comment the code, only comment what seems necessary for understanding the code or 
// that helps with understanding. Here is the code:"

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

// Global variables for timer handling
uint16_t timer_overflows = 0;
uint16_t start_time = 0, end_time = 0;

// Initialize ADC configuration
void ADC_init() {
    ADMUX = (1 << REFS0);  // Set reference voltage for ADC
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);  // Enable ADC and set prescaler to 128
}

// Read ADC value from specified channel
uint16_t read_ADC(uint8_t channel) {
    ADMUX = (ADMUX & 0xF8) | (channel & 0x07);  // Select ADC channel with safety mask
    ADCSRA |= (1 << ADSC);  // Start ADC conversion
    while (ADCSRA & (1 << ADSC));  // Wait until ADC conversion is complete
    return ADC;  // Return ADC conversion result
}

// Initialize system configurations
void initialize(void) {
    TIMSK1 |= (1 << TOIE1);  // Enable Timer1 overflow interrupt
    
    // Set buzzer pin as output
    DDRD |= (1 << DDD5);

    // Configure Timer0 for fast PWM mode
    TCCR0A |= (1 << WGM01) | (1 << WGM00);
    TCCR0B |= (1 << WGM02);
    
    // Set Timer0 Compare Output modes for non-inverting PWM
    TCCR0A |= (1 << COM0A0);
    TCCR0A &= ~(1 << COM0A1);
    TCCR0A |= (1 << COM0B1);
    TCCR0A &= ~(1 << COM0B0);
    
    // Set compare value for PWM duty cycle
    OCR0A = 100;

    // Set Timer0 clock prescaler to 8
    TCCR0B |= (1 << CS01);
    TCCR0B &= ~(1 << CS02);
    
    sei();  // Enable global interrupts
}

// Timer1 overflow interrupt service routine
ISR(TIMER1_OVF_vect) {
    timer_overflows++;  // Increment overflow count
}

// Define structure for PID controller
typedef struct {
    float Kp, Ki, Kd; // PID coefficients
    float target; // Target PID value
    float prevError; // Previous error
    float Vout; // Measured output
    float controlOutput; // Control output
    float D; // Duty cycle
} PIDController;

// Initialize PID controller settings
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

// Update PID control loop
void PID_Update(PIDController* pid, float vout, float target) {
    pid->target = target;
    float error = pid->target - vout;
    float derivative = error - pid->prevError;

    pid->prevError = error;
    pid->Vout = vout;

    if (vout > target + 50) {
        pid->D -= pid->Kp * (vout / 6 - target) + pid->Kd * derivative;
    } else if (vout < target - 50) {
        pid->D += pid->Kp * (target - vout / 6) + pid->Kd * derivative;
    } else {
        pid->D += 0;
    }
    
    // Limit duty cycle to prevent saturation
    if (pid->D > 0.9) {
        pid->D = 0.9;
    }
    if (pid->D < 0.05) {
        pid->D = 0.05;
    }
}

// Main function to set up and run control loop
int main(void) {
    initialize(); // Initialize peripherals
    ADC_init(); // Initialize ADC
    PIDController pid; // Create PID controller instance
    PID_Init(&pid, 0.00001f, 0, 0.001f, 10.0f); // Initialize PID controller
    while (1) {
        double pot_value = read_ADC(2); // Read potentiometer value
        int vin = read_ADC(0) * 3; // Read and scale input voltage
        int vout = read_ADC(1) * 6; // Read and scale output voltage
        PID_Update(&pid, vout, pot_value); // Update PID controller
        OCR0B = (uint8_t)(pid.D * OCR0A); // Update PWM duty cycle based on PID output
    }
}