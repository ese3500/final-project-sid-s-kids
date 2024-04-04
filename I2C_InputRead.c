#include <avr/io.h>
#include <util/twi.h>
#include <stdio.h>

#define F_CPU 16000000UL
#define F_SCL 100000UL // I2C clock frequency, this will likely need to be slowed down significantly to not run out of memory, probably more like a few hundred Hz
#define TWBR_value (((F_CPU / F_SCL) - 16) / (2 * 1))

#define I2C_ADDR // set value

// voltage buffer
#define BUFFER_SIZE 1000
volatile uint16_t voltageBuffer[BUFFER_SIZE];

void setupI2C(void) {
    TWBR = (uint8_t)TWBR_value;
}

void readI2C(uint8_t addr, uint16_t* buffer, uint16_t bufferSize) {
    for(int i = 0; i < bufferSize; i++) {
        TWCR = (1<<TWSTA)|(1<<TWEN);
        while (!(TWCR & (1<<TWINT)));
        
        TWDR = addr | TW_WRITE;
        TWCR = (1<<TWEN);
        while (!(TWCR & (1<<TWINT)));

        TWDR = 0x00;
        TWCR = (1<<TWEN);
        while (!(TWCR & (1<<TWINT)));
        
        TWCR = (1<<TWSTA)|(1<<TWEN);
        while (!(TWCR & (1<<TWINT)));

        TWDR = addr | TW_READ;
        TWCR = (1<<TWEN);
        while (!(TWCR & (1<<TWINT)));
        
        TWCR = (1<<TWEN)|(1<<TWEA);
        while (!(TWCR & (1<<TWINT)));
        uint8_t high_byte = TWDR;
        
        TWCR = (1<<TWEN);
        while (!(TWCR & (1<<TWINT)));
        uint8_t low_byte = TWDR;
        
        buffer[i] = (high_byte << 8) | low_byte;

        TWCR = (1<<TWSTO)|(1<<TWEN);
    }
}

void calculateFrequencyAndAmplitude(uint16_t* buffer, uint16_t bufferSize, float* frequency, float* amplitude) {
    uint16_t max = 0, min = 65535;
    for(uint16_t i = 0; i < bufferSize; i++) {
        if(buffer[i] > max) max = buffer[i];
        if(buffer[i] < min) min = buffer[i];
    }
    *amplitude = (max - min) / 10.0; // Assuming ADC full scale is 5V

    // count zero crossings
    uint16_t zeroCrossings = 0;
    for(uint16_t i = 1; i < bufferSize; i++) {
        if((buffer[i-1] < 32768 && buffer[i] >= 32768) ||
           (buffer[i-1] > 32768 && buffer[i] <= 32768)) {
            zeroCrossings++;
        }
    }
    // Calculate the frequency based on the amount of time elapsed over the buffer, essentially buffer size divide by SCL frequency?
    // *frequency = zeroCrossings / some value;
}

int main(void) {
    float frequency, amplitude;
    setupI2C();
    while(1) {
        readI2C(I2C_ADDR, voltageBuffer, BUFFER_SIZE);
        calculateFrequencyAndAmplitude(voltageBuffer, BUFFER_SIZE, &frequency, &amplitude);
        printf("Frequency: %.2f Hz, Amplitude: %.2f V\n", frequency, amplitude);
    }
}