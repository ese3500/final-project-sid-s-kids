#include <stdint.h>

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

int main(void) {
    PIDController pid;
    PID_Init(&pid, 1.0f, 1.0f, 1.0f, 2.0f);
    while (1) {
        float actualVoltage = 0.0f;
        PID_Update(&myPID, actualVoltage);
    }

    return 0;
}