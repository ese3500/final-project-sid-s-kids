#include <stdint.h>

typedef struct {
    float Kp;
    float Ki;
    float Kd;

    float target;
    float integral;
    float prevError;

    float controlOutput; // Output of the PID controller (voltage from which you calculate firing angle?)
} PIDController;

void PID_Init(PIDController* pid, float Kp, float Ki, float Kd, float target) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->target = target;
    pid->integral = 0.0f;
    pid->prevError = 0.0f;
    pid->controlOutput = 0.0f;
}

// Update PID Controller - Call this function at a regular interval
void PID_Update(PIDController* pid, float actualValue) {
    float error = pid->target - actualValue; 
    pid->integral += error;
    float derivative = error - pid->prevError;

    pid->controlOutput = (pid->Kp * error) + (pid->Ki * pid->integral) + (pid->Kd * derivative);
    pid->prevError = error;

    // Here, maybe add code to set the firing angle based on pid->controlOutput?
    // Map the control output to a specific angle range.
}

int main(void) {
    PIDController myPID;
    PID_Init(&myPID, 1.0f, 0.01f, 0.005f, 5.0f); // example
    while (1) {
        float actualVoltage = 0.0f;
        PID_Update(&myPID, actualVoltage);
        // The firing angle is now in myPID.controlOutput
    }

    return 0;
}