#ifndef PID_H
#define PID_H

typedef struct {
    float Kp;
    float Ki;
    float Kd;

    float setpoint;
    float prev_error;
    float integral;

    float output_limit;     // Max abs(output)
    float integral_limit;   // Max abs(integral term)

    float output;           // Last PID output
}PID_Controller;

void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float output_limit, float integral_limit);
float PID_Compute(PID_Controller *pid, float target, float measured, float dt);
void PID_Reset(PID_Controller *pid);

#endif
