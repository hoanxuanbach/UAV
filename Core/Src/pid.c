#include "pid.h"

void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float output_limit, float integral_limit)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;

    pid->output_limit = output_limit;
    pid->integral_limit = integral_limit;
}

float PID_Compute(PID_Controller *pid, float target, float measured, float dt)
{
    float error = target - measured;
    pid->integral += error * dt;

    // Limit integral to prevent windup
    if (pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;

    float derivative = (error - pid->prev_error) / dt;

    pid->output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;

    // Limit output
    if (pid->output > pid->output_limit) pid->output = pid->output_limit;
    if (pid->output < -pid->output_limit) pid->output = -pid->output_limit;

    pid->prev_error = error;

    return pid->output;
}

void PID_Reset(PID_Controller *pid)
{
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}
