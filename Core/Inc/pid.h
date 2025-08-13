#ifndef PID_H
#define PID_H

typedef struct {
    float Kp;
    float Ki;
    float Kd;

    float target_value;
    float measured_value;
    float measured_value_prev;

    float error;
    float error_sum;
    float error_deriv;

    float output_limit;     // Max abs(output)
    float integral_limit;   // Max abs(integral term)

    float output;           // Last PID output
}PID_Controller;

typedef struct{
	PID_Controller outer_loop;
	PID_Controller inner_loop;
}Double_PID_Controller;


void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float output_limit, float integral_limit);
float PID_Compute(PID_Controller *pid, float target, float measured, float measured_rate, float dt);
float PID_Double_Calculation(Double_PID_Controller *axis, float target_angle, float measured_angle, float measured_rate, float dt);
float PID_Yaw_Rate_Calculation(PID_Controller *axis, float target_rate, float measured_rate, float dt);
float PID_Yaw_Angle_Calculation(PID_Controller *axis, float target_angle, float measured_angle, float measured_rate, float dt);

void PID_Reset(PID_Controller *pid);

#endif
