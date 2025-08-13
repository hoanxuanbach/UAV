#include "pid.h"

void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float output_limit, float integral_limit)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    pid->target_value = 0.0f;
    pid->measured_value = 0.0f;
    pid->measured_value_prev = 0.0f;

    pid->error = 0.0f;
    pid->error_sum = 0.0f;
    pid->error_deriv = 0.0f;

    pid->output = 0.0f;

    pid->output_limit = output_limit;
    pid->integral_limit = integral_limit;
}

float PID_Compute(PID_Controller *pid, float target, float measured, float measured_rate, float dt)
{
    pid->target_value = target;
    pid->measured_value = measured;

    pid->error = pid->target_value - pid->measured_value;
    pid->error_sum += pid->error * dt;

    // Clamp integral (anti-windup)
    if (pid->error_sum > pid->integral_limit)
        pid->error_sum = pid->integral_limit;
    else if (pid->error_sum < -pid->integral_limit)
        pid->error_sum = -pid->integral_limit;

    // Use gyro rate as derivative of error
    pid->error_deriv = -measured_rate;

    // PID output
    pid->output = pid->Kp * pid->error +
                  pid->Ki * pid->error_sum +
                  pid->Kd * pid->error_deriv;

    // Clamp output
    if (pid->output > pid->output_limit)
        pid->output = pid->output_limit;
    else if (pid->output < -pid->output_limit)
        pid->output = -pid->output_limit;

    return pid->output;
}

float PID_Double_Calculation(Double_PID_Controller *axis, float target_angle, float measured_angle, float measured_rate, float dt){
	/*Double PID outer loop calculation */
	axis->outer_loop.target_value = target_angle;
	axis->outer_loop.measured_value = measured_angle;

	axis->outer_loop.error = axis->outer_loop.target_value - axis->outer_loop.measured_value;
	axis->outer_loop.error_sum += axis->outer_loop.error * dt;
	if (axis->outer_loop.error_sum > axis->outer_loop.integral_limit) axis->outer_loop.error_sum = axis->outer_loop.integral_limit;
	else if (axis->outer_loop.error_sum < -axis->outer_loop.integral_limit) axis->outer_loop.error_sum = -axis->outer_loop.integral_limit;

	axis->outer_loop.error_deriv = -measured_rate;

	axis->outer_loop.output = axis->outer_loop.Kd * axis->outer_loop.error +
							  axis->outer_loop.Ki * axis->outer_loop.error_sum +
							  axis->outer_loop.Kp * axis->outer_loop.error_deriv;

	if (axis->outer_loop.output > axis->outer_loop.output_limit) axis->outer_loop.output = axis->outer_loop.output_limit;
	else if (axis->outer_loop.output < -axis->outer_loop.output_limit) axis->outer_loop.output = -axis->outer_loop.output_limit;

	/*Double PID inner loop calculation*/
	axis->inner_loop.target_value = axis->outer_loop.output;
	axis->inner_loop.measured_value = measured_rate;

	axis->inner_loop.error = axis->inner_loop.target_value - axis->inner_loop.measured_value;
	axis->inner_loop.error_sum += axis->inner_loop.error * dt;
	if (axis->inner_loop.error_sum > axis->inner_loop.integral_limit) axis->inner_loop.error_sum = axis->inner_loop.integral_limit;
	else if (axis->inner_loop.error_sum < -axis->inner_loop.integral_limit) axis->inner_loop.error_sum = -axis->inner_loop.integral_limit;

	axis->inner_loop.error_deriv = -(axis->inner_loop.measured_value - axis->inner_loop.measured_value_prev) / dt;
	axis->inner_loop.measured_value_prev = axis->inner_loop.measured_value;

	axis->inner_loop.output = axis->inner_loop.Kd * axis->inner_loop.error +
			  	  	  	  	  axis->inner_loop.Ki * axis->inner_loop.error_sum +
							  axis->inner_loop.Kp * axis->inner_loop.error_deriv;

	return axis->inner_loop.output;
}

/*
 * Yaw angle duoc dung khi throttle chinh yaw duoc giu nguyen -> giu cho heading khong thay doi
 * Yaw rate duoc dung khi throttle di chuyen -> dung pid dieu chinh toc do
 */
float PID_Yaw_Angle_Calculation(PID_Controller *axis, float target_angle, float measured_angle, float measured_rate, float dt){
	axis->target_value = target_angle;
	axis->measured_value = measured_angle;

	axis->error = axis->target_value - axis->measured_value;
	//Deal voi truong hop di tu 10 do quay sang 350 do chang han
	if (axis->error > 180.f) axis->error -= 360.f;
	else if (axis->error < -180.f) axis->error += 360.f;

	axis->error_sum += axis->error * dt;

	axis->error_deriv = -measured_rate;

	axis->output = axis->Kd * axis->error +
  	  	  	  	   axis->Ki * axis->error_sum +
				   axis->Kp * axis->error_deriv;
}

float PID_Yaw_Rate_Calculation(PID_Controller *axis, float target_rate, float measured_rate, float dt){
	axis->target_value = target_rate;
	axis->measured_value = measured_rate;

	axis->error = axis->target_value - axis->measured_value;

	axis->error_sum += axis->error * dt;

	axis->error_deriv = -(axis->measured_value - axis->measured_value_prev) / dt;
	axis->measured_value_prev = axis->measured_value;

	axis->output = axis->Kd * axis->error +
  	  	  	  	   axis->Ki * axis->error_sum +
				   axis->Kp * axis->error_deriv;
}
void PID_Reset(PID_Controller *pid)
{
    pid->target_value = 0.0f;
    pid->measured_value = 0.0f;
    pid->measured_value_prev = 0.0f;

    pid->error = 0.0f;
    pid->error_sum = 0.0f;
    pid->error_deriv = 0.0f;

    pid->output = 0.0f;
}
