/*
 * PID.c
 *
 *  Created on: Aug 30, 2026
 *      Author: kitanat
 */
#include "PID.h"
#include <math.h>

void PID_Init(PID_Controller_t *pid, float Kp, float Ki, float Kd, float min, float max) {
    pid->Kp = Kp; pid->Ki = Ki; pid->Kd = Kd;
    pid->out_min = min; pid->out_max = max;
    pid->integral_limit = max * 0.5f;
    pid->deadband = 0.05f;
    pid->d_filter_alpha = 0.2f;
    pid->integral = 0.0f; pid->prev_error = 0.0f;
    pid->is_initialized = false;
}

float PID_UnwrapAngle(PID_Controller_t *pid, float current_angle_deg) {
    if (!pid->is_initialized) {
        pid->last_raw_angle_deg = current_angle_deg;
        pid->accumulated_angle_deg = current_angle_deg;
        pid->is_initialized = true;
        return pid->accumulated_angle_deg;
    }
    float delta = current_angle_deg - pid->last_raw_angle_deg;
    if (delta > 180.0f) delta -= 360.0f;
    else if (delta < -180.0f) delta += 360.0f;

    pid->accumulated_angle_deg += delta;
    pid->last_raw_angle_deg = current_angle_deg;
    return pid->accumulated_angle_deg;
}

float PID_Update(PID_Controller_t *pid, float setpoint, float feedback, float dt) {
    if (dt <= 0.0f) return 0.0f;
    float error = setpoint - feedback;

    if (fabsf(error) < pid->deadband) error = 0.0f;

    float p_out = pid->Kp * error;

    pid->integral += error * dt;
    if (pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    float i_out = pid->Ki * pid->integral;

    float derivative = (error - pid->prev_error) / dt;
    pid->d_filtered = (pid->d_filter_alpha * derivative) + ((1.0f - pid->d_filter_alpha) * pid->d_filtered);
    float d_out = pid->Kd * pid->d_filtered;

    pid->prev_error = error;
    float output = p_out + i_out + d_out;

    if (output > pid->out_max) output = pid->out_max;
    if (output < pid->out_min) output = pid->out_min;
    return output;
}

