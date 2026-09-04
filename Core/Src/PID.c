/*
 * PID.c
 *
 *  Created on: Aug 30, 2026
 *      Author: kitanat
 */
#include "PID.h"
#include <math.h>

void PID_Init(PID_Controller_t *pid, float Kp, float Ki, float Kd, float out_min, float out_max)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->integral_limit = out_max * 0.5f; /* Default clamp[cite: 5] */
    pid->deadband = 0.05f;                /* In degrees[cite: 5] */
    pid->d_filter_alpha = 0.2f;           /* Default low-pass smoothing[cite: 5] */

    PID_Reset(pid);
}

void PID_Reset(PID_Controller_t *pid)
{
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
    pid->d_filtered = 0.0f;
    pid->prev_feedback = 0.0f;
    pid->d_initialized = false;
    pid->last_raw_angle_deg = 0.0f;
    pid->accumulated_angle_deg = 0.0f;
    pid->is_initialized = false;
}

/* Clears only the dynamic terms when a new target is issued.
 * Keeps the multi-turn tracking intact so the accumulated angle stays valid. */
void PID_SetTarget(PID_Controller_t *pid, float new_target)
{
    (void)new_target;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->d_filtered = 0.0f;
    pid->d_initialized = false;
}

/* Unwraps 0..360 degree jumps to track multi-turn continuous positions[cite: 5] */
float PID_UnwrapAngle(PID_Controller_t *pid, float current_angle_deg)
{
    if (!pid->is_initialized) {
        pid->last_raw_angle_deg = current_angle_deg;
        pid->accumulated_angle_deg = current_angle_deg;
        pid->is_initialized = true;
        return pid->accumulated_angle_deg;
    }

    float delta = current_angle_deg - pid->last_raw_angle_deg;

    /* Handle wrap-around across 0 / 360 boundary[cite: 5] */
    if (delta > 180.0f) {
        delta -= 360.0f;
    } else if (delta < -180.0f) {
        delta += 360.0f;
    }

    pid->accumulated_angle_deg += delta;
    pid->last_raw_angle_deg = current_angle_deg;

    return pid->accumulated_angle_deg;
}

float PID_Update(PID_Controller_t *pid, float setpoint, float feedback, float dt)
{
    if (dt <= 0.0f) return 0.0f;

    float error = setpoint - feedback;

    /* Inside the deadband the axis is considered parked: zero the error AND
     * bleed the integrator off. Leaving the integrator charged here makes it
     * climb on a standing sub-deadband error until the output crosses the step
     * generator's minimum frequency, which shows up as slow creep at rest. */
    if (fabsf(error) < pid->deadband) {
        error = 0.0f;
        pid->integral = 0.0f;
    }

    /* Proportional term[cite: 5] */
    float p_out = pid->Kp * error;

    /* Derivative on measurement: differentiating the feedback instead of the
     * error removes the spike that a setpoint step (or the deadband snapping
     * the error to zero) would otherwise inject into the D term. */
    float derivative = 0.0f;
    if (pid->d_initialized) {
        derivative = -(feedback - pid->prev_feedback) / dt;
    } else {
        pid->d_initialized = true;
    }
    pid->prev_feedback = feedback;

    pid->d_filtered = (pid->d_filter_alpha * derivative) + ((1.0f - pid->d_filter_alpha) * pid->d_filtered);
    float d_out = pid->Kd * pid->d_filtered;

    /* Integral term, clamped so Ki * integral cannot exceed integral_limit */
    pid->integral += error * dt;
    float i_out = pid->Ki * pid->integral;
    if (i_out > pid->integral_limit) {
        i_out = pid->integral_limit;
        if (pid->Ki != 0.0f) pid->integral = i_out / pid->Ki;
    } else if (i_out < -pid->integral_limit) {
        i_out = -pid->integral_limit;
        if (pid->Ki != 0.0f) pid->integral = i_out / pid->Ki;
    }

    pid->prev_error = error;

    /* Total output limited to actuator limits[cite: 5] */
    float output = p_out + i_out + d_out;

    /* Anti-windup by back-calculation: when the output saturates, roll the
     * integrator back by the excess so it does not keep charging. */
    if (output > pid->out_max) {
        if (pid->Ki != 0.0f) pid->integral -= (output - pid->out_max) / pid->Ki;
        output = pid->out_max;
    } else if (output < pid->out_min) {
        if (pid->Ki != 0.0f) pid->integral -= (output - pid->out_min) / pid->Ki;
        output = pid->out_min;
    }

    return output;
}
