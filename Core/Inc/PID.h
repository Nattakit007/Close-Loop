/*
 * PID.h
 *
 *  Created on: Aug 30, 2026
 *      Author: kitanat
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    /* PID Gains */
    float Kp;
    float Ki;
    float Kd;

    /* Limits & Deadband */
    float out_min;
    float out_max;
    float integral_limit;
    float deadband;          /* Position error threshold to stop hunting[cite: 5] */

    /* Controller State */
    float prev_error;
    float integral;
    float d_filter_alpha;    /* Low-pass filter coefficient for derivative (0.0 - 1.0)[cite: 5] */
    float d_filtered;
    float prev_feedback;     /* For derivative-on-measurement (avoids setpoint kick) */
    bool  d_initialized;     /* Skips the first derivative sample after a reset */

    /* Multi-turn Tracking */
    float last_raw_angle_deg;
    float accumulated_angle_deg;
    bool  is_initialized;
} PID_Controller_t;

void PID_Init(PID_Controller_t *pid, float Kp, float Ki, float Kd, float out_min, float out_max);
float PID_UnwrapAngle(PID_Controller_t *pid, float current_angle_deg);
float PID_Update(PID_Controller_t *pid, float setpoint, float feedback, float dt);
void PID_Reset(PID_Controller_t *pid);
void PID_SetTarget(PID_Controller_t *pid, float new_target);

#endif /* PID_CONTROLLER_H */
