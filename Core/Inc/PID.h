/*
 * PID.h
 *
 *  Created on: Aug 30, 2026
 *      Author: kitanat
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float Kp, Ki, Kd;
    float out_min, out_max, integral_limit, deadband;
    float prev_error, integral, d_filter_alpha, d_filtered;
    float last_raw_angle_deg, accumulated_angle_deg;
    bool is_initialized;
} PID_Controller_t;

void PID_Init(PID_Controller_t *pid, float Kp, float Ki, float Kd, float out_min, float out_max);
float PID_UnwrapAngle(PID_Controller_t *pid, float current_angle_deg);
float PID_Update(PID_Controller_t *pid, float setpoint, float feedback, float dt);

#endif /* INC_PID_H_ */
