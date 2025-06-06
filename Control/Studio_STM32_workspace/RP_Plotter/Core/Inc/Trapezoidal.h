/*
 * Trapezoidal.h
 *
 *  Created on: May 14, 2025
 *      Author: jirat
 */

#ifndef INC_TRAPEZOIDAL_H_
#define INC_TRAPEZOIDAL_H_

typedef struct {
    float distance_total;
    float v_max;
    float a_max;
    float v_peak;
    float current_position;
    float current_velocity;
    float current_acceleration;
    float target_position;
    float direction;
    float elapsed_time;
    int finished;
    int is_triangular;

    // Precomputed values for smoother update
    float t_acc;
    float t_flat;
    float t_total;
    float d_acc;
    float d_flat;
} VELO_PROFILE;


void Trapezoidal_Init(VELO_PROFILE *profile, float distance_total, float v_max, float a_max);
void Trapezoidal_Update(VELO_PROFILE *profile, float dt);

#endif /* INC_TRAPEZOIDAL_H_ */
