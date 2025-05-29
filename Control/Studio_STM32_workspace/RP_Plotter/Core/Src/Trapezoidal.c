/*
 * Trapezoidal.c
 *
 *  Created on: May 14, 2025
 *      Author: jirat
 */

#include "math.h"
#include "Trapezoidal.h"

void Trapezoidal_Init(VELO_PROFILE *profile, float distance_total, float v_max,
		float a_max) {
	profile->distance_total = fabsf(distance_total);
	profile->v_max = fabsf(v_max);
	profile->a_max = fabsf(a_max);
	profile->current_position = 0.0f;
	profile->current_velocity = 0.0f;
	profile->target_position = distance_total;
	profile->direction = (distance_total >= 0) ? 1.0f : -1.0f;
	profile->finished = 0;

	// Check if triangular or trapezoidal
	float d_total_min = (profile->v_max * profile->v_max) / profile->a_max;
	if (profile->distance_total < d_total_min) {
		// Triangular profile
		profile->is_triangular = 1;
		profile->v_peak = sqrtf(profile->a_max * profile->distance_total);
	} else {
		// Full trapezoidal profile
		profile->is_triangular = 0;
		profile->v_peak = profile->v_max;
	}
}

void Trapezoidal_Update(VELO_PROFILE *profile, float dt) {
	profile->direction = (profile->target_position - profile->current_position >= 0) ? 1.0f : -1.0f;
    if (profile->finished) return;

    float remaining_distance = profile->target_position - profile->current_position;
    float stopping_distance = (profile->current_velocity * profile->current_velocity) / (2.0f * profile->a_max);

    if (fabsf(remaining_distance) <= stopping_distance) {
        // Decelerate
        profile->current_velocity -= profile->a_max * dt * profile->direction;
        if (profile->direction > 0 && profile->current_velocity < 0.0f) profile->current_velocity = 0.0f;
        if (profile->direction < 0 && profile->current_velocity > 0.0f) profile->current_velocity = 0.0f;
    } else {
        // Accelerate
        if (fabsf(profile->current_velocity) < profile->v_peak) {
            profile->current_velocity += profile->a_max * dt * profile->direction;
            if (fabsf(profile->current_velocity) > profile->v_peak) {
                profile->current_velocity = profile->v_peak * profile->direction;
            }
        }
        // In triangular mode, no cruising phase
    }

    // Update position
    // profile->current_position += profile->current_velocity * dt;

    // Check if finished
    if ((profile->direction > 0 && profile->current_position >= profile->target_position) ||
        (profile->direction < 0 && profile->current_position <= profile->target_position)) {
        profile->current_position = profile->target_position;
        profile->current_velocity = 0.0f;
        profile->finished = 1;
    }
}


