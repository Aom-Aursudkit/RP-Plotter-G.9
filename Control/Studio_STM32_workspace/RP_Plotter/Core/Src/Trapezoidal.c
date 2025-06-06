/*
 * Trapezoidal.c
 *
 *  Created on: May 14, 2025
 *      Author: jirat
 */

#include "math.h"
#include "Trapezoidal.h"

void Trapezoidal_Init(VELO_PROFILE *profile, float distance_total, float v_max, float a_max) {
    profile->distance_total = fabsf(distance_total);
    profile->v_max = fabsf(v_max);
    profile->a_max = fabsf(a_max);
    profile->current_position = 0.0f;
    profile->current_velocity = 0.0f;
    profile->target_position = distance_total;
    profile->direction = (distance_total >= 0.0f) ? 1.0f : -1.0f;
    profile->finished = 0;
    profile->elapsed_time = 0.0f;

    // Check if triangular profile is required
    float d_total_min = (profile->v_max * profile->v_max) / profile->a_max;
    if (profile->distance_total < d_total_min) {
        profile->is_triangular = 1;
        profile->v_peak = sqrtf(profile->a_max * profile->distance_total);
    } else {
        profile->is_triangular = 0;
        profile->v_peak = profile->v_max;
    }

    // Precompute timing and distances
    profile->t_acc = profile->v_peak / profile->a_max;
    profile->d_acc = 0.5f * profile->a_max * profile->t_acc * profile->t_acc;

    if (profile->is_triangular) {
        profile->t_flat = 0.0f;
        profile->d_flat = 0.0f;
    } else {
        profile->d_flat = profile->distance_total - 2 * profile->d_acc;
        profile->t_flat = profile->d_flat / profile->v_peak;
    }

    profile->t_total = 2 * profile->t_acc + profile->t_flat;
}

void Trapezoidal_Update(VELO_PROFILE *profile, float dt) {
    if (profile->finished) return;

    profile->elapsed_time += dt;
    float t = profile->elapsed_time;

    float a = profile->a_max;
    float v = profile->v_peak;
    float d_total = profile->distance_total;

    float t_acc = v / a;
    float d_acc = 0.5f * a * t_acc * t_acc;

    float t_flat = 0.0f;
    float d_flat = 0.0f;

    if (!profile->is_triangular) {
        d_flat = d_total - 2 * d_acc;
        t_flat = d_flat / v;
    }

    float t1 = t_acc;
    float t2 = t_acc + t_flat;
    float t3 = t_acc + t_flat + t_acc;

    float pos = 0.0f;
    float vel = 0.0f;
    float acc = 0.0f;
    float eps = 1e-6f;

    if (t < t1 - eps) {
        // Acceleration phase
        pos = 0.5f * a * t * t;
        vel = a * t;
        acc = a;
    } else if (t < t2 - eps) {
        // Constant velocity phase
        float t_flat_phase = t - t1;
        pos = d_acc + v * t_flat_phase;
        vel = v;
        acc = 0.0f;
    } else if (t < t3 - eps) {
        // Deceleration phase
        float td = t - t2;
        pos = d_acc + d_flat + v * td - 0.5f * a * td * td;
        vel = v - a * td;
        acc = -a;
    } else {
        // Motion complete
        pos = d_total;
        vel = 0.0f;
        acc = 0.0f;
        profile->finished = 1;
    }

    // Apply direction
    profile->current_position = pos * profile->direction;
    profile->current_velocity = vel * profile->direction;
    profile->current_acceleration = acc * profile->direction;
}
