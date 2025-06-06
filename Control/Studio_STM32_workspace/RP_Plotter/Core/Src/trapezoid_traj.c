/*
 * trapezoid_traj.c
 *
 *  Created on: Jun 3, 2025
 *      Author: pongn
 */

#include "trapezoid_traj.h"
#include <math.h>

void Trapezoid_Init(TrapezoidProfile* traj, float total_distance, float v_max, float a_max) {
    traj->total_distance = total_distance;
    traj->v_max = v_max;
    traj->a_max = a_max;

    traj->t_acc = v_max / a_max;
    traj->d_acc = 0.5f * a_max * traj->t_acc * traj->t_acc;

    if (2 * traj->d_acc > total_distance) {
        // triangular profile (no flat section)
        traj->t_acc = sqrtf(total_distance / a_max);
        traj->d_acc = 0.5f * a_max * traj->t_acc * traj->t_acc;
        traj->t_flat = 0;
        traj->t_total = 2 * traj->t_acc;
    } else {
        float d_flat = total_distance - 2 * traj->d_acc;
        traj->t_flat = d_flat / v_max;
        traj->t_total = 2 * traj->t_acc + traj->t_flat;
    }
}

float Trapezoid_Update(TrapezoidProfile* traj, float t) {
    float t1 = traj->t_acc;
    float t2 = t1 + traj->t_flat;
    float a = traj->a_max;
    float v = traj->v_max;

    if (t < 0)
        return 0;
    else if (t < t1)
        return 0.5f * a * t * t;
    else if (t < t2)
        return traj->d_acc + v * (t - t1);
    else if (t < traj->t_total) {
        float td = t - t2;
        return traj->d_acc + v * traj->t_flat + v * td - 0.5f * a * td * td;
    } else
        return traj->total_distance;
}

