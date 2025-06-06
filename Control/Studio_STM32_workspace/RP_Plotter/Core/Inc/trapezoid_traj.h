/*
 * trapezoid_traj.h
 *
 *  Created on: Jun 3, 2025
 *      Author: pongn
 */

#ifndef TRAPEZOID_TRAJ_H
#define TRAPEZOID_TRAJ_H

typedef struct {
    float total_distance;
    float v_max;
    float a_max;

    float t_acc;   // acceleration time
    float t_flat;  // constant velocity time
    float t_total; // total duration
    float d_acc;   // distance during acceleration
} TrapezoidProfile;

void Trapezoid_Init(TrapezoidProfile* traj, float total_distance, float v_max, float a_max);
float Trapezoid_Update(TrapezoidProfile* traj, float t);

#endif
