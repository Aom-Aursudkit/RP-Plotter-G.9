/*
 * Trapezoidal.h
 *
 *  Created on: May 14, 2025
 *      Author: jirat
 */

#ifndef INC_TRAPEZOIDAL_H_
#define INC_TRAPEZOIDAL_H_

typedef struct {
	float distance_total;    // Total distance to move
	float v_max;             // Maximum velocity
	float v_peak;   // Peak velocity (may be less than v_max in triangular mode)
	float a_max;             // Maximum acceleration
	float current_position;  // Current position
	float current_velocity;  // Current velocity
	float target_position;   // Final target position
	float direction;         // +1 or -1
	int finished;            // 1 if finished
	int is_triangular;       // 1 if triangular profile, 0 if full trapezoidal

} VELO_PROFILE;

void Trapezoidal_Init(VELO_PROFILE *profile, float distance_total, float v_max, float a_max);
void Trapezoidal_Update(VELO_PROFILE *profile, float dt);

#endif /* INC_TRAPEZOIDAL_H_ */
