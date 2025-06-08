/*
 * all_path.h
 *
 *  Created on: Jun 6, 2025
 *      Author: pongn
 */

#ifndef INC_ALL_PATH_H_
#define INC_ALL_PATH_H_

typedef struct {
    float x;
    float y;
} Point;

extern const Point path_center_cross[];
extern const Point path_f[];
extern const Point path_i[];
extern const Point path_b_1[];
extern const Point path_b_2[];
extern const Point path_b_3[];
extern const Point path_o_1[];
extern const Point path_o_2[];
extern const Point path_underscore[];
extern const Point path_g[];
extern const Point path_0_1[];
extern const Point path_0_2[];
extern const Point path_9_1[];
extern const Point path_9_2[];
extern const int path_center_cross_length;
extern const int path_f_length;
extern const int path_i_length;
extern const int path_b_1_length;
extern const int path_b_2_length;
extern const int path_b_3_length;
extern const int path_o_1_length;
extern const int path_o_2_length;
extern const int path_underscore_length;
extern const int path_g_length;
extern const int path_0_1_length;
extern const int path_0_2_length;
extern const int path_9_1_length;
extern const int path_9_2_length;

extern const Point* paths[];
extern const int path_lengths[];

#endif /* INC_ALL_PATH_H_ */
