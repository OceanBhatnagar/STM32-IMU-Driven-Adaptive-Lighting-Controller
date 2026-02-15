/*
 * kalman_filter.h
 *
 *  Created on: 12-Feb-2026
 *      Author: Ocean
 */

#ifndef BSP_INC_KALMAN_FILTER_H_
#define BSP_INC_KALMAN_FILTER_H_


typedef struct {
    float q_angle;
    float r_measure;
    float angle;
    float p[2][2];
} KalmanFilter;

void kalman_filter_init(KalmanFilter* kf) ;
float kalman_filter_get_angle(KalmanFilter* kf, float new_angle, float dt);


#endif /* BSP_INC_KALMAN_FILTER_H_ */
