/*
 * motion_types.h
 *
 *  Created on: Oct 9, 2025
 *      Author: Geek
 */

#ifndef INC_MOTION_TYPES_H_
#define INC_MOTION_TYPES_H_

typedef struct {
//    float accel[3];
//    float gyro[3];
	int16_t ax;
	int16_t ay;
	int16_t az;
	int16_t gx;
	int16_t gy;
	int16_t gz;
} MotionRaw_T;

#endif /* INC_MOTION_TYPES_H_ */
