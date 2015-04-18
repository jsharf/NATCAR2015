/*
 * motor.h
 *
 *  Created on: Apr 17, 2015
 *      Author: Kevin
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdint.h>

void motor_init();
void motor_setSpeedi(uint32_t speed);
void motor_setSpeedf(float speed);
uint32_t motor_getMaxSpeed();
void motor_brake();
void motor_releaseBrake();

void servo_setPosf(float pos);
void servo_setPosi(uint32_t pos);

#endif /* MOTOR_H_ */
