/*
 * motor.h
 *
 *  Created on: 2023Äê2ÔÂ3ÈÕ
 *      Author: Roronoa zoro
 */

#ifndef CODE_MOTOR_H_
#define CODE_MOTOR_H_
#include "zf_common_headfile.h"
void motor_init(void);
void forward_pwm(int duty);
void Leftwheel_pwm(int duty);
void Rightwheel_pwm(int duty);
void shache(void);
extern int16 encoder_data;
extern int16 PWM_Balance,PWM_Velocity;
extern uint16 MOTOR_OUT_DEAD;
#endif /* CODE_MOTOR_H_ */
