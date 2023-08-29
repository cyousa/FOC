/*
 * pid.h
 *
 *  Created on: 2023年1月24日
 *      Author: Roronoa zoro
 */

#ifndef CODE_PID_H_
#define CODE_PID_H_
#include "zf_common_headfile.h"

typedef struct PID
{
  float P;         //参数
  float I;
  float D;
  float Error;     //比例项e(k)
  float Integral;  //积分项
  float Differ;    //微分项
  float PreError;   //e(k-1)
  float PrePreError;//e(k-2)
  float Ilimit;
  float Irang;
  double Pout;
  double Iout;
  double Dout;
  double OutPut;
  uint8_t Ilimit_flag;    //积分分离
}PID_TYPE;

//速度环
extern PID_TYPE ROL_velocity;
extern PID_TYPE PITCH_velocity;
//角度环
extern PID_TYPE ROL_angle;
extern PID_TYPE PITCH_angle;
//角速度环
extern PID_TYPE ROL_Rate;
extern PID_TYPE PITCH_Rate;
extern PID_TYPE turn;
extern PID_TYPE turn_rate;
extern PID_TYPE turn_velocity;
extern float Encoder;
extern float Turn_pwm;
extern float left_pwm,right_pwm;
extern float Target_Encoder;
extern float Y_Balance[2][3];
void PidParameter_init(void);
void X_suduhuan(int encoder);
void X_jiaosuduhuan(float Gyro);
void X_jiaoduhuan(float Angle);
void Y_suduhuan(int encoder);
void Y_jiaoduhuan(float Angle);
void Y_jiaosuduhuan(float Gyro);
float Y_zhilihuan(float Angle,float Gyro);
float Y_Suduhuan(int encoder);
void Turn(float error);
float Turn_gryo(float gyro);
void Turn_suduhuan(int encoder);
#endif /* CODE_PID_H_ */
