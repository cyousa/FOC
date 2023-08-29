/*
 * kalman.h
 *
 *  Created on: 2023年5月1日
 *      Author: hanser
 */

#ifndef CODE_KALMAN_H_
#define CODE_KALMAN_H_
#include "zf_common_headfile.h"
//卡尔曼解算法库

 struct CAR{

        float strff_X;
        float strff_Y;
        float strff_PWM;
        float strff_PWM_GO;
        float speed_l;
        float speed_r;
        uint8 tail[4];

};
extern int16 aacx,aacy,aacz;      //加速度传感器原始数据  angular acceleration
extern int16 gyrox,gyroy,gyroz;       //陀螺仪原始数据  gyroscope
extern float Gyro_x;               //X轴陀螺仪数据暂存
extern float Gyro_y;               //Y轴陀螺仪数据暂存
extern float Gyro_z;               //Z轴陀螺仪数据暂存
extern float Angle_x_temp;
extern float Angle_y_temp;
extern float Q_bias;
void Angle_Calcu(void);
void Kalman_Filter_X(float Accel,float Gyro);
void Kalman_Filter_Y(float Accel,float Gyro);//卡尔曼解算法库

extern float Angle_X_Final;         //解算后俯仰角
extern float Angle_Y_Final;         //解算后横滚角



void Angle_Calcu(void);
void Kalman_Filter_X(float Accel,float Gyro);
void Kalman_Filter_Y(float Accel,float Gyro);
float Complementary_Filter_x(void);


#endif /* CODE_KALMAN_H_ */
