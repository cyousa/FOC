/*
 * kalman.h
 *
 *  Created on: 2023��5��1��
 *      Author: hanser
 */

#ifndef CODE_KALMAN_H_
#define CODE_KALMAN_H_
#include "zf_common_headfile.h"
//���������㷨��

 struct CAR{

        float strff_X;
        float strff_Y;
        float strff_PWM;
        float strff_PWM_GO;
        float speed_l;
        float speed_r;
        uint8 tail[4];

};
extern int16 aacx,aacy,aacz;      //���ٶȴ�����ԭʼ����  angular acceleration
extern int16 gyrox,gyroy,gyroz;       //������ԭʼ����  gyroscope
extern float Gyro_x;               //X�������������ݴ�
extern float Gyro_y;               //Y�������������ݴ�
extern float Gyro_z;               //Z�������������ݴ�
extern float Angle_x_temp;
extern float Angle_y_temp;
extern float Q_bias;
void Angle_Calcu(void);
void Kalman_Filter_X(float Accel,float Gyro);
void Kalman_Filter_Y(float Accel,float Gyro);//���������㷨��

extern float Angle_X_Final;         //���������
extern float Angle_Y_Final;         //���������



void Angle_Calcu(void);
void Kalman_Filter_X(float Accel,float Gyro);
void Kalman_Filter_Y(float Accel,float Gyro);
float Complementary_Filter_x(void);


#endif /* CODE_KALMAN_H_ */
