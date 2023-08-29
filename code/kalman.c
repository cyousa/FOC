/*
 * kalman.c
 *
 *  Created on: 2023��1��28��
 *      Author: Roronoa zoro
 */
//20ms ����һ��,����5ms��ȡһ��
#include "kalman.h"
//���������㷨��
#define PI 3.14159265358979323846
int16 aacx,aacy,aacz;       //���ٶȴ�����ԭʼ����
int16 gyrox,gyroy,gyroz;    //������ԭʼ����
int16 temperature;          //�������¶�����
float Accel_x;              //X����ٶ�ֵ�ݴ�
float Accel_y;              //Y����ٶ�ֵ�ݴ�
float Accel_z;              //Z����ٶ�ֵ�ݴ�
float Gyro_x;               //X�������������ݴ�
float Gyro_y;               //Y�������������ݴ�
float Gyro_z;               //Z�������������ݴ�
float Angle_x_temp;         //�ɼ��ٶȼ����x��б�Ƕ�
float Angle_y_temp;         //�ɼ��ٶȼ����y��б�Ƕ�
 float Angle_z_temp;
float Angle_X_Final;        //X������б�Ƕ�
float Angle_Y_Final;        //Y������б�Ƕ�
float kermanX;
float bias;
//char tail[4]={0x00,0x00,0x80,0x7f};
struct CAR JD;
float acc_ratio  = 1.2;                                 //���ٶȼƱ���
float gyro_ratio = 1.00;                               //�����Ǳ���
float temp_angle;

//x�ụ���˲�
float x_angle_calc(float angle_m, float gyro_m)
{
    float gyro_now;
    float error_angle;
    static float x_last_angle;
    static uint8 x_first_angle=0;

    if(!x_first_angle)//�ж��Ƿ�Ϊ��һ�����б�����
    {
        //����ǵ�һ�����У����ϴνǶ�ֵ����Ϊ����ٶ�ֵһ��
        x_first_angle = 1;
        x_last_angle = angle_m;
    }
    gyro_now = gyro_m * gyro_ratio;
    //���ݲ������ļ��ٶ�ֵת��Ϊ�Ƕ�֮�����ϴεĽǶ�ֵ��ƫ��
    error_angle = (angle_m - x_last_angle)*acc_ratio;
//    //����ƫ���������ǲ����õ��ĽǶ�ֵ���㵱ǰ�Ƕ�ֵ
    temp_angle = x_last_angle + (error_angle + gyro_now)*0.002;
//    //���浱ǰ�Ƕ�ֵ
    x_last_angle = temp_angle;
    return temp_angle;
}
//y�ụ���˲�
float y_angle_calc(float angle_m, float gyro_m)
{
    float gyro_now;
    float error_angle;
    static float y_last_angle;
    static uint8 y_first_angle=0;

    if(!y_first_angle)//�ж��Ƿ�Ϊ��һ�����б�����
    {
        //����ǵ�һ�����У����ϴνǶ�ֵ����Ϊ����ٶ�ֵһ��
        y_first_angle = 1;
        y_last_angle = angle_m;
    }
    gyro_now = gyro_m * gyro_ratio;
    //���ݲ������ļ��ٶ�ֵת��Ϊ�Ƕ�֮�����ϴεĽǶ�ֵ��ƫ��
    error_angle = (angle_m - y_last_angle)*acc_ratio;
//    //����ƫ���������ǲ����õ��ĽǶ�ֵ���㵱ǰ�Ƕ�ֵ
    temp_angle = y_last_angle + (error_angle + gyro_now)*0.002;
//    //���浱ǰ�Ƕ�ֵ
    y_last_angle = temp_angle;
    return temp_angle;
}


void UDP_send(unsigned char *dat, unsigned char len)

{
    //���len������λ
    //static char temp_num[4] = {'0', '0', '\r', '\n'};//ʮλ���ִ�С ��λ���ִ�С
    unsigned char index = 0;

    if(dat && len > 0)
    {

              do
              {
                  uart_write_byte(UART_2, dat[index]);//���ͳ�������ʱdat�ǽṹ��ָ��
                  index++;
             }while(len--);


    }
}

//void Kalman_Filter_X(float Accel,float Gyro);//����������
//void Kalman_Filter_Y(float Accel,float Gyro);
float dt=0.004;       //ÿ5ms����һ���˲�
float Kalman_Filter_x(float Accel,float Gyro)
{
    static float angle,angle_dot;
    float Q_angle=0.001; // ����������Э����
    float Q_gyro=0.003; //0.003 ����������Э���� ����������Э����Ϊһ��һ�����о���
    float R_angle=0.5;      // ����������Э���� �Ȳ���ƫ��
    char  C_0 = 1;
    static float Q_bias, Angle_err;
    static float PCt_0, PCt_1, E;
    static float K_0, K_1, t_0, t_1;
    static float Pdot[4] ={0,0,0,0};
    static float PP[2][2] = { { 1, 0 },{ 0, 1 } };
    angle+=(Gyro - Q_bias) * dt; //�������
    Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-����������Э�����΢��

    Pdot[1]=-PP[1][1];
    Pdot[2]=-PP[1][1];
    Pdot[3]=Q_gyro;
    PP[0][0] += Pdot[0] * dt;   // Pk-����������Э����΢�ֵĻ���
    PP[0][1] += Pdot[1] * dt;   // =����������Э����
    PP[1][0] += Pdot[2] * dt;
    PP[1][1] += Pdot[3] * dt;

    Angle_err = Accel - angle;  //zk-�������

    PCt_0 = C_0 * PP[0][0];
    PCt_1 = C_0 * PP[1][0];

    E = R_angle + C_0 * PCt_0;

    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;

    t_0 = PCt_0;
    t_1 = C_0 * PP[0][1];

    PP[0][0] -= K_0 * t_0;       //����������Э����
    PP[0][1] -= K_0 * t_1;
    PP[1][0] -= K_1 * t_0;
    PP[1][1] -= K_1 * t_1;

    angle   += K_0 * Angle_err;  //�������
    Q_bias  += K_1 * Angle_err;  //�������
    angle_dot   = Gyro - Q_bias;     //���ֵ(�������)��΢��=���ٶ�
   // Gyro_x=angle_dot;
    return angle;
}
float Kalman_Filter_y(float Accel,float Gyro)
{
    static float angle_dot;
    static float angle;
    float Q_angle=0.001; // ����������Э����
    float Q_gyro=0.003; //0.003 ����������Э���� ����������Э����Ϊһ��һ�����о���
    float R_angle=0.0005;      // ����������Э���� �Ȳ���ƫ��
    char  C_0 = 1;
    static float Q_bias, Angle_err;
    static float PCt_0, PCt_1, E;
    static float K_0, K_1, t_0, t_1;
    static float Pdot[4] ={0,0,0,0};
    static float PP[2][2] = { { 1, 0 },{ 0, 1 } };
    angle+=(Gyro - Q_bias) * dt; //�������
    Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-����������Э�����΢��
    Pdot[1]=-PP[1][1];
    Pdot[2]=-PP[1][1];
    Pdot[3]=Q_gyro;
    PP[0][0] += Pdot[0] * dt;   // Pk-����������Э����΢�ֵĻ���
    PP[0][1] += Pdot[1] * dt;   // =����������Э����
    PP[1][0] += Pdot[2] * dt;
    PP[1][1] += Pdot[3] * dt;
    Angle_err = Accel - angle;  //zk-�������

    PCt_0 = C_0 * PP[0][0];
    PCt_1 = C_0 * PP[1][0];

    E = R_angle + C_0 * PCt_0;

    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;

    t_0 = PCt_0;
    t_1 = C_0 * PP[0][1];

    PP[0][0] -= K_0 * t_0;       //����������Э����
    PP[0][1] -= K_0 * t_1;
    PP[1][0] -= K_1 * t_0;
    PP[1][1] -= K_1 * t_1;

    angle   += K_0 * Angle_err;    //�������
    Q_bias  += K_1 * Angle_err;  //�������
    angle_dot   = Gyro - Q_bias;    //���ֵ(�������)��΢��=���ٶ�
    //Gyro_y=angle_dot;
    return angle;
}
float Complementary_Filter_y(float angle_m, float gyro_m)
{
     static float angle;
     float K1 =0.02;
     angle = K1 * angle_m+ (1-K1) * (angle + (gyro_m-0.2) * dt);
     return angle;
}
void Angle_Calcu(void)
{

    //Gyro_x=mpu6050_gyro_x;
    //Gyro_y=mpu6050_gyro_y;
    //Gyro_z=mpu6050_gyro_z;
    //Accel_x=mpu6050_acc_x;
   // Accel_y=mpu6050_acc_y;
    //Accel_z=mpu6050_acc_z;
    Gyro_x=imu660ra_gyro_x;
    Gyro_y=imu660ra_gyro_y;
    Gyro_z=imu660ra_gyro_z;
    Accel_x=imu660ra_acc_x;
    Accel_y=imu660ra_acc_y;
    Accel_z=imu660ra_acc_z;
    if(Gyro_x>32768)  Gyro_x-=65536;                 //��������ת��  Ҳ��ͨ��shortǿ������ת��
    if(Gyro_y>32768)  Gyro_y-=65536;                 //��������ת��  Ҳ��ͨ��shortǿ������ת��
    if(Gyro_z>32768)  Gyro_z-=65536;                 //��������ת��
    if(Accel_x>32768) Accel_x-=65536;                //��������ת��
    if(Accel_y>32768) Accel_y-=65536;                //��������ת��
    if(Accel_z>32768) Accel_z-=65536;                //��������ת��
    Accel_x=imu660ra_acc_transition(imu660ra_acc_x);
    Accel_y=imu660ra_acc_transition(imu660ra_acc_y);
    Accel_z=imu660ra_acc_transition(imu660ra_acc_z);
    Gyro_x=imu660ra_gyro_transition(imu660ra_gyro_x);
    Gyro_y=imu660ra_gyro_transition(imu660ra_gyro_y);
    Gyro_z=imu660ra_gyro_transition(imu660ra_gyro_z);
    Angle_z_temp+=(Gyro_z+0.1)*0.002;

    Angle_x_temp=atan2(Accel_y,Accel_z)*180/PI;     //������ǣ�ת����λΪ��
    Angle_y_temp=atan2(Accel_x,Accel_z)*180/PI;     //������ǣ�ת����λΪ��

    Angle_X_Final=x_angle_calc(Angle_x_temp, Gyro_x);
 //   Angle_X_Final=Kalman_Filter_x(Angle_x_temp,Gyro_x);//�������˲� �õ����սǶ�
    Angle_Y_Final= y_angle_calc( Angle_y_temp, -Gyro_y);
    //Angle_Y_Final=Kalman_Filter_y(Angle_y_temp,-Gyro_y);


//    Angle_X_Final=Complementary_Filter_y(Angle_x_temp,Gyro_x);
}
//��ȡ����Ԥ����
//void Angle_Calcu(void)
//{
//    //1.ԭʼ���ݶ�ȡ
//    float accx,accy,accz;//������Ǽ��ٶ�ֵ
//    Accel_x = mpu6050_acc_x;//x����ٶ�ֵ�ݴ�
//    Accel_y = mpu6050_acc_y;//y����ٶ�ֵ�ݴ�
//    Accel_z = mpu6050_acc_z;//z����ٶ�ֵ�ݴ�
//    Gyro_x  = mpu6050_gyro_x;//x��������ֵ�ݴ�
//    Gyro_y  = mpu6050_gyro_y;//y��������ֵ�ݴ�
//    Gyro_z  = mpu6050_gyro_z;//z��������ֵ�ݴ�
//
//    //2.�Ǽ��ٶ�ԭʼֵ�������
//    //���ٶȴ��������üĴ���0X1C��д��0x01,���÷�ΧΪ��2g�������ϵ��2^16/4 = 16384LSB/g
//    if(Accel_x<32764) accx=Accel_x/16384;//����x����ٶ�
//    else              accx=1-(Accel_x-49152)/16384;
//    if(Accel_y<32764) accy=Accel_y/16384;//����y����ٶ�
//    else              accy=1-(Accel_y-49152)/16384;
//    if(Accel_z<32764) accz=Accel_z/16384;//����z����ٶ�
//    else              accz=(Accel_z-49152)/16384;
//    //���ٶȷ����й�ʽ�����������ˮƽ������ϵ֮��ļн�
//    Angle_x_temp=(atan2(accy,accz))*180/3.14;
//    Angle_y_temp=(atan2(accx,accz))*180/3.14;
//    //�жϼ����Ƕȵ�������
//    if(Accel_x<32764) Angle_y_temp = +Angle_y_temp;
//    if(Accel_x>32764) Angle_y_temp = -Angle_y_temp;
//    if(Accel_y<32764) Angle_x_temp = +Angle_x_temp;
//    if(Accel_y>32764) Angle_x_temp = -Angle_x_temp;
//
//    //3.���ٶ�ԭʼֵ�������
//    //���������üĴ���0X1B��д��0x18�����÷�ΧΪ��2000deg/s�������ϵ��2^16/4000=16.4LSB/(��/S)
//    ////������ٶ�
//    if(Gyro_x<32768) Gyro_x=-(Gyro_x/16.4);
//    if(Gyro_x>32768) Gyro_x=+(65535-Gyro_x)/16.4;
//    if(Gyro_y<32768) Gyro_y=-(Gyro_y/16.4);
//    if(Gyro_y>32768) Gyro_y=+(65535-Gyro_y)/16.4;
//    if(Gyro_z<32768) Gyro_z=-(Gyro_z/16.4);
//    if(Gyro_z>32768) Gyro_z=+(65535-Gyro_z)/16.4;
//
//    //4.���ÿ���������
//    (Angle_x_temp,-Gyro_x);  //�������˲�����X���
//    Kalman_Filter_Y(Angle_y_temp,Gyro_y);  //�������˲�����Y���
//}

////����������
//float Q_angle = 0.001;      //�Ƕ��������Ŷȣ��Ƕ�������Э����
//float Q_gyro  = 0.003;      //���ٶ��������Ŷȣ����ٶ�������Э����
//float R_angle = 0.5;       //���ٶȼƲ���������Э����
//float dt      = 0.005;       //�˲��㷨�������ڣ��ɶ�ʱ����ʱ5ms
//char  C_0     = 1;          //H����ֵ
//float Q_bias, Angle_err;    //Q_bias:�����ǵ�ƫ��  Angle_err:�Ƕ�ƫ��
//float PCt_0, PCt_1, E;      //����Ĺ�����
//float K_0, K_1, t_0, t_1;   //����������  K_0:���ڼ������Ź���ֵ  K_1:���ڼ������Ź���ֵ��ƫ�� t_0/1:�м����
//float P[4] ={0,0,0,0};  //����Э��������΢�־����м����
//float PP[2][2] = { { 1, 0 },{ 0, 1 } };//����Э�������P

//void Kalman_Filter_X(float Accel,float Gyro) //����������
//{
//    //����һ���������
//    //��ʽ��X(k|k-1) = AX(k-1|k-1) + BU(k)
//    //X = (Angle,Q_bias)
//    //A(1,1) = 1,A(1,2) = -dt
//    //A(2,1) = 0,A(2,2) = 1
//    Angle_X_Final += (Gyro - Q_bias) * dt; //״̬����,�Ƕȹ���ֵ=��һ�νǶȹ���ֵ+(��ǰ���ٶ�-���ٶ�Ư��)*dt
//
//    //��������������Э��������΢�־���
//    //��ʽ��P(k|k-1)=AP(k-1|k-1)A^T + Q
//    //Q(1,1) = cov(Angle,Angle) Q(1,2) = cov(Q_bias,Angle)
//    //Q(2,1) = cov(Angle,Q_bias)    Q(2,2) = cov(Q_bias,Q_bias)
//    P[0]= Q_angle - PP[0][1] - PP[1][0];
//    P[1]= -PP[1][1];// ����������Э����
//    P[2]= -PP[1][1];
//    P[3]= Q_gyro;
//    PP[0][0] += P[0] * dt;
//    PP[0][1] += P[1] * dt;
//    PP[1][0] += P[2] * dt;
//    PP[1][1] += P[3] * dt;
//    Angle_err = Accel - Angle_X_Final;
//    //�����������㿨��������
//    //��ʽ��Kg(k)= P(k|k-1)H^T/(HP(k|k-1)H^T+R)
//    //Kg = (K_0,K_1) ��ӦAngle,Q_bias����
//    //H = (1,0) ����z=HX+v���z:Accel
//    PCt_0 = C_0 * PP[0][0];//C_0����H
//    PCt_1 = C_0 * PP[1][0];
//    E = R_angle + C_0 * PCt_0;
//    K_0 = PCt_0 / E;
//    K_1 = PCt_1 / E;
//
//    //�����ģ�����������Э����
//    //��ʽ��P(k|k)=(I-Kg(k)H)P(k|k-1)
//    //Ҳ��дΪ��P(k|k)=P(k|k-1)-Kg(k)HP(k|k-1)
//    t_0 = PCt_0;
//    t_1 = C_0 * PP[0][1];
//    PP[0][0] -= K_0 * t_0;
//    PP[0][1] -= K_0 * t_1;
//    PP[1][0] -= K_1 * t_0;
//    PP[1][1] -= K_1 * t_1;
//
//    //�����壬�������Ž��ٶ�ֵ
//    //��ʽ��X(k|k)= X(k|k-1)+Kg(k)(Z(k)-X(k|k-1))
//      //Z(k)������� ����Ƕ�ƫ��
//    Angle_X_Final += K_0 * Angle_err;    //������ƣ��������Ź���ֵ
//    Q_bias        += K_1 * Angle_err;    //������ƣ��������Ź���ֵƫ��
//    Gyro_x         = Gyro - Q_bias;
//}
//
//void Kalman_Filter_Y(float Accel,float Gyro)
//{
//    Angle_Y_Final += (Gyro - Q_bias) * dt;
//    P[0]=Q_angle - PP[0][1] - PP[1][0];
//    P[1]=-PP[1][1];
//    P[2]=-PP[1][1];
//    P[3]=Q_gyro;
//    PP[0][0] += P[0] * dt;
//    PP[0][1] += P[1] * dt;
//    PP[1][0] += P[2] * dt;
//    PP[1][1] += P[3] * dt;
//    Angle_err = Accel - Angle_Y_Final;
//    PCt_0 = C_0 * PP[0][0];
//    PCt_1 = C_0 * PP[1][0];
//    E = R_angle + C_0 * PCt_0;
//    K_0 = PCt_0 / E;
//    K_1 = PCt_1 / E;
//    t_0 = PCt_0;
//    t_1 = C_0 * PP[0][1];
//    PP[0][0] -= K_0 * t_0;
//    PP[0][1] -= K_0 * t_1;
//    PP[1][0] -= K_1 * t_0;
//    PP[1][1] -= K_1 * t_1;
//    Angle_Y_Final   += K_0 * Angle_err;
//    Q_bias  += K_1 * Angle_err;
//    Gyro_y   = Gyro - Q_bias;
//}

/**************************************************************************
Function: First order complementary filtering
Input   : acceleration��angular velocity
Output  : none
�������ܣ�һ�׻����˲�
��ڲ��������ٶȻ�ȡ�ĽǶȡ����ٶ�
����  ֵ��x����ٶ�
**************************************************************************/
float Complementary_Filter_x(void)
{
     static float angle;
     float K1 =0.1;
     //1.ԭʼ���ݶ�ȡ
     float accx,accy,accz;//������Ǽ��ٶ�ֵ
     Accel_x = imu660ra_acc_x;    //x����ٶ�ֵ�ݴ�
     Accel_y = imu660ra_acc_y;    //y����ٶ�ֵ�ݴ�
     Accel_z = imu660ra_acc_z;    //z����ٶ�ֵ�ݴ�
     Gyro_x  = imu660ra_gyro_x;  //x��������ֵ�ݴ�
     Gyro_y  = imu660ra_gyro_y;  //y��������ֵ�ݴ�
     Gyro_z  = imu660ra_gyro_z;  //z��������ֵ�ݴ�
     //2.�Ǽ��ٶ�ԭʼֵ�������
     //���ٶȴ��������üĴ���0X1C��д��0x01,���÷�ΧΪ��2g�������ϵ��2^16/4 = 16384LSB/g
     if(Accel_x<32764) accx=Accel_x/16384;//����x����ٶ�
     else              accx=1-(Accel_x-49152)/16384;
     if(Accel_y<32764) accy=Accel_y/16384;//����y����ٶ�
     else              accy=1-(Accel_y-49152)/16384;
     if(Accel_z<32764) accz=Accel_z/16384;//����z����ٶ�
     else              accz=(Accel_z-49152)/16384;
     //���ٶȷ����й�ʽ�����������ˮƽ������ϵ֮��ļн�
     Angle_x_temp=(atan(accy/accz))*180/3.14;
     Angle_y_temp=(atan(accx/accz))*180/3.14;
     //�жϼ����Ƕȵ�������
     if(Accel_x<32764) Angle_y_temp = +Angle_y_temp;
     if(Accel_x>32764) Angle_y_temp = -Angle_y_temp;
     if(Accel_y<32764) Angle_x_temp = +Angle_x_temp;
     if(Accel_y>32764) Angle_x_temp = -Angle_x_temp;

     //3.���ٶ�ԭʼֵ�������
     //���������üĴ���0X1B��д��0x18�����÷�ΧΪ��2000deg/s�������ϵ��2^16/4000=16.4LSB/(��/S)
     ////������ٶ�
     if(Gyro_x<32768) Gyro_x=-(Gyro_x/16.4);
     if(Gyro_x>32768) Gyro_x=+(65535-Gyro_x)/16.4;
     if(Gyro_y<32768) Gyro_y=-(Gyro_y/16.4);
     if(Gyro_y>32768) Gyro_y=+(65535-Gyro_y)/16.4;
     if(Gyro_z<32768) Gyro_z=-(Gyro_z/16.4);
     if(Gyro_z>32768) Gyro_z=+(65535-Gyro_z)/16.4;
     angle = K1 * Angle_x_temp+ (1-K1) * (angle + (Gyro_x-0.46)* dt);
     return angle;
}

