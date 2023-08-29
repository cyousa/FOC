/*
 * kalman.c
 *
 *  Created on: 2023年1月28日
 *      Author: Roronoa zoro
 */
//20ms 解算一次,数据5ms读取一次
#include "kalman.h"
//卡尔曼解算法库
#define PI 3.14159265358979323846
int16 aacx,aacy,aacz;       //加速度传感器原始数据
int16 gyrox,gyroy,gyroz;    //陀螺仪原始数据
int16 temperature;          //陀螺仪温度数据
float Accel_x;              //X轴加速度值暂存
float Accel_y;              //Y轴加速度值暂存
float Accel_z;              //Z轴加速度值暂存
float Gyro_x;               //X轴陀螺仪数据暂存
float Gyro_y;               //Y轴陀螺仪数据暂存
float Gyro_z;               //Z轴陀螺仪数据暂存
float Angle_x_temp;         //由加速度计算的x倾斜角度
float Angle_y_temp;         //由加速度计算的y倾斜角度
 float Angle_z_temp;
float Angle_X_Final;        //X最终倾斜角度
float Angle_Y_Final;        //Y最终倾斜角度
float kermanX;
float bias;
//char tail[4]={0x00,0x00,0x80,0x7f};
struct CAR JD;
float acc_ratio  = 1.2;                                 //加速度计比例
float gyro_ratio = 1.00;                               //陀螺仪比例
float temp_angle;

//x轴互补滤波
float x_angle_calc(float angle_m, float gyro_m)
{
    float gyro_now;
    float error_angle;
    static float x_last_angle;
    static uint8 x_first_angle=0;

    if(!x_first_angle)//判断是否为第一次运行本函数
    {
        //如果是第一次运行，则将上次角度值设置为与加速度值一致
        x_first_angle = 1;
        x_last_angle = angle_m;
    }
    gyro_now = gyro_m * gyro_ratio;
    //根据测量到的加速度值转换为角度之后与上次的角度值求偏差
    error_angle = (angle_m - x_last_angle)*acc_ratio;
//    //根据偏差与陀螺仪测量得到的角度值计算当前角度值
    temp_angle = x_last_angle + (error_angle + gyro_now)*0.002;
//    //保存当前角度值
    x_last_angle = temp_angle;
    return temp_angle;
}
//y轴互补滤波
float y_angle_calc(float angle_m, float gyro_m)
{
    float gyro_now;
    float error_angle;
    static float y_last_angle;
    static uint8 y_first_angle=0;

    if(!y_first_angle)//判断是否为第一次运行本函数
    {
        //如果是第一次运行，则将上次角度值设置为与加速度值一致
        y_first_angle = 1;
        y_last_angle = angle_m;
    }
    gyro_now = gyro_m * gyro_ratio;
    //根据测量到的加速度值转换为角度之后与上次的角度值求偏差
    error_angle = (angle_m - y_last_angle)*acc_ratio;
//    //根据偏差与陀螺仪测量得到的角度值计算当前角度值
    temp_angle = y_last_angle + (error_angle + gyro_now)*0.002;
//    //保存当前角度值
    y_last_angle = temp_angle;
    return temp_angle;
}


void UDP_send(unsigned char *dat, unsigned char len)

{
    //最大len限制两位
    //static char temp_num[4] = {'0', '0', '\r', '\n'};//十位数字大小 个位数字大小
    unsigned char index = 0;

    if(dat && len > 0)
    {

              do
              {
                  uart_write_byte(UART_2, dat[index]);//发送车体数据时dat是结构体指针
                  index++;
             }while(len--);


    }
}

//void Kalman_Filter_X(float Accel,float Gyro);//卡尔曼函数
//void Kalman_Filter_Y(float Accel,float Gyro);
float dt=0.004;       //每5ms进行一次滤波
float Kalman_Filter_x(float Accel,float Gyro)
{
    static float angle,angle_dot;
    float Q_angle=0.001; // 过程噪声的协方差
    float Q_gyro=0.003; //0.003 过程噪声的协方差 过程噪声的协方差为一个一行两列矩阵
    float R_angle=0.5;      // 测量噪声的协方差 既测量偏差
    char  C_0 = 1;
    static float Q_bias, Angle_err;
    static float PCt_0, PCt_1, E;
    static float K_0, K_1, t_0, t_1;
    static float Pdot[4] ={0,0,0,0};
    static float PP[2][2] = { { 1, 0 },{ 0, 1 } };
    angle+=(Gyro - Q_bias) * dt; //先验估计
    Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分

    Pdot[1]=-PP[1][1];
    Pdot[2]=-PP[1][1];
    Pdot[3]=Q_gyro;
    PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
    PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
    PP[1][0] += Pdot[2] * dt;
    PP[1][1] += Pdot[3] * dt;

    Angle_err = Accel - angle;  //zk-先验估计

    PCt_0 = C_0 * PP[0][0];
    PCt_1 = C_0 * PP[1][0];

    E = R_angle + C_0 * PCt_0;

    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;

    t_0 = PCt_0;
    t_1 = C_0 * PP[0][1];

    PP[0][0] -= K_0 * t_0;       //后验估计误差协方差
    PP[0][1] -= K_0 * t_1;
    PP[1][0] -= K_1 * t_0;
    PP[1][1] -= K_1 * t_1;

    angle   += K_0 * Angle_err;  //后验估计
    Q_bias  += K_1 * Angle_err;  //后验估计
    angle_dot   = Gyro - Q_bias;     //输出值(后验估计)的微分=角速度
   // Gyro_x=angle_dot;
    return angle;
}
float Kalman_Filter_y(float Accel,float Gyro)
{
    static float angle_dot;
    static float angle;
    float Q_angle=0.001; // 过程噪声的协方差
    float Q_gyro=0.003; //0.003 过程噪声的协方差 过程噪声的协方差为一个一行两列矩阵
    float R_angle=0.0005;      // 测量噪声的协方差 既测量偏差
    char  C_0 = 1;
    static float Q_bias, Angle_err;
    static float PCt_0, PCt_1, E;
    static float K_0, K_1, t_0, t_1;
    static float Pdot[4] ={0,0,0,0};
    static float PP[2][2] = { { 1, 0 },{ 0, 1 } };
    angle+=(Gyro - Q_bias) * dt; //先验估计
    Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分
    Pdot[1]=-PP[1][1];
    Pdot[2]=-PP[1][1];
    Pdot[3]=Q_gyro;
    PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
    PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
    PP[1][0] += Pdot[2] * dt;
    PP[1][1] += Pdot[3] * dt;
    Angle_err = Accel - angle;  //zk-先验估计

    PCt_0 = C_0 * PP[0][0];
    PCt_1 = C_0 * PP[1][0];

    E = R_angle + C_0 * PCt_0;

    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;

    t_0 = PCt_0;
    t_1 = C_0 * PP[0][1];

    PP[0][0] -= K_0 * t_0;       //后验估计误差协方差
    PP[0][1] -= K_0 * t_1;
    PP[1][0] -= K_1 * t_0;
    PP[1][1] -= K_1 * t_1;

    angle   += K_0 * Angle_err;    //后验估计
    Q_bias  += K_1 * Angle_err;  //后验估计
    angle_dot   = Gyro - Q_bias;    //输出值(后验估计)的微分=角速度
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
    if(Gyro_x>32768)  Gyro_x-=65536;                 //数据类型转换  也可通过short强制类型转换
    if(Gyro_y>32768)  Gyro_y-=65536;                 //数据类型转换  也可通过short强制类型转换
    if(Gyro_z>32768)  Gyro_z-=65536;                 //数据类型转换
    if(Accel_x>32768) Accel_x-=65536;                //数据类型转换
    if(Accel_y>32768) Accel_y-=65536;                //数据类型转换
    if(Accel_z>32768) Accel_z-=65536;                //数据类型转换
    Accel_x=imu660ra_acc_transition(imu660ra_acc_x);
    Accel_y=imu660ra_acc_transition(imu660ra_acc_y);
    Accel_z=imu660ra_acc_transition(imu660ra_acc_z);
    Gyro_x=imu660ra_gyro_transition(imu660ra_gyro_x);
    Gyro_y=imu660ra_gyro_transition(imu660ra_gyro_y);
    Gyro_z=imu660ra_gyro_transition(imu660ra_gyro_z);
    Angle_z_temp+=(Gyro_z+0.1)*0.002;

    Angle_x_temp=atan2(Accel_y,Accel_z)*180/PI;     //计算倾角，转换单位为度
    Angle_y_temp=atan2(Accel_x,Accel_z)*180/PI;     //计算倾角，转换单位为度

    Angle_X_Final=x_angle_calc(Angle_x_temp, Gyro_x);
 //   Angle_X_Final=Kalman_Filter_x(Angle_x_temp,Gyro_x);//卡尔曼滤波 得到最终角度
    Angle_Y_Final= y_angle_calc( Angle_y_temp, -Gyro_y);
    //Angle_Y_Final=Kalman_Filter_y(Angle_y_temp,-Gyro_y);


//    Angle_X_Final=Complementary_Filter_y(Angle_x_temp,Gyro_x);
}
//读取数据预处理
//void Angle_Calcu(void)
//{
//    //1.原始数据读取
//    float accx,accy,accz;//三方向角加速度值
//    Accel_x = mpu6050_acc_x;//x轴加速度值暂存
//    Accel_y = mpu6050_acc_y;//y轴加速度值暂存
//    Accel_z = mpu6050_acc_z;//z轴加速度值暂存
//    Gyro_x  = mpu6050_gyro_x;//x轴陀螺仪值暂存
//    Gyro_y  = mpu6050_gyro_y;//y轴陀螺仪值暂存
//    Gyro_z  = mpu6050_gyro_z;//z轴陀螺仪值暂存
//
//    //2.角加速度原始值处理过程
//    //加速度传感器配置寄存器0X1C内写入0x01,设置范围为±2g。换算关系：2^16/4 = 16384LSB/g
//    if(Accel_x<32764) accx=Accel_x/16384;//计算x轴加速度
//    else              accx=1-(Accel_x-49152)/16384;
//    if(Accel_y<32764) accy=Accel_y/16384;//计算y轴加速度
//    else              accy=1-(Accel_y-49152)/16384;
//    if(Accel_z<32764) accz=Accel_z/16384;//计算z轴加速度
//    else              accz=(Accel_z-49152)/16384;
//    //加速度反正切公式计算三个轴和水平面坐标系之间的夹角
//    Angle_x_temp=(atan2(accy,accz))*180/3.14;
//    Angle_y_temp=(atan2(accx,accz))*180/3.14;
//    //判断计算后角度的正负号
//    if(Accel_x<32764) Angle_y_temp = +Angle_y_temp;
//    if(Accel_x>32764) Angle_y_temp = -Angle_y_temp;
//    if(Accel_y<32764) Angle_x_temp = +Angle_x_temp;
//    if(Accel_y>32764) Angle_x_temp = -Angle_x_temp;
//
//    //3.角速度原始值处理过程
//    //陀螺仪配置寄存器0X1B内写入0x18，设置范围为±2000deg/s。换算关系：2^16/4000=16.4LSB/(°/S)
//    ////计算角速度
//    if(Gyro_x<32768) Gyro_x=-(Gyro_x/16.4);
//    if(Gyro_x>32768) Gyro_x=+(65535-Gyro_x)/16.4;
//    if(Gyro_y<32768) Gyro_y=-(Gyro_y/16.4);
//    if(Gyro_y>32768) Gyro_y=+(65535-Gyro_y)/16.4;
//    if(Gyro_z<32768) Gyro_z=-(Gyro_z/16.4);
//    if(Gyro_z>32768) Gyro_z=+(65535-Gyro_z)/16.4;
//
//    //4.调用卡尔曼函数
//    (Angle_x_temp,-Gyro_x);  //卡尔曼滤波计算X倾角
//    Kalman_Filter_Y(Angle_y_temp,Gyro_y);  //卡尔曼滤波计算Y倾角
//}

////卡尔曼参数
//float Q_angle = 0.001;      //角度数据置信度，角度噪声的协方差
//float Q_gyro  = 0.003;      //角速度数据置信度，角速度噪声的协方差
//float R_angle = 0.5;       //加速度计测量噪声的协方差
//float dt      = 0.005;       //滤波算法计算周期，由定时器定时5ms
//char  C_0     = 1;          //H矩阵值
//float Q_bias, Angle_err;    //Q_bias:陀螺仪的偏差  Angle_err:角度偏量
//float PCt_0, PCt_1, E;      //计算的过程量
//float K_0, K_1, t_0, t_1;   //卡尔曼增益  K_0:用于计算最优估计值  K_1:用于计算最优估计值的偏差 t_0/1:中间变量
//float P[4] ={0,0,0,0};  //过程协方差矩阵的微分矩阵，中间变量
//float PP[2][2] = { { 1, 0 },{ 0, 1 } };//过程协方差矩阵P

//void Kalman_Filter_X(float Accel,float Gyro) //卡尔曼函数
//{
//    //步骤一，先验估计
//    //公式：X(k|k-1) = AX(k-1|k-1) + BU(k)
//    //X = (Angle,Q_bias)
//    //A(1,1) = 1,A(1,2) = -dt
//    //A(2,1) = 0,A(2,2) = 1
//    Angle_X_Final += (Gyro - Q_bias) * dt; //状态方程,角度估计值=上一次角度估计值+(当前角速度-角速度漂移)*dt
//
//    //步骤二，计算过程协方差矩阵的微分矩阵
//    //公式：P(k|k-1)=AP(k-1|k-1)A^T + Q
//    //Q(1,1) = cov(Angle,Angle) Q(1,2) = cov(Q_bias,Angle)
//    //Q(2,1) = cov(Angle,Q_bias)    Q(2,2) = cov(Q_bias,Q_bias)
//    P[0]= Q_angle - PP[0][1] - PP[1][0];
//    P[1]= -PP[1][1];// 先验估计误差协方差
//    P[2]= -PP[1][1];
//    P[3]= Q_gyro;
//    PP[0][0] += P[0] * dt;
//    PP[0][1] += P[1] * dt;
//    PP[1][0] += P[2] * dt;
//    PP[1][1] += P[3] * dt;
//    Angle_err = Accel - Angle_X_Final;
//    //步骤三，计算卡尔曼增益
//    //公式：Kg(k)= P(k|k-1)H^T/(HP(k|k-1)H^T+R)
//    //Kg = (K_0,K_1) 对应Angle,Q_bias增益
//    //H = (1,0) 可由z=HX+v求出z:Accel
//    PCt_0 = C_0 * PP[0][0];//C_0代替H
//    PCt_1 = C_0 * PP[1][0];
//    E = R_angle + C_0 * PCt_0;
//    K_0 = PCt_0 / E;
//    K_1 = PCt_1 / E;
//
//    //步骤四，后验估计误差协方差
//    //公式：P(k|k)=(I-Kg(k)H)P(k|k-1)
//    //也可写为：P(k|k)=P(k|k-1)-Kg(k)HP(k|k-1)
//    t_0 = PCt_0;
//    t_1 = C_0 * PP[0][1];
//    PP[0][0] -= K_0 * t_0;
//    PP[0][1] -= K_0 * t_1;
//    PP[1][0] -= K_1 * t_0;
//    PP[1][1] -= K_1 * t_1;
//
//    //步骤五，计算最优角速度值
//    //公式：X(k|k)= X(k|k-1)+Kg(k)(Z(k)-X(k|k-1))
//      //Z(k)先验估计 计算角度偏差
//    Angle_X_Final += K_0 * Angle_err;    //后验估计，给出最优估计值
//    Q_bias        += K_1 * Angle_err;    //后验估计，跟新最优估计值偏差
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
Input   : acceleration、angular velocity
Output  : none
函数功能：一阶互补滤波
入口参数：加速度获取的角度、角速度
返回  值：x轴角速度
**************************************************************************/
float Complementary_Filter_x(void)
{
     static float angle;
     float K1 =0.1;
     //1.原始数据读取
     float accx,accy,accz;//三方向角加速度值
     Accel_x = imu660ra_acc_x;    //x轴加速度值暂存
     Accel_y = imu660ra_acc_y;    //y轴加速度值暂存
     Accel_z = imu660ra_acc_z;    //z轴加速度值暂存
     Gyro_x  = imu660ra_gyro_x;  //x轴陀螺仪值暂存
     Gyro_y  = imu660ra_gyro_y;  //y轴陀螺仪值暂存
     Gyro_z  = imu660ra_gyro_z;  //z轴陀螺仪值暂存
     //2.角加速度原始值处理过程
     //加速度传感器配置寄存器0X1C内写入0x01,设置范围为±2g。换算关系：2^16/4 = 16384LSB/g
     if(Accel_x<32764) accx=Accel_x/16384;//计算x轴加速度
     else              accx=1-(Accel_x-49152)/16384;
     if(Accel_y<32764) accy=Accel_y/16384;//计算y轴加速度
     else              accy=1-(Accel_y-49152)/16384;
     if(Accel_z<32764) accz=Accel_z/16384;//计算z轴加速度
     else              accz=(Accel_z-49152)/16384;
     //加速度反正切公式计算三个轴和水平面坐标系之间的夹角
     Angle_x_temp=(atan(accy/accz))*180/3.14;
     Angle_y_temp=(atan(accx/accz))*180/3.14;
     //判断计算后角度的正负号
     if(Accel_x<32764) Angle_y_temp = +Angle_y_temp;
     if(Accel_x>32764) Angle_y_temp = -Angle_y_temp;
     if(Accel_y<32764) Angle_x_temp = +Angle_x_temp;
     if(Accel_y>32764) Angle_x_temp = -Angle_x_temp;

     //3.角速度原始值处理过程
     //陀螺仪配置寄存器0X1B内写入0x18，设置范围为±2000deg/s。换算关系：2^16/4000=16.4LSB/(°/S)
     ////计算角速度
     if(Gyro_x<32768) Gyro_x=-(Gyro_x/16.4);
     if(Gyro_x>32768) Gyro_x=+(65535-Gyro_x)/16.4;
     if(Gyro_y<32768) Gyro_y=-(Gyro_y/16.4);
     if(Gyro_y>32768) Gyro_y=+(65535-Gyro_y)/16.4;
     if(Gyro_z<32768) Gyro_z=-(Gyro_z/16.4);
     if(Gyro_z>32768) Gyro_z=+(65535-Gyro_z)/16.4;
     angle = K1 * Angle_x_temp+ (1-K1) * (angle + (Gyro_x-0.46)* dt);
     return angle;
}

