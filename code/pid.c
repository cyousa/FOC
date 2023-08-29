/*
 * pid.c
 *
 *  Created on: 2023年5月1日
 *      Author: hanser
 */
#include "pid.h"
#define max 5000
//速度环
PID_TYPE ROL_velocity;
PID_TYPE PITCH_velocity;
float P_yawan=0;
//角度环
PID_TYPE ROL_angle;
PID_TYPE PITCH_angle;
//转向环
PID_TYPE turn;
PID_TYPE turn_rate;
PID_TYPE turn_velocity;
//角速度环
PID_TYPE ROL_Rate;
PID_TYPE PITCH_Rate;
/*动量轮速度环kp:10.2,ki:0.04
 *    角度环kp:-340,kd:-15.5
 *    角速度环kp:-185,ki:158//165
 *
 * */

float Angle_zero;
float Turn_pwm;
float Target_Encoder;

/*****************************************************************************
* 函  数：void PID_Postion_Cal(PID_TYPE*PID,float target,float measure)
* 功  能：位置式PID算法
* 参  数：PID: 算法P I D参数的结构体
*       target: 目标值(输入值)
*       measure: 测量值
*       Pwm=Kp*e(k)+Ki*∑ e(k)+Kd[e(k)-e(k-1)]
* 返回值：无
* 备  注:
*****************************************************************************/
void PID_Postion_Cal(PID_TYPE*PID,float target,float measure)
{
    PID->Error  = target - measure;                                 //误差
    PID->Differ = PID->Error - PID->PreError;                           //微分量

    PID->Pout = PID->P * PID->Error;                        //比例控制
    PID->Iout = PID->Ilimit_flag * PID->I * PID->Integral;  //积分控制
    PID->Dout = PID->D * PID->Differ;                       //微分控制

    PID->OutPut =  PID->Pout + PID->Iout + PID->Dout;       //比例 + 积分 + 微分总控制
    if(measure > (PID->Ilimit)||measure < -PID->Ilimit)   //积分分离（当被控量和设定值偏差较大时，取消积分作用）
    {PID->Ilimit_flag = 0;}
    else
    {
        PID->Ilimit_flag = 1;                               //加入积分（积分分离标志为1，即积分项参与调节）
        PID->Integral += PID->Error;                        //对误差进行积分
        if(PID->Integral > PID->Irang)                      //积分限幅
            PID->Integral = PID->Irang;
        if(PID->Integral < -PID->Irang)                     //积分限幅
            PID->Integral = -PID->Irang;
    }
    PID->PreError = PID->Error;
}
/*****************************************************************************
* 函  数：void Pid_increment_Cal(PID_TYPE*PID,float target,float measure)
* 功  能：增量式PID
* 注  意：Pwm=Kp[e(k)-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
* 返回值：无
* 备  注:
*****************************************************************************/
void Pid_increment_Cal(PID_TYPE*PID,float target,float measure)
{
    PID->Error  = target - measure;                                 //误差
    PID->Pout = PID->P * (PID->Error-PID->PreError);                        //比例控制
    PID->Iout = PID->I * PID->Error;                                        //积分控制
    PID->Dout = PID->D * (PID->Error-2*PID->PreError+PID->PrePreError);     //微分控制
    PID->OutPut =  PID->Pout + PID->Iout + PID->Dout;       //比例 + 积分 + 微分总控制

    if(PID->Integral > PID->Irang)     //积分限幅
        PID->Integral = PID->Irang;
    if(PID->Integral < -PID->Irang)    //积分限幅
        PID->Integral = -PID->Irang;

    PID->PrePreError=PID->PreError; //记忆e(k-2)
    PID->PreError=PID->Error;       //记忆e(k-1)
}


/*****************************************************************************
* 函  数：void PidParameter_init(void)
* 功  能：初始化PID结构体里的一些成员值
* 参  数：无
* 返回值：无
* 备  注: 不使用flash存储PID参数 运行时所有参数全部进行初始化
*****************************************************************************/
void PidParameter_init(void)
{

        /*      ROL_velocity.P=6;//18;//20;
      ROL_velocity.I=0.05;//0.07;
      ROL_velocity.D=0;
      ROL_velocity.Ilimit_flag = 0;//Roll轴角度积分的分离标志
      ROL_velocity.Ilimit =1000;    //Roll角度积分范围
      ROL_velocity.Irang = max;    //Roll轴角速度积分限幅度（由于电机输出有限，所以积分输出也是有限的）
      //ROL轴角度
      ROL_angle.P =-680;//-450;//
      ROL_angle.I = 0;
      ROL_angle.D = -30;//15;//-25;//130;
      ROL_angle.Ilimit_flag = 0;//Roll轴角度积分的分离标志
      ROL_angle.Ilimit =max;    //Roll轴角度积分范围
      ROL_angle.Irang = max;    //Roll轴角度积分限幅度（由于电机输出有限，所以积分输出也是有限的）
      //ROL角速度
      ROL_Rate.P =250;//350;//230
      ROL_Rate.I =125;//150;//100;
      ROL_Rate.D = 0;//20;
      ROL_Rate.Ilimit_flag = 0; //Roll轴角速度积分的分离标志
      ROL_Rate.Ilimit = 20000;   //Roll轴角速度积分范围
      ROL_Rate.Irang = max;    //Roll轴角速度积分限幅度（由于电机输出有限，所以积分输出也是有限的）

\
         * */
         P_yawan=-0.0000;//0.0001;

         ROL_velocity.P=10;//6.5;//10.5;//7;
                 ROL_velocity.I=0.02;//0.045
                 ROL_velocity.D=0;
                 ROL_velocity.Ilimit_flag = 0;//Roll轴角度积分的分离标志
                 ROL_velocity.Ilimit =1000;    //Roll角度积分范围
                 ROL_velocity.Irang = max;    //Roll轴角速度积分限幅度（由于电机输出有限，所以积分输出也是有限的）
                 //ROL轴角度
                 ROL_angle.P =-1400;//-1400;//-430;//-600;//-670;// -625;//-802
                 ROL_angle.I = 0;
                 ROL_angle.D = 0;//100;//-25;//130;
                 ROL_angle.Ilimit_flag = 0;//Roll轴角度积分的分离标志
                 ROL_angle.Ilimit =max;    //Roll轴角度积分范围
                 ROL_angle.Irang = max;    //Roll轴角度积分限幅度（由于电机输出有限，所以积分输出也是有限的）
                 //ROL角速度
                 ROL_Rate.P =250;//200
                 ROL_Rate.I =30;//30;//100;
                 ROL_Rate.D = 0;//20;
                 ROL_Rate.Ilimit_flag = 0; //Roll轴角速度积分的分离标志
                 ROL_Rate.Ilimit = 20000;   //Roll轴角速度积分范围
                 ROL_Rate.Irang = max;    //Roll轴角速度积分限幅度（由于电机输出有限，所以积分输出也是有限的）
                //PITCH轴速度
                PITCH_velocity.P=3.3;//3;//3;
                PITCH_velocity.D=0;
                PITCH_velocity.I=0.025;
                PITCH_velocity.Ilimit_flag = 0;//Roll轴速度积分的分离标志
                PITCH_velocity.Ilimit =800;    //Roll轴速度积分范围
                PITCH_velocity.Irang = 50000;    //Roll轴速度积分限幅度（由于电机输出有限，所以积分输出也是有限的）

                //PITCH轴角度
                PITCH_angle.P=8;//10;//11;//10;
                PITCH_angle.I=0;//
                PITCH_angle.D=-15;//20;//20;//10;
                PITCH_angle.Ilimit_flag = 0;//Roll轴角度积分的分离标志
                PITCH_angle.Ilimit = 5000;    //Roll轴角度积分范围
                PITCH_angle.Irang = 5000;    //Roll轴角度积分限幅度（由于电机输出有限，所以积分输出也是有限的）
                //PITCH轴角速度
                PITCH_Rate.P=55;//50;//50;
                PITCH_Rate.I=40;//10;//50;//50;
                PITCH_Rate.D=0;//3;//2;
                PITCH_Rate.Ilimit_flag = 0; //Pitch轴角速度积分的分离标志
                PITCH_Rate.Ilimit = 5000;    //Pitch轴角速度积分范围
                PITCH_Rate.Irang = 10000;    //Pitch轴角速度积分限幅度（由于电机输出有限，所以积分输出也是有限的）

       turn.P=10;//10;
       turn.D=1;//1;//25;;
       turn.I=0;//-0.5;
       turn.Irang=10;


       Target_Encoder=0;//

       turn_rate.P=30;
       turn_rate.D=0;
       turn_rate.I=0;
       turn_rate.Irang=0;

       turn_velocity.P=0;//-50;(+负反馈)（-正反馈）
       turn_velocity.I=0;
       turn_velocity.Irang=1000;
       turn_velocity.Ilimit=5000;


}
float Xianfu(float pwm)
{
    if(pwm>10000)
    {
        pwm= 10000.0;
    }
    else if(pwm<-10000)
    {
        pwm= -10000.0;
    }

    return pwm;
}
/*****************************************************************************
* 函  数：void X_suduhuan(int encoder)
* 功  能：动量轮速度PI控制,速度正反馈环,速度环 pi控制
* 参  数：encoder 当前编码器数值(两侧编码器数值的均值)
* 返回值：无，PID积分器的输出在PID结构体中获取
* 备  注: Pwm=Kp[e(k)-e(k-1)]+Ki*e(k)(期望速度为0 -当前速度  由于期望速度为0 就不写了)
* 此时的Encoder就是e(k)
*****************************************************************************/
float Encoder;

void X_suduhuan(int encoder)
{
    float Encoder_Least;
    Encoder_Least = (float)encoder*1.0;//当前速度滤波
    //一阶低通滤波
    Encoder *= 0.8;//之前编码器数值占70%
    Encoder += Encoder_Least*0.2;//当前的30%
    //右压，目标速度为+
        //左压，目标速度为-

    ROL_velocity.Error  = 0 - Encoder;                                 //误差
    ROL_velocity.Differ = ROL_velocity.Error - ROL_velocity.PreError;                           //微分量

    ROL_velocity.Pout = ROL_velocity.P * ROL_velocity.Error;                        //比例控制
    ROL_velocity.Iout = ROL_velocity.Ilimit_flag * ROL_velocity.I * ROL_velocity.Integral;  //积分控制
    ROL_velocity.Dout = ROL_velocity.D * ROL_velocity.Differ;                       //微分控制

    ROL_velocity.OutPut =  ROL_velocity.Pout/100 + ROL_velocity.Iout/100 + ROL_velocity.Dout;       //比例 + 积分 + 微分总控制

        if(Encoder > (ROL_velocity.Ilimit)||Encoder < -ROL_velocity.Ilimit)   //积分分离（当被控量和设定值偏差较大时，取消积分作用）
        {ROL_velocity.Ilimit_flag = 0;}
        else
        {
            ROL_velocity.Ilimit_flag = 1;                               //加入积分（积分分离标志为1，即积分项参与调节）
            ROL_velocity.Integral += ROL_velocity.Error;                        //对误差进行积分
            if(ROL_velocity.Integral > ROL_velocity.Irang)                      //积分限幅
                ROL_velocity.Integral = ROL_velocity.Irang;
            if(ROL_velocity.Integral < -ROL_velocity.Irang)                     //积分限幅
                ROL_velocity.Integral = -ROL_velocity.Irang;
        }
        ROL_velocity.PreError = ROL_velocity.Error;
}
/*****************************************************************************
* 函  数：void X_jiaoduhuan(float Angle,float taget)
* 功  能：动量轮角度PD控制
* 参  数：Angle 当前卡尔曼滤波后的角度
*       taget 机械中值
* 返回值：无，PID积分器的输出在PID结构体中获取
* 备  注: Pwm=Kp*e(k)+Kd[e(k)-e(k-1)]
* 在定时器中断中调用三环控制器，最终的PWM输出在角速度环的PID.Output中
*****************************************************************************/
void X_jiaoduhuan(float Angle)
{


    ROL_angle.Error = (ROL_velocity.OutPut)-Angle;//-1.3->-0.625->-1.1ROL_velocity.OutPut+
    ROL_angle.Differ=ROL_angle.Error-ROL_angle.PreError;

    ROL_angle.Pout = ROL_angle.P * ROL_angle.Error;//比例控制
    ROL_angle.Dout = ROL_angle.D * ROL_angle.Differ;//微分控制

    ROL_angle.OutPut =  ROL_angle.Pout/100 + ROL_angle.Dout/100;       //比例+微分总控制
    ROL_angle.PreError = ROL_angle.Error;
}
/*****************************************************************************
* 函  数：void X_jiaosuduhuan(float Gyro,float taget)
* 功  能：动量轮角速度度PI控制
* 参  数：Gyro 当前直接获取的角速度
*       taget 角度环的输出
* 返回值：无，PID积分器的输出在PID结构体中获取
* 备  注: Pwm=Kp[e(k)-e(k-1)]+Ki*e(k)
*****************************************************************************/
float left_pwm=0,right_pwm=0;
void X_jiaosuduhuan(float Gyro)
{
    ROL_Rate.Error  = ROL_angle.OutPut - Gyro;                                 //误差
       ROL_Rate.Differ = ROL_Rate.Error - ROL_Rate.PreError;                           //微分量

       ROL_Rate.Pout = ROL_Rate.P * ROL_Rate.Error;                        //比例控制
       ROL_Rate.Iout = ROL_Rate.Ilimit_flag * ROL_Rate.I * ROL_Rate.Integral;  //积分控制
       ROL_Rate.Dout = ROL_Rate.D *ROL_Rate.Differ;                       //微分控制

       ROL_Rate.OutPut =  ROL_Rate.Pout +ROL_Rate.Iout/100 + ROL_Rate.Dout;       //比例 + 积分 + 微分总控制
       if(Gyro > (ROL_Rate.Ilimit)||Gyro < -ROL_Rate.Ilimit)   //积分分离（当被控量和设定值偏差较大时，取消积分作用）
       {ROL_Rate.Ilimit_flag = 0;}
       else
       {
           ROL_Rate.Ilimit_flag = 1;                               //加入积分（积分分离标志为1，即积分项参与调节）
           ROL_Rate.Integral += ROL_Rate.Error;                        //对误差进行积分
           if(ROL_Rate.Integral > ROL_Rate.Irang)                      //积分限幅
               ROL_Rate.Integral = ROL_Rate.Irang;
           if(ROL_Rate.Integral < -ROL_Rate.Irang)                     //积分限幅
               ROL_Rate.Integral = -ROL_Rate.Irang;
       }
       ROL_Rate.PreError = ROL_Rate.Error;
       ROL_Rate.OutPut=Xianfu(ROL_Rate.OutPut);
       ROL_Rate.OutPut=- ROL_Rate.OutPut;

       left_pwm=ROL_Rate.OutPut-Turn_pwm;
       right_pwm=ROL_Rate.OutPut+Turn_pwm;
       left_pwm=Xianfu(left_pwm);
       right_pwm=Xianfu(right_pwm);
    if(Angle_Y_Final<15&&Angle_Y_Final>-15||Angle_X_Final>25||Angle_X_Final<-25)//电机保护
    {
      Leftwheel_pwm(left_pwm);
      Rightwheel_pwm(-right_pwm);


    }
    else
    {
        Leftwheel_pwm(0);
        Rightwheel_pwm(0);
        forward_pwm(0);
    }

}
/*****************************************************************************
* 函  数：void Y_suduhuan(int encoder)
* 功  能：行进轮速度PI控制,速度正反馈环,速度环 pi控制
* 参  数：encoder 当前编码器数值
* 返回值：无，PID积分器的输出在PID结构体中获取
* 备  注: Pwm=Kp[e(k)-e(k-1)]+Ki*e(k)(期望速度为0 -当前速度  由于期望速度为0 就不写了)
* 此时的Y_Encoder就是e(k)
*****************************************************************************/
float Y_Encoder;

void Y_suduhuan(int encoder)
{
    float Encoder_Least;
    Encoder_Least = (float)encoder*1.0;//当前速度滤波
    //一阶低通滤波
    Y_Encoder *= 0.7;//之前编码器数值占70%
    Y_Encoder += Encoder_Least*0.3;//当前的30%

    PITCH_velocity.Error  = Target_Encoder - Y_Encoder;                                 //误差
    PITCH_velocity.Differ = PITCH_velocity.Error - PITCH_velocity.PreError;                           //微分量

    PITCH_velocity.Pout = PITCH_velocity.P * PITCH_velocity.Error;                        //比例控制
    PITCH_velocity.Iout = PITCH_velocity.Ilimit_flag * PITCH_velocity.I * PITCH_velocity.Integral;  //积分控制
    PITCH_velocity.Dout = PITCH_velocity.D * PITCH_velocity.Differ;                       //微分控制

    PITCH_velocity.OutPut =  PITCH_velocity.Pout/100 + PITCH_velocity.Iout/100 + PITCH_velocity.Dout;       //比例 + 积分 + 微分总控制
    if(Y_Encoder > (PITCH_velocity.Ilimit)||Y_Encoder < -PITCH_velocity.Ilimit)   //积分分离（当被控量和设定值偏差较大时，取消积分作用）
    {PITCH_velocity.Ilimit_flag = 0;}
    else
    {
        PITCH_velocity.Ilimit_flag = 1;                               //加入积分（积分分离标志为1，即积分项参与调节）
        PITCH_velocity.Integral += PITCH_velocity.Error;                        //对误差进行积分
        if(PITCH_velocity.Integral > PITCH_velocity.Irang)                      //积分限幅
            PITCH_velocity.Integral = PITCH_velocity.Irang;
        if(PITCH_velocity.Integral < -PITCH_velocity.Irang)                     //积分限幅
            PITCH_velocity.Integral = -PITCH_velocity.Irang;
    }
    PITCH_velocity.PreError = PITCH_velocity.Error;
}
/*****************************************************************************
* 函  数：void Y_jiaoduhuan(float Angle)
* 功  能：行进电机角度PD控制
* 参  数：Angle 当前卡尔曼滤波后的角度
* 返回值：无，PID积分器的输出在PID结构体中获取
* 备  注: Pwm=Kp*e(k)+Kd[e(k)-e(k-1)]
* 在定时器中断中调用三环控制器，最终的PWM输出在角速度环的PID.Output中
*****************************************************************************/
void Y_jiaoduhuan(float Angle)
{
    PITCH_angle.Error = (PITCH_velocity.OutPut)-Angle;//PITCH_velocity.OutPut3.3
    PITCH_angle.Differ=PITCH_angle.Error-PITCH_angle.PreError;

    PITCH_angle.Integral += PITCH_angle.Error;                        //对误差进行积分
    if(PITCH_angle.Integral > PITCH_angle.Irang)                      //积分限幅
        PITCH_angle.Integral = PITCH_angle.Irang;
    if(PITCH_angle.Integral < -PITCH_angle.Irang)                     //积分限幅
        PITCH_angle.Integral = -PITCH_angle.Irang;

    PITCH_angle.Pout = PITCH_angle.P * PITCH_angle.Error;//比例控制
    PITCH_angle.Dout = PITCH_angle.D * PITCH_angle.Differ;//微分控制
    PITCH_angle.Iout = PITCH_angle.I * PITCH_angle.Integral;  //积分控制
    PITCH_angle.OutPut =  PITCH_angle.Pout + PITCH_angle.Dout+ PITCH_angle.Iout/100;       //比例+微分总控制
    PITCH_angle.PreError=PITCH_angle.Error;
}
/*****************************************************************************
* 函  数：void Y_jiaosuduhuan(float Gyro)
* 功  能：行进电机角速度度PI控制
* 参  数：Gyro 当前直接获取的角速度
* 返回值：无，PID积分器的输出在PID结构体中获取
* 备  注: Pwm=Kp[e(k)-e(k-1)]+Ki*e(k)
*****************************************************************************/
void Y_jiaosuduhuan(float Gyro)
{
//   if(Target_Encoder)
 //   {
        PITCH_Rate.Ilimit_flag=1;
 //   }
 //   else
  //  {
    //    PITCH_Rate.Ilimit_flag=0;
   // }
    PITCH_Rate.Error = PITCH_angle.OutPut-Gyro;
    PITCH_Rate.Differ=PITCH_Rate.Error-PITCH_Rate.PreError;

    PITCH_Rate.Pout = PITCH_Rate.P * PITCH_Rate.Error;                        //比例控制
    PITCH_Rate.Iout = PITCH_Rate.Ilimit_flag * PITCH_Rate.I * PITCH_Rate.Integral;  //积分控制
    PITCH_Rate.Dout = PITCH_Rate.D *PITCH_Rate.Differ;

    PITCH_Rate.OutPut =  PITCH_Rate.Pout +PITCH_Rate.Iout/100 + PITCH_Rate.Dout;       //比例 + 积分 + 微分总控制
    if(Gyro > (PITCH_Rate.Ilimit)||Gyro < -PITCH_Rate.Ilimit)   //积分分离（当被控量和设定值偏差较大时，取消积分作用）
    {PITCH_Rate.Ilimit_flag = 0;}
    else
    {
        PITCH_Rate.Ilimit_flag = 1;                               //加入积分（积分分离标志为1，即积分项参与调节）
        PITCH_Rate.Integral += PITCH_Rate.Error;                        //对误差进行积分
        if(PITCH_Rate.Integral > PITCH_Rate.Irang)                      //积分限幅
            PITCH_Rate.Integral = PITCH_Rate.Irang;
        if(PITCH_Rate.Integral < -PITCH_Rate.Irang)                     //积分限幅
            PITCH_Rate.Integral = -PITCH_Rate.Irang;
    }
    PITCH_Rate.PreError = PITCH_Rate.Error;
    if(PITCH_Rate.OutPut>9000)PITCH_Rate.OutPut=9000;
    if(PITCH_Rate.OutPut<-9000)PITCH_Rate.OutPut=-9000;

   if(Angle_X_Final<25&&Angle_X_Final>-25&&Angle_Y_Final<15&&Angle_Y_Final>-15)//电机保护
    {

      forward_pwm(PITCH_Rate.OutPut);
   }
    else
    {
        Leftwheel_pwm(0);
        Rightwheel_pwm(20);
        forward_pwm(0);
    }
}

void Turn(float error)//转向环
{

    turn.Error=(turn_velocity.OutPut-error);
    turn.Differ=turn.Error-turn.PreError;
    turn.Pout = turn.P * (turn.Error);//比例控制
    turn.Dout = turn.D * turn.Differ;//微分控制


    turn.Integral += turn.Error;                        //对误差进行积分
    if(turn.Integral > turn.Irang)                      //积分限幅
        turn.Integral = turn.Irang;
    if(turn.Integral < -turn.Irang)                     //积分限幅
        turn.Integral = -turn.Irang;
    turn.Iout=turn.I*turn.Integral;
    if(turn.Error==0)
    {
        turn.Integral=0;

    }
    turn.OutPut =  turn.Pout + turn.Dout + turn.Iout;       //比例+微分总控制
    turn.PreError = turn.Error;



}

float Turn_gryo(float gyro)//转向环
{

       turn_rate.Error =  turn.OutPut - gyro;                                 //误差
       turn_rate.Differ = turn_rate.Error - turn_rate.PreError;                           //微分量

       turn_rate.Pout = turn_rate.P * turn_rate.Error;                        //比例控制
       turn_rate.Iout = turn_rate.I * turn_rate.Integral;  //积分控制
       turn_rate.Dout = turn_rate.D *turn_rate.Differ;                       //微分控制

       turn_rate.OutPut =  turn_rate.Pout +turn_rate.Iout/100 + turn_rate.Dout;       //比例 + 积分 + 微分总控制

      // turn_rate.Integral+=turn_rate.Error;
       turn_rate.PreError = turn_rate.Error;
      return turn_rate.OutPut;


}
void Turn_suduhuan(int encoder)
{
    float Encoder_Least;
    Encoder_Least = (float)encoder*1.0;//当前速度滤波
    //一阶低通滤波
    Encoder *= 0.8;//之前编码器数值占70%
    Encoder += Encoder_Least*0.2;//当前的30%

    turn_velocity.Error  = 0 - Encoder;                                 //误差
    turn_velocity.Differ = turn_velocity.Error - turn_velocity.PreError;                           //微分量

    turn_velocity.Pout = turn_velocity.P * turn_velocity.Error;                        //比例控制
    turn_velocity.Iout = turn_velocity.I * turn_velocity.Integral;  //积分控制
    turn_velocity.Dout = turn_velocity.D * turn_velocity.Differ;                       //微分控制

    turn_velocity.OutPut =  turn_velocity.Pout/100 + turn_velocity.Iout/100 + turn_velocity.Dout;       //比例 + 积分 + 微分总控制


    turn_velocity.PreError = turn_velocity.Error;
}
