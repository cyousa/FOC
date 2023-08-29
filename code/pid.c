/*
 * pid.c
 *
 *  Created on: 2023��5��1��
 *      Author: hanser
 */
#include "pid.h"
#define max 5000
//�ٶȻ�
PID_TYPE ROL_velocity;
PID_TYPE PITCH_velocity;
float P_yawan=0;
//�ǶȻ�
PID_TYPE ROL_angle;
PID_TYPE PITCH_angle;
//ת��
PID_TYPE turn;
PID_TYPE turn_rate;
PID_TYPE turn_velocity;
//���ٶȻ�
PID_TYPE ROL_Rate;
PID_TYPE PITCH_Rate;
/*�������ٶȻ�kp:10.2,ki:0.04
 *    �ǶȻ�kp:-340,kd:-15.5
 *    ���ٶȻ�kp:-185,ki:158//165
 *
 * */

float Angle_zero;
float Turn_pwm;
float Target_Encoder;

/*****************************************************************************
* ��  ����void PID_Postion_Cal(PID_TYPE*PID,float target,float measure)
* ��  �ܣ�λ��ʽPID�㷨
* ��  ����PID: �㷨P I D�����Ľṹ��
*       target: Ŀ��ֵ(����ֵ)
*       measure: ����ֵ
*       Pwm=Kp*e(k)+Ki*�� e(k)+Kd[e(k)-e(k-1)]
* ����ֵ����
* ��  ע:
*****************************************************************************/
void PID_Postion_Cal(PID_TYPE*PID,float target,float measure)
{
    PID->Error  = target - measure;                                 //���
    PID->Differ = PID->Error - PID->PreError;                           //΢����

    PID->Pout = PID->P * PID->Error;                        //��������
    PID->Iout = PID->Ilimit_flag * PID->I * PID->Integral;  //���ֿ���
    PID->Dout = PID->D * PID->Differ;                       //΢�ֿ���

    PID->OutPut =  PID->Pout + PID->Iout + PID->Dout;       //���� + ���� + ΢���ܿ���
    if(measure > (PID->Ilimit)||measure < -PID->Ilimit)   //���ַ��루�����������趨ֵƫ��ϴ�ʱ��ȡ���������ã�
    {PID->Ilimit_flag = 0;}
    else
    {
        PID->Ilimit_flag = 1;                               //������֣����ַ����־Ϊ1���������������ڣ�
        PID->Integral += PID->Error;                        //�������л���
        if(PID->Integral > PID->Irang)                      //�����޷�
            PID->Integral = PID->Irang;
        if(PID->Integral < -PID->Irang)                     //�����޷�
            PID->Integral = -PID->Irang;
    }
    PID->PreError = PID->Error;
}
/*****************************************************************************
* ��  ����void Pid_increment_Cal(PID_TYPE*PID,float target,float measure)
* ��  �ܣ�����ʽPID
* ע  �⣺Pwm=Kp[e(k)-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
* ����ֵ����
* ��  ע:
*****************************************************************************/
void Pid_increment_Cal(PID_TYPE*PID,float target,float measure)
{
    PID->Error  = target - measure;                                 //���
    PID->Pout = PID->P * (PID->Error-PID->PreError);                        //��������
    PID->Iout = PID->I * PID->Error;                                        //���ֿ���
    PID->Dout = PID->D * (PID->Error-2*PID->PreError+PID->PrePreError);     //΢�ֿ���
    PID->OutPut =  PID->Pout + PID->Iout + PID->Dout;       //���� + ���� + ΢���ܿ���

    if(PID->Integral > PID->Irang)     //�����޷�
        PID->Integral = PID->Irang;
    if(PID->Integral < -PID->Irang)    //�����޷�
        PID->Integral = -PID->Irang;

    PID->PrePreError=PID->PreError; //����e(k-2)
    PID->PreError=PID->Error;       //����e(k-1)
}


/*****************************************************************************
* ��  ����void PidParameter_init(void)
* ��  �ܣ���ʼ��PID�ṹ�����һЩ��Աֵ
* ��  ������
* ����ֵ����
* ��  ע: ��ʹ��flash�洢PID���� ����ʱ���в���ȫ�����г�ʼ��
*****************************************************************************/
void PidParameter_init(void)
{

        /*      ROL_velocity.P=6;//18;//20;
      ROL_velocity.I=0.05;//0.07;
      ROL_velocity.D=0;
      ROL_velocity.Ilimit_flag = 0;//Roll��ǶȻ��ֵķ����־
      ROL_velocity.Ilimit =1000;    //Roll�ǶȻ��ַ�Χ
      ROL_velocity.Irang = max;    //Roll����ٶȻ����޷��ȣ����ڵ��������ޣ����Ի������Ҳ�����޵ģ�
      //ROL��Ƕ�
      ROL_angle.P =-680;//-450;//
      ROL_angle.I = 0;
      ROL_angle.D = -30;//15;//-25;//130;
      ROL_angle.Ilimit_flag = 0;//Roll��ǶȻ��ֵķ����־
      ROL_angle.Ilimit =max;    //Roll��ǶȻ��ַ�Χ
      ROL_angle.Irang = max;    //Roll��ǶȻ����޷��ȣ����ڵ��������ޣ����Ի������Ҳ�����޵ģ�
      //ROL���ٶ�
      ROL_Rate.P =250;//350;//230
      ROL_Rate.I =125;//150;//100;
      ROL_Rate.D = 0;//20;
      ROL_Rate.Ilimit_flag = 0; //Roll����ٶȻ��ֵķ����־
      ROL_Rate.Ilimit = 20000;   //Roll����ٶȻ��ַ�Χ
      ROL_Rate.Irang = max;    //Roll����ٶȻ����޷��ȣ����ڵ��������ޣ����Ի������Ҳ�����޵ģ�

\
         * */
         P_yawan=-0.0000;//0.0001;

         ROL_velocity.P=10;//6.5;//10.5;//7;
                 ROL_velocity.I=0.02;//0.045
                 ROL_velocity.D=0;
                 ROL_velocity.Ilimit_flag = 0;//Roll��ǶȻ��ֵķ����־
                 ROL_velocity.Ilimit =1000;    //Roll�ǶȻ��ַ�Χ
                 ROL_velocity.Irang = max;    //Roll����ٶȻ����޷��ȣ����ڵ��������ޣ����Ի������Ҳ�����޵ģ�
                 //ROL��Ƕ�
                 ROL_angle.P =-1400;//-1400;//-430;//-600;//-670;// -625;//-802
                 ROL_angle.I = 0;
                 ROL_angle.D = 0;//100;//-25;//130;
                 ROL_angle.Ilimit_flag = 0;//Roll��ǶȻ��ֵķ����־
                 ROL_angle.Ilimit =max;    //Roll��ǶȻ��ַ�Χ
                 ROL_angle.Irang = max;    //Roll��ǶȻ����޷��ȣ����ڵ��������ޣ����Ի������Ҳ�����޵ģ�
                 //ROL���ٶ�
                 ROL_Rate.P =250;//200
                 ROL_Rate.I =30;//30;//100;
                 ROL_Rate.D = 0;//20;
                 ROL_Rate.Ilimit_flag = 0; //Roll����ٶȻ��ֵķ����־
                 ROL_Rate.Ilimit = 20000;   //Roll����ٶȻ��ַ�Χ
                 ROL_Rate.Irang = max;    //Roll����ٶȻ����޷��ȣ����ڵ��������ޣ����Ի������Ҳ�����޵ģ�
                //PITCH���ٶ�
                PITCH_velocity.P=3.3;//3;//3;
                PITCH_velocity.D=0;
                PITCH_velocity.I=0.025;
                PITCH_velocity.Ilimit_flag = 0;//Roll���ٶȻ��ֵķ����־
                PITCH_velocity.Ilimit =800;    //Roll���ٶȻ��ַ�Χ
                PITCH_velocity.Irang = 50000;    //Roll���ٶȻ����޷��ȣ����ڵ��������ޣ����Ի������Ҳ�����޵ģ�

                //PITCH��Ƕ�
                PITCH_angle.P=8;//10;//11;//10;
                PITCH_angle.I=0;//
                PITCH_angle.D=-15;//20;//20;//10;
                PITCH_angle.Ilimit_flag = 0;//Roll��ǶȻ��ֵķ����־
                PITCH_angle.Ilimit = 5000;    //Roll��ǶȻ��ַ�Χ
                PITCH_angle.Irang = 5000;    //Roll��ǶȻ����޷��ȣ����ڵ��������ޣ����Ի������Ҳ�����޵ģ�
                //PITCH����ٶ�
                PITCH_Rate.P=55;//50;//50;
                PITCH_Rate.I=40;//10;//50;//50;
                PITCH_Rate.D=0;//3;//2;
                PITCH_Rate.Ilimit_flag = 0; //Pitch����ٶȻ��ֵķ����־
                PITCH_Rate.Ilimit = 5000;    //Pitch����ٶȻ��ַ�Χ
                PITCH_Rate.Irang = 10000;    //Pitch����ٶȻ����޷��ȣ����ڵ��������ޣ����Ի������Ҳ�����޵ģ�

       turn.P=10;//10;
       turn.D=1;//1;//25;;
       turn.I=0;//-0.5;
       turn.Irang=10;


       Target_Encoder=0;//

       turn_rate.P=30;
       turn_rate.D=0;
       turn_rate.I=0;
       turn_rate.Irang=0;

       turn_velocity.P=0;//-50;(+������)��-��������
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
* ��  ����void X_suduhuan(int encoder)
* ��  �ܣ��������ٶ�PI����,�ٶ���������,�ٶȻ� pi����
* ��  ����encoder ��ǰ��������ֵ(�����������ֵ�ľ�ֵ)
* ����ֵ���ޣ�PID�������������PID�ṹ���л�ȡ
* ��  ע: Pwm=Kp[e(k)-e(k-1)]+Ki*e(k)(�����ٶ�Ϊ0 -��ǰ�ٶ�  ���������ٶ�Ϊ0 �Ͳ�д��)
* ��ʱ��Encoder����e(k)
*****************************************************************************/
float Encoder;

void X_suduhuan(int encoder)
{
    float Encoder_Least;
    Encoder_Least = (float)encoder*1.0;//��ǰ�ٶ��˲�
    //һ�׵�ͨ�˲�
    Encoder *= 0.8;//֮ǰ��������ֵռ70%
    Encoder += Encoder_Least*0.2;//��ǰ��30%
    //��ѹ��Ŀ���ٶ�Ϊ+
        //��ѹ��Ŀ���ٶ�Ϊ-

    ROL_velocity.Error  = 0 - Encoder;                                 //���
    ROL_velocity.Differ = ROL_velocity.Error - ROL_velocity.PreError;                           //΢����

    ROL_velocity.Pout = ROL_velocity.P * ROL_velocity.Error;                        //��������
    ROL_velocity.Iout = ROL_velocity.Ilimit_flag * ROL_velocity.I * ROL_velocity.Integral;  //���ֿ���
    ROL_velocity.Dout = ROL_velocity.D * ROL_velocity.Differ;                       //΢�ֿ���

    ROL_velocity.OutPut =  ROL_velocity.Pout/100 + ROL_velocity.Iout/100 + ROL_velocity.Dout;       //���� + ���� + ΢���ܿ���

        if(Encoder > (ROL_velocity.Ilimit)||Encoder < -ROL_velocity.Ilimit)   //���ַ��루�����������趨ֵƫ��ϴ�ʱ��ȡ���������ã�
        {ROL_velocity.Ilimit_flag = 0;}
        else
        {
            ROL_velocity.Ilimit_flag = 1;                               //������֣����ַ����־Ϊ1���������������ڣ�
            ROL_velocity.Integral += ROL_velocity.Error;                        //�������л���
            if(ROL_velocity.Integral > ROL_velocity.Irang)                      //�����޷�
                ROL_velocity.Integral = ROL_velocity.Irang;
            if(ROL_velocity.Integral < -ROL_velocity.Irang)                     //�����޷�
                ROL_velocity.Integral = -ROL_velocity.Irang;
        }
        ROL_velocity.PreError = ROL_velocity.Error;
}
/*****************************************************************************
* ��  ����void X_jiaoduhuan(float Angle,float taget)
* ��  �ܣ������ֽǶ�PD����
* ��  ����Angle ��ǰ�������˲���ĽǶ�
*       taget ��е��ֵ
* ����ֵ���ޣ�PID�������������PID�ṹ���л�ȡ
* ��  ע: Pwm=Kp*e(k)+Kd[e(k)-e(k-1)]
* �ڶ�ʱ���ж��е������������������յ�PWM����ڽ��ٶȻ���PID.Output��
*****************************************************************************/
void X_jiaoduhuan(float Angle)
{


    ROL_angle.Error = (ROL_velocity.OutPut)-Angle;//-1.3->-0.625->-1.1ROL_velocity.OutPut+
    ROL_angle.Differ=ROL_angle.Error-ROL_angle.PreError;

    ROL_angle.Pout = ROL_angle.P * ROL_angle.Error;//��������
    ROL_angle.Dout = ROL_angle.D * ROL_angle.Differ;//΢�ֿ���

    ROL_angle.OutPut =  ROL_angle.Pout/100 + ROL_angle.Dout/100;       //����+΢���ܿ���
    ROL_angle.PreError = ROL_angle.Error;
}
/*****************************************************************************
* ��  ����void X_jiaosuduhuan(float Gyro,float taget)
* ��  �ܣ������ֽ��ٶȶ�PI����
* ��  ����Gyro ��ǰֱ�ӻ�ȡ�Ľ��ٶ�
*       taget �ǶȻ������
* ����ֵ���ޣ�PID�������������PID�ṹ���л�ȡ
* ��  ע: Pwm=Kp[e(k)-e(k-1)]+Ki*e(k)
*****************************************************************************/
float left_pwm=0,right_pwm=0;
void X_jiaosuduhuan(float Gyro)
{
    ROL_Rate.Error  = ROL_angle.OutPut - Gyro;                                 //���
       ROL_Rate.Differ = ROL_Rate.Error - ROL_Rate.PreError;                           //΢����

       ROL_Rate.Pout = ROL_Rate.P * ROL_Rate.Error;                        //��������
       ROL_Rate.Iout = ROL_Rate.Ilimit_flag * ROL_Rate.I * ROL_Rate.Integral;  //���ֿ���
       ROL_Rate.Dout = ROL_Rate.D *ROL_Rate.Differ;                       //΢�ֿ���

       ROL_Rate.OutPut =  ROL_Rate.Pout +ROL_Rate.Iout/100 + ROL_Rate.Dout;       //���� + ���� + ΢���ܿ���
       if(Gyro > (ROL_Rate.Ilimit)||Gyro < -ROL_Rate.Ilimit)   //���ַ��루�����������趨ֵƫ��ϴ�ʱ��ȡ���������ã�
       {ROL_Rate.Ilimit_flag = 0;}
       else
       {
           ROL_Rate.Ilimit_flag = 1;                               //������֣����ַ����־Ϊ1���������������ڣ�
           ROL_Rate.Integral += ROL_Rate.Error;                        //�������л���
           if(ROL_Rate.Integral > ROL_Rate.Irang)                      //�����޷�
               ROL_Rate.Integral = ROL_Rate.Irang;
           if(ROL_Rate.Integral < -ROL_Rate.Irang)                     //�����޷�
               ROL_Rate.Integral = -ROL_Rate.Irang;
       }
       ROL_Rate.PreError = ROL_Rate.Error;
       ROL_Rate.OutPut=Xianfu(ROL_Rate.OutPut);
       ROL_Rate.OutPut=- ROL_Rate.OutPut;

       left_pwm=ROL_Rate.OutPut-Turn_pwm;
       right_pwm=ROL_Rate.OutPut+Turn_pwm;
       left_pwm=Xianfu(left_pwm);
       right_pwm=Xianfu(right_pwm);
    if(Angle_Y_Final<15&&Angle_Y_Final>-15||Angle_X_Final>25||Angle_X_Final<-25)//�������
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
* ��  ����void Y_suduhuan(int encoder)
* ��  �ܣ��н����ٶ�PI����,�ٶ���������,�ٶȻ� pi����
* ��  ����encoder ��ǰ��������ֵ
* ����ֵ���ޣ�PID�������������PID�ṹ���л�ȡ
* ��  ע: Pwm=Kp[e(k)-e(k-1)]+Ki*e(k)(�����ٶ�Ϊ0 -��ǰ�ٶ�  ���������ٶ�Ϊ0 �Ͳ�д��)
* ��ʱ��Y_Encoder����e(k)
*****************************************************************************/
float Y_Encoder;

void Y_suduhuan(int encoder)
{
    float Encoder_Least;
    Encoder_Least = (float)encoder*1.0;//��ǰ�ٶ��˲�
    //һ�׵�ͨ�˲�
    Y_Encoder *= 0.7;//֮ǰ��������ֵռ70%
    Y_Encoder += Encoder_Least*0.3;//��ǰ��30%

    PITCH_velocity.Error  = Target_Encoder - Y_Encoder;                                 //���
    PITCH_velocity.Differ = PITCH_velocity.Error - PITCH_velocity.PreError;                           //΢����

    PITCH_velocity.Pout = PITCH_velocity.P * PITCH_velocity.Error;                        //��������
    PITCH_velocity.Iout = PITCH_velocity.Ilimit_flag * PITCH_velocity.I * PITCH_velocity.Integral;  //���ֿ���
    PITCH_velocity.Dout = PITCH_velocity.D * PITCH_velocity.Differ;                       //΢�ֿ���

    PITCH_velocity.OutPut =  PITCH_velocity.Pout/100 + PITCH_velocity.Iout/100 + PITCH_velocity.Dout;       //���� + ���� + ΢���ܿ���
    if(Y_Encoder > (PITCH_velocity.Ilimit)||Y_Encoder < -PITCH_velocity.Ilimit)   //���ַ��루�����������趨ֵƫ��ϴ�ʱ��ȡ���������ã�
    {PITCH_velocity.Ilimit_flag = 0;}
    else
    {
        PITCH_velocity.Ilimit_flag = 1;                               //������֣����ַ����־Ϊ1���������������ڣ�
        PITCH_velocity.Integral += PITCH_velocity.Error;                        //�������л���
        if(PITCH_velocity.Integral > PITCH_velocity.Irang)                      //�����޷�
            PITCH_velocity.Integral = PITCH_velocity.Irang;
        if(PITCH_velocity.Integral < -PITCH_velocity.Irang)                     //�����޷�
            PITCH_velocity.Integral = -PITCH_velocity.Irang;
    }
    PITCH_velocity.PreError = PITCH_velocity.Error;
}
/*****************************************************************************
* ��  ����void Y_jiaoduhuan(float Angle)
* ��  �ܣ��н�����Ƕ�PD����
* ��  ����Angle ��ǰ�������˲���ĽǶ�
* ����ֵ���ޣ�PID�������������PID�ṹ���л�ȡ
* ��  ע: Pwm=Kp*e(k)+Kd[e(k)-e(k-1)]
* �ڶ�ʱ���ж��е������������������յ�PWM����ڽ��ٶȻ���PID.Output��
*****************************************************************************/
void Y_jiaoduhuan(float Angle)
{
    PITCH_angle.Error = (PITCH_velocity.OutPut)-Angle;//PITCH_velocity.OutPut3.3
    PITCH_angle.Differ=PITCH_angle.Error-PITCH_angle.PreError;

    PITCH_angle.Integral += PITCH_angle.Error;                        //�������л���
    if(PITCH_angle.Integral > PITCH_angle.Irang)                      //�����޷�
        PITCH_angle.Integral = PITCH_angle.Irang;
    if(PITCH_angle.Integral < -PITCH_angle.Irang)                     //�����޷�
        PITCH_angle.Integral = -PITCH_angle.Irang;

    PITCH_angle.Pout = PITCH_angle.P * PITCH_angle.Error;//��������
    PITCH_angle.Dout = PITCH_angle.D * PITCH_angle.Differ;//΢�ֿ���
    PITCH_angle.Iout = PITCH_angle.I * PITCH_angle.Integral;  //���ֿ���
    PITCH_angle.OutPut =  PITCH_angle.Pout + PITCH_angle.Dout+ PITCH_angle.Iout/100;       //����+΢���ܿ���
    PITCH_angle.PreError=PITCH_angle.Error;
}
/*****************************************************************************
* ��  ����void Y_jiaosuduhuan(float Gyro)
* ��  �ܣ��н�������ٶȶ�PI����
* ��  ����Gyro ��ǰֱ�ӻ�ȡ�Ľ��ٶ�
* ����ֵ���ޣ�PID�������������PID�ṹ���л�ȡ
* ��  ע: Pwm=Kp[e(k)-e(k-1)]+Ki*e(k)
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

    PITCH_Rate.Pout = PITCH_Rate.P * PITCH_Rate.Error;                        //��������
    PITCH_Rate.Iout = PITCH_Rate.Ilimit_flag * PITCH_Rate.I * PITCH_Rate.Integral;  //���ֿ���
    PITCH_Rate.Dout = PITCH_Rate.D *PITCH_Rate.Differ;

    PITCH_Rate.OutPut =  PITCH_Rate.Pout +PITCH_Rate.Iout/100 + PITCH_Rate.Dout;       //���� + ���� + ΢���ܿ���
    if(Gyro > (PITCH_Rate.Ilimit)||Gyro < -PITCH_Rate.Ilimit)   //���ַ��루�����������趨ֵƫ��ϴ�ʱ��ȡ���������ã�
    {PITCH_Rate.Ilimit_flag = 0;}
    else
    {
        PITCH_Rate.Ilimit_flag = 1;                               //������֣����ַ����־Ϊ1���������������ڣ�
        PITCH_Rate.Integral += PITCH_Rate.Error;                        //�������л���
        if(PITCH_Rate.Integral > PITCH_Rate.Irang)                      //�����޷�
            PITCH_Rate.Integral = PITCH_Rate.Irang;
        if(PITCH_Rate.Integral < -PITCH_Rate.Irang)                     //�����޷�
            PITCH_Rate.Integral = -PITCH_Rate.Irang;
    }
    PITCH_Rate.PreError = PITCH_Rate.Error;
    if(PITCH_Rate.OutPut>9000)PITCH_Rate.OutPut=9000;
    if(PITCH_Rate.OutPut<-9000)PITCH_Rate.OutPut=-9000;

   if(Angle_X_Final<25&&Angle_X_Final>-25&&Angle_Y_Final<15&&Angle_Y_Final>-15)//�������
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

void Turn(float error)//ת��
{

    turn.Error=(turn_velocity.OutPut-error);
    turn.Differ=turn.Error-turn.PreError;
    turn.Pout = turn.P * (turn.Error);//��������
    turn.Dout = turn.D * turn.Differ;//΢�ֿ���


    turn.Integral += turn.Error;                        //�������л���
    if(turn.Integral > turn.Irang)                      //�����޷�
        turn.Integral = turn.Irang;
    if(turn.Integral < -turn.Irang)                     //�����޷�
        turn.Integral = -turn.Irang;
    turn.Iout=turn.I*turn.Integral;
    if(turn.Error==0)
    {
        turn.Integral=0;

    }
    turn.OutPut =  turn.Pout + turn.Dout + turn.Iout;       //����+΢���ܿ���
    turn.PreError = turn.Error;



}

float Turn_gryo(float gyro)//ת��
{

       turn_rate.Error =  turn.OutPut - gyro;                                 //���
       turn_rate.Differ = turn_rate.Error - turn_rate.PreError;                           //΢����

       turn_rate.Pout = turn_rate.P * turn_rate.Error;                        //��������
       turn_rate.Iout = turn_rate.I * turn_rate.Integral;  //���ֿ���
       turn_rate.Dout = turn_rate.D *turn_rate.Differ;                       //΢�ֿ���

       turn_rate.OutPut =  turn_rate.Pout +turn_rate.Iout/100 + turn_rate.Dout;       //���� + ���� + ΢���ܿ���

      // turn_rate.Integral+=turn_rate.Error;
       turn_rate.PreError = turn_rate.Error;
      return turn_rate.OutPut;


}
void Turn_suduhuan(int encoder)
{
    float Encoder_Least;
    Encoder_Least = (float)encoder*1.0;//��ǰ�ٶ��˲�
    //һ�׵�ͨ�˲�
    Encoder *= 0.8;//֮ǰ��������ֵռ70%
    Encoder += Encoder_Least*0.2;//��ǰ��30%

    turn_velocity.Error  = 0 - Encoder;                                 //���
    turn_velocity.Differ = turn_velocity.Error - turn_velocity.PreError;                           //΢����

    turn_velocity.Pout = turn_velocity.P * turn_velocity.Error;                        //��������
    turn_velocity.Iout = turn_velocity.I * turn_velocity.Integral;  //���ֿ���
    turn_velocity.Dout = turn_velocity.D * turn_velocity.Differ;                       //΢�ֿ���

    turn_velocity.OutPut =  turn_velocity.Pout/100 + turn_velocity.Iout/100 + turn_velocity.Dout;       //���� + ���� + ΢���ܿ���


    turn_velocity.PreError = turn_velocity.Error;
}
