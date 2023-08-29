/*
 * motor.c
 *
 *  Created on: 2023��2��3��
 *      Author: Roronoa zoro
 */
#include "motor.h"
#define LEFT_FLYWHEEL_PWM               (ATOM0_CH2_P21_4        )     // ������PWM��������
#define LEFT_FLYWHEEL_DUTY_LEVEL        (0                      )     // ������ռ�ձ���Ч��ƽ
#define LEFT_FLYWHEEL_DIR               (P21_5                  )     // �����ַ����������
#define LEFT_FLYWHEEL_CLOCKWISE         (1                      )     // ������˳ʱ����ת����
#define LEFT_FLYWHEEL_BRAKE             (P02_4                  )     // ������ɲ����������
#define LEFT_FLYWHEEL_ENCODER_INDEX     (TIM5_ENCOEDER         )     // �����ֱ��������
#define LEFT_FLYWHEEL_ENCODER_CH1       (TIM5_ENCOEDER_CH1_P10_3)     // �����ֱ�����ͨ��1
#define LEFT_FLYWHEEL_ENCODER_CH2       (TIM5_ENCOEDER_CH2_P10_1)     // �����ֱ�����ͨ��2

#define RIGHT_FLYWHEEL_PWM              (ATOM0_CH5_P02_5        )     // �Ҳ����PWM��������
#define RIGHT_FLYWHEEL_DUTY_LEVEL       (0                      )     // �Ҳ����ռ�ձ���Ч��ƽ
#define RIGHT_FLYWHEEL_DIR              (P02_6                  )     // �Ҳ���ַ����������
#define RIGHT_FLYWHEEL_CLOCKWISE        (1                      )     // �Ҳ����˳ʱ����ת����
#define RIGHT_FLYWHEEL_BRAKE            (P02_7                  )     // �Ҳ����ɲ����������
#define RIGHT_FLYWHEEL_ENCODER_INDEX    (TIM6_ENCOEDER          )     // �Ҳ���ֱ��������
#define RIGHT_FLYWHEEL_ENCODER_CH1      (TIM6_ENCOEDER_CH1_P20_3)     // �Ҳ���ֱ�����ͨ��1
#define RIGHT_FLYWHEEL_ENCODER_CH2      (TIM6_ENCOEDER_CH2_P20_0)     // �Ҳ���ֱ�����ͨ��2
int16 encoder_data;
int16 PWM_Balance,PWM_Velocity;
#define ENCODER_1               (TIM4_ENCOEDER)
#define ENCODER_1_A             (TIM4_ENCOEDER_CH1_P02_8)
#define ENCODER_1_B             (TIM4_ENCOEDER_CH2_P00_9)

uint16 MOTOR_OUT_DEAD=800;
#define MAX_DUTY                (50)// ��� MAX_DUTY% ռ�ձ�
#define DIR                     (P21_2)
#define PWM                     (ATOM0_CH1_P21_3)
void motor_init(void)
{
    //��ˢ��ʼ��
    pwm_init (LEFT_FLYWHEEL_PWM, 25000, LEFT_FLYWHEEL_DUTY_LEVEL == 0 ? PWM_DUTY_MAX : 0);                  // ��ʼ��������PWM�ź�
    gpio_init(LEFT_FLYWHEEL_DIR, GPO, LEFT_FLYWHEEL_CLOCKWISE, GPO_PUSH_PULL);                              // ��ʼ��������DIR�ź�
    gpio_init(LEFT_FLYWHEEL_BRAKE, GPO, 1, GPO_PUSH_PULL);                                                  // ��ʼ��������ɲ���ź�
    encoder_quad_init(LEFT_FLYWHEEL_ENCODER_INDEX, LEFT_FLYWHEEL_ENCODER_CH1, LEFT_FLYWHEEL_ENCODER_CH2);   // ��ʼ�������ֱ������ӿ�

    pwm_init (RIGHT_FLYWHEEL_PWM, 25000, RIGHT_FLYWHEEL_DUTY_LEVEL == 0 ? PWM_DUTY_MAX : 0);                // ��ʼ���Ҳ����PWM�ź�
    gpio_init(RIGHT_FLYWHEEL_DIR, GPO, RIGHT_FLYWHEEL_CLOCKWISE, GPO_PUSH_PULL);                            // ��ʼ���Ҳ����DIR�ź�
    gpio_init(RIGHT_FLYWHEEL_BRAKE, GPO, 1, GPO_PUSH_PULL);                                                 // ��ʼ���Ҳ����ɲ���ź�
    encoder_quad_init(RIGHT_FLYWHEEL_ENCODER_INDEX, RIGHT_FLYWHEEL_ENCODER_CH1, RIGHT_FLYWHEEL_ENCODER_CH2);// ��ʼ���Ҳ���ֱ������ӿ�
    //�н���������ʼ��
    encoder_dir_init(ENCODER_1, ENCODER_1_A, ENCODER_1_B);// ��ʼ��������ģ�������� ������������ģʽ
    //DRV8701��ʼ��
    gpio_init(DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);// GPIO ��ʼ��Ϊ��� Ĭ�����������
    pwm_init(ATOM0_CH0_P21_2, 25000, 0);// PWM ͨ����ʼ��Ƶ�� 17KHz ռ�ձȳ�ʼΪ 0,LED2����
    pwm_init(PWM, 25000, 0);
}
void forward_pwm(int duty)
{
    //���ռ�ձ�8v���޷�
   // duty=-100;
    if(duty>9000)
    {
        duty=9000;
    }
    else if(duty<-9000)
    {
        duty=-9000;
    }
    if(duty>0)
    {
        duty+=MOTOR_OUT_DEAD;
        gpio_set_level(ATOM0_CH0_P21_2, 0);// DIR����ߵ�ƽ
        pwm_set_duty(ATOM0_CH1_P21_3, duty);// ����ռ�ձ�
       // pwm_set_duty(PWM, 0);// ����ռ�ձ�
    }
    else                                                                    // ��ת
    {
        duty=-duty;
        duty+=MOTOR_OUT_DEAD;
        //gpio_set_level(DIR, GPIO_LOW);// DIR����͵�ƽ
        pwm_set_duty(ATOM0_CH0_P21_2, duty);
        pwm_set_duty(ATOM0_CH1_P21_3, 0);// ����ռ�ձ�
        //pwm_set_duty(PWM, 0);// ����ռ�ձ�
    }


}

void Leftwheel_pwm(int duty)
{
    gpio_set_level(LEFT_FLYWHEEL_BRAKE, 1);// ɲ���ź�ȡ��
    if(duty > 0)// ����ռ�ձ��л���������
    {
//        duty+=MOTOR_OUT_DEAD;
        gpio_set_level(LEFT_FLYWHEEL_DIR, LEFT_FLYWHEEL_CLOCKWISE);
    }
    else
    {
//        duty-=MOTOR_OUT_DEAD;
        gpio_set_level(LEFT_FLYWHEEL_DIR, !LEFT_FLYWHEEL_CLOCKWISE);
    }
    pwm_set_duty(LEFT_FLYWHEEL_PWM,  LEFT_FLYWHEEL_DUTY_LEVEL  == 0 ? PWM_DUTY_MAX - func_abs(duty) : func_abs(duty));  // ���ռ�ձ� ռ�ձȱ���Ϊ��ֵ ��˴˴�ȡ�˾���ֵ
}
void Rightwheel_pwm(int duty)
{
   // gpio_set_level(LEFT_FLYWHEEL_BRAKE, 1);// ɲ���ź�ȡ��
    gpio_set_level(RIGHT_FLYWHEEL_BRAKE, 1);
    if(duty > 0)// ����ռ�ձ��л���������
    {
//        duty+=MOTOR_OUT_DEAD;
        gpio_set_level(RIGHT_FLYWHEEL_DIR, RIGHT_FLYWHEEL_CLOCKWISE);
    }
    else
    {
//        duty-=MOTOR_OUT_DEAD;
        gpio_set_level(RIGHT_FLYWHEEL_DIR, !RIGHT_FLYWHEEL_CLOCKWISE);
    }
    pwm_set_duty(RIGHT_FLYWHEEL_PWM, RIGHT_FLYWHEEL_DUTY_LEVEL == 0 ? PWM_DUTY_MAX - func_abs(duty) : func_abs(duty));
}
void shache(void)
{
    gpio_set_level(LEFT_FLYWHEEL_BRAKE, 0);// ɲ���ź�ȡ��
    gpio_set_level(RIGHT_FLYWHEEL_BRAKE, 0);
}
