/*
 * motor.c
 *
 *  Created on: 2023年2月3日
 *      Author: Roronoa zoro
 */
#include "motor.h"
#define LEFT_FLYWHEEL_PWM               (ATOM0_CH2_P21_4        )     // 左侧飞轮PWM控制引脚
#define LEFT_FLYWHEEL_DUTY_LEVEL        (0                      )     // 左侧飞轮占空比有效电平
#define LEFT_FLYWHEEL_DIR               (P21_5                  )     // 左侧飞轮方向控制引脚
#define LEFT_FLYWHEEL_CLOCKWISE         (1                      )     // 左侧飞轮顺时针旋转方向
#define LEFT_FLYWHEEL_BRAKE             (P02_4                  )     // 左侧飞轮刹车控制引脚
#define LEFT_FLYWHEEL_ENCODER_INDEX     (TIM5_ENCOEDER         )     // 左侧飞轮编码器编号
#define LEFT_FLYWHEEL_ENCODER_CH1       (TIM5_ENCOEDER_CH1_P10_3)     // 左侧飞轮编码器通道1
#define LEFT_FLYWHEEL_ENCODER_CH2       (TIM5_ENCOEDER_CH2_P10_1)     // 左侧飞轮编码器通道2

#define RIGHT_FLYWHEEL_PWM              (ATOM0_CH5_P02_5        )     // 右侧飞轮PWM控制引脚
#define RIGHT_FLYWHEEL_DUTY_LEVEL       (0                      )     // 右侧飞轮占空比有效电平
#define RIGHT_FLYWHEEL_DIR              (P02_6                  )     // 右侧飞轮方向控制引脚
#define RIGHT_FLYWHEEL_CLOCKWISE        (1                      )     // 右侧飞轮顺时针旋转方向
#define RIGHT_FLYWHEEL_BRAKE            (P02_7                  )     // 右侧飞轮刹车控制引脚
#define RIGHT_FLYWHEEL_ENCODER_INDEX    (TIM6_ENCOEDER          )     // 右侧飞轮编码器编号
#define RIGHT_FLYWHEEL_ENCODER_CH1      (TIM6_ENCOEDER_CH1_P20_3)     // 右侧飞轮编码器通道1
#define RIGHT_FLYWHEEL_ENCODER_CH2      (TIM6_ENCOEDER_CH2_P20_0)     // 右侧飞轮编码器通道2
int16 encoder_data;
int16 PWM_Balance,PWM_Velocity;
#define ENCODER_1               (TIM4_ENCOEDER)
#define ENCODER_1_A             (TIM4_ENCOEDER_CH1_P02_8)
#define ENCODER_1_B             (TIM4_ENCOEDER_CH2_P00_9)

uint16 MOTOR_OUT_DEAD=800;
#define MAX_DUTY                (50)// 最大 MAX_DUTY% 占空比
#define DIR                     (P21_2)
#define PWM                     (ATOM0_CH1_P21_3)
void motor_init(void)
{
    //无刷初始化
    pwm_init (LEFT_FLYWHEEL_PWM, 25000, LEFT_FLYWHEEL_DUTY_LEVEL == 0 ? PWM_DUTY_MAX : 0);                  // 初始化左侧飞轮PWM信号
    gpio_init(LEFT_FLYWHEEL_DIR, GPO, LEFT_FLYWHEEL_CLOCKWISE, GPO_PUSH_PULL);                              // 初始化左侧飞轮DIR信号
    gpio_init(LEFT_FLYWHEEL_BRAKE, GPO, 1, GPO_PUSH_PULL);                                                  // 初始化左侧飞轮刹车信号
    encoder_quad_init(LEFT_FLYWHEEL_ENCODER_INDEX, LEFT_FLYWHEEL_ENCODER_CH1, LEFT_FLYWHEEL_ENCODER_CH2);   // 初始化左侧飞轮编码器接口

    pwm_init (RIGHT_FLYWHEEL_PWM, 25000, RIGHT_FLYWHEEL_DUTY_LEVEL == 0 ? PWM_DUTY_MAX : 0);                // 初始化右侧飞轮PWM信号
    gpio_init(RIGHT_FLYWHEEL_DIR, GPO, RIGHT_FLYWHEEL_CLOCKWISE, GPO_PUSH_PULL);                            // 初始化右侧飞轮DIR信号
    gpio_init(RIGHT_FLYWHEEL_BRAKE, GPO, 1, GPO_PUSH_PULL);                                                 // 初始化右侧飞轮刹车信号
    encoder_quad_init(RIGHT_FLYWHEEL_ENCODER_INDEX, RIGHT_FLYWHEEL_ENCODER_CH1, RIGHT_FLYWHEEL_ENCODER_CH2);// 初始化右侧飞轮编码器接口
    //行进编码器初始化
    encoder_dir_init(ENCODER_1, ENCODER_1_A, ENCODER_1_B);// 初始化编码器模块与引脚 方向解码编码器模式
    //DRV8701初始化
    gpio_init(DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);// GPIO 初始化为输出 默认上拉输出高
    pwm_init(ATOM0_CH0_P21_2, 25000, 0);// PWM 通道初始化频率 17KHz 占空比初始为 0,LED2会亮
    pwm_init(PWM, 25000, 0);
}
void forward_pwm(int duty)
{
    //最大占空比8v，限幅
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
        gpio_set_level(ATOM0_CH0_P21_2, 0);// DIR输出高电平
        pwm_set_duty(ATOM0_CH1_P21_3, duty);// 计算占空比
       // pwm_set_duty(PWM, 0);// 计算占空比
    }
    else                                                                    // 反转
    {
        duty=-duty;
        duty+=MOTOR_OUT_DEAD;
        //gpio_set_level(DIR, GPIO_LOW);// DIR输出低电平
        pwm_set_duty(ATOM0_CH0_P21_2, duty);
        pwm_set_duty(ATOM0_CH1_P21_3, 0);// 计算占空比
        //pwm_set_duty(PWM, 0);// 计算占空比
    }


}

void Leftwheel_pwm(int duty)
{
    gpio_set_level(LEFT_FLYWHEEL_BRAKE, 1);// 刹车信号取消
    if(duty > 0)// 根据占空比切换方向引脚
    {
//        duty+=MOTOR_OUT_DEAD;
        gpio_set_level(LEFT_FLYWHEEL_DIR, LEFT_FLYWHEEL_CLOCKWISE);
    }
    else
    {
//        duty-=MOTOR_OUT_DEAD;
        gpio_set_level(LEFT_FLYWHEEL_DIR, !LEFT_FLYWHEEL_CLOCKWISE);
    }
    pwm_set_duty(LEFT_FLYWHEEL_PWM,  LEFT_FLYWHEEL_DUTY_LEVEL  == 0 ? PWM_DUTY_MAX - func_abs(duty) : func_abs(duty));  // 输出占空比 占空比必须为正值 因此此处取了绝对值
}
void Rightwheel_pwm(int duty)
{
   // gpio_set_level(LEFT_FLYWHEEL_BRAKE, 1);// 刹车信号取消
    gpio_set_level(RIGHT_FLYWHEEL_BRAKE, 1);
    if(duty > 0)// 根据占空比切换方向引脚
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
    gpio_set_level(LEFT_FLYWHEEL_BRAKE, 0);// 刹车信号取消
    gpio_set_level(RIGHT_FLYWHEEL_BRAKE, 0);
}
