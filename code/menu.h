/*
 * menu.h
 *
 *  Created on: 2023年1月24日
 *      Author: Roronoa zoro
 */

#ifndef CODE_MENU_H_
#define CODE_MENU_H_
#include "zf_common_headfile.h"

typedef struct
{
    uint8 current;     //当前状态索引号
    uint8 next;        //向下一个
    uint8 enter;       //确定
    uint8 back;        //返回
    void (*current_operation)(void); //当前状态应该执行的操作
} Menu_table;
extern uint8_t  func_index;
extern float X_zhongzhi;
extern float Y_zhongzhi;
extern float PID_para_wu[3][3];
extern float PID_para_you[3][3];
void  Menu_key_set(void);
void Menu_updata(void);
void Menu_flash(void);
enum
{
    Normol_speed,                                                           // 350
    High_speed,                                                            // 直道加速
    low_speed,                                                            // 300
};
#endif /* CODE_MENU_H_ */
