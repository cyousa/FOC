/*
 * menu.h
 *
 *  Created on: 2023��1��24��
 *      Author: Roronoa zoro
 */

#ifndef CODE_MENU_H_
#define CODE_MENU_H_
#include "zf_common_headfile.h"

typedef struct
{
    uint8 current;     //��ǰ״̬������
    uint8 next;        //����һ��
    uint8 enter;       //ȷ��
    uint8 back;        //����
    void (*current_operation)(void); //��ǰ״̬Ӧ��ִ�еĲ���
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
    High_speed,                                                            // ֱ������
    low_speed,                                                            // 300
};
#endif /* CODE_MENU_H_ */
