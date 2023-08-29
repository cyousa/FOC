#ifndef _IMAGE_H
#define _IMAGE_H


//��������������������ֲ������ֲ��ʱ�����ɾ�����ĳ����Լ��ģ�
/*
 * kalman.h
 *
 *  Created on: 2023��5��1��
 *      Author: hanser
 */

//��ɫ����  ��Ϊ������������ɫ���Ĳ�������ֱ�ӷ�����
#define uesr_RED     0XF800    //��ɫ
#define uesr_GREEN   0X07E0    //��ɫ
#define uesr_BLUE    0X001F    //��ɫ

#include "zf_device_mt9v03x.h"


//�궨��
#define image_h 60//ͼ��߶�
#define image_w 80//ͼ����

#define white_pixel 255
#define black_pixel 0

#define bin_jump_num    1//�����ĵ���
#define border_max  image_w-2 //�߽����ֵ
#define border_min  1   //�߽���Сֵ
extern uint8 original_image[image_h][image_w];
extern uint8 bin_image[image_h][image_w];//ͼ������

extern void image_process(void); //ֱ�����жϻ�ѭ������ô˳���Ϳ���ѭ��ִ����



#endif /*_IMAGE_H*/

