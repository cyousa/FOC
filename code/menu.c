#include "menu.h"

#define FLASH_SECTION_INDEX       (0)                                 // �洢�����õ�����
#define FLASH_PAGE_INDEX          (11)                                // �洢�����õ�ҳ�� ������һ��ҳ��
/*
    flash�Ĵ���ռ�
    (1)ҳ
    [0]~[8]:PID_para_you[3][3];
    [9]~[17]:PID_para_wu[3][3];
    (2)ҳ
    [0]:X_zhongzhi
    [1]:Y_zhongzhi
*/

uint8 KEY1;
uint8 KEY2;
uint8 KEY3;
uint8 KEY4;

extern float left_adc_1;
extern float right_adc_2;
extern float right_adc_1;
extern float Mid_adc_1;

uint8 menu_choice;

extern uint32 enconder_MAX;
extern uint8 image_thereshold;

uint8 huandao_Manual;//���һ����ж� 0���Զ��ж� 1���һ���   2���󻷵�
uint8 rump_Manual;//�Ƿ����¼���
uint8 garge_Manual; // 0 ����⣬�����    1�ҳ��⣬�ҳ���
extern float weight;


extern uint8 image_thereshold;


uint8 speed_mode;
extern float cam_inroad;


enum Menu
{
    Bizhang,
    OTS,
    SPEED,
    huandao,
    rump,
    out_garge,
    back_error,
    ADC_Value

};
/*Ĭ�ϲ�������
 * bizhang 260000
 * OTS
 * SPEED 0,����ģʽ
 * ���� 0 �Զ�ʶ��
 * rump �Ƿ����¼���
 *������� �����
 * backerror ��������ʱ����ͷ������ϵ������ֹת�����
 * */
void flash_init(void)
{
          //flash_erase_page(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);//���Լ�����ʱ�������д�������أ������ע�ͺ�����һ������Ըոյ������ݿ�ʼ��
            if(flash_check(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX))                      // �ж��Ƿ�������
            {
                    flash_buffer_clear();
                    flash_read_page_to_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);           // �����ݴ� flash ��ȡ��������
                    enconder_MAX=flash_union_buffer[0].uint32_type;                             // �򻺳����� 1 ��λ��д��  ����
                    image_thereshold=flash_union_buffer[1].uint8_type;                            // �򻺳����� 2 ��λ��д��   ����                                 // �򻺳����� 3 ��λ��д��  ����
                    speed_mode=flash_union_buffer[2].uint8_type;                                 // �򻺳����� 4 ��λ��д��   ����
                    huandao_Manual= flash_union_buffer[3].uint8_type;                                    // �򻺳����� 5 ��λ��д��   ����
                    rump_Manual=flash_union_buffer[4].uint8_type;                                   // �򻺳����� 6 ��λ��д��    ����
                    garge_Manual=flash_union_buffer[5].uint8_type;
                    cam_inroad=flash_union_buffer[6].float_type;
            }
}

void set_value()
{

    if(menu_choice==Bizhang)//���ñ��Ϲ�һ�����ֵ
    {
        if (KEY3 == KEY_SHORT_PRESS)
        {
            enconder_MAX+=500;
            tft180_clear();

        }
        else if (KEY4 == KEY_SHORT_PRESS )
        {
            enconder_MAX-=500;
            tft180_clear();
        }

    }
    else if(menu_choice==OTS)//���ñ��Ϲ�һ�����ֵ
    {
        if (KEY3 == KEY_SHORT_PRESS)
        {
            image_thereshold+=1;
            tft180_clear();

        }
        else if (KEY4 == KEY_SHORT_PRESS )
        {
            image_thereshold-=1;
            tft180_clear();
        }

    }
    else if(menu_choice==SPEED)
    {

            if (KEY3 == KEY_SHORT_PRESS)
            {
                speed_mode+=1;
                tft180_clear();

            }
            else if (KEY4 == KEY_SHORT_PRESS )
            {
                speed_mode-=1;
                tft180_clear();
            }
            if(speed_mode>3)
            {
                speed_mode=3;
            }

    }
    else if(menu_choice==huandao)
    {

            if (KEY3 == KEY_SHORT_PRESS)
            {
                huandao_Manual+=1;
                tft180_clear();

            }
            else if (KEY4 == KEY_SHORT_PRESS )
            {
                huandao_Manual-=1;
                tft180_clear();
            }
            if(huandao_Manual>2)
            {
                huandao_Manual=2;
            }

    }
    else if(menu_choice==rump)
    {

            if (KEY3 == KEY_SHORT_PRESS)
            {
                rump_Manual+=1;
                tft180_clear();

            }
            else if (KEY4 == KEY_SHORT_PRESS )
            {
                rump_Manual-=1;
                tft180_clear();
            }
            if(rump_Manual>1)
            {
                rump_Manual=1;
            }

    }
    else if(menu_choice==out_garge)
    {

            if (KEY3 == KEY_SHORT_PRESS)
            {
                garge_Manual+=1;
                tft180_clear();

            }
            else if (KEY4 == KEY_SHORT_PRESS )
            {
                garge_Manual-=1;
                tft180_clear();
            }
            if(garge_Manual>2)
            {
                garge_Manual=2;
            }

    }
    else if(menu_choice==back_error)
    {

            if (KEY3 == KEY_SHORT_PRESS)
            {
                cam_inroad+=0.1;
                tft180_clear();

            }
            else if (KEY4 == KEY_SHORT_PRESS )
            {
                cam_inroad-=0.1;
                tft180_clear();
            }
            if(cam_inroad>1)
            {
                cam_inroad=1;
            }

    }
    if (KEY3 == KEY_SHORT_PRESS||KEY4 == KEY_SHORT_PRESS)    // д������
       {
           flash_buffer_clear();
           flash_union_buffer[0].uint32_type = enconder_MAX;                           // �򻺳����� 1 ��λ��д��  ����
           flash_union_buffer[1].uint8_type = image_thereshold;                         // �򻺳����� 2 ��λ��д��   ����                                 // �򻺳����� 3 ��λ��д��  ����
           flash_union_buffer[2].uint8_type = speed_mode;                                 // �򻺳����� 4 ��λ��д��   ����
           flash_union_buffer[3].uint8_type = huandao_Manual;                                    // �򻺳����� 5 ��λ��д��   ����
           flash_union_buffer[4].uint8_type = rump_Manual;                                   // �򻺳����� 6 ��λ��д��    ����
           flash_union_buffer[5].uint8_type = garge_Manual;
           flash_union_buffer[6].float_type = cam_inroad;
           flash_union_buffer[7].float_type = 0;
           flash_union_buffer[8].uint16_type = 0;
           flash_union_buffer[9].float_type = 0;




           if (flash_check(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX))                      // �ж��Ƿ�������
               flash_erase_page(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);                // ������һҳ
           flash_write_page_from_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);        // ��ָ�� Flash ������ҳ��д�뻺��������
       }









}


void Key_Menu()
{
    key_scanner();
    KEY1 = key_get_state(KEY_1);
    KEY2 = key_get_state(KEY_2);
    KEY3 = key_get_state(KEY_3);
    KEY4 = key_get_state(KEY_4);
    if (KEY1 == KEY_SHORT_PRESS && menu_choice < 8)
    {
        menu_choice++;
        tft180_clear();

    }
    else if (KEY2 == KEY_SHORT_PRESS && menu_choice > 0)
    {
        menu_choice--;
        tft180_clear();
    }
    set_value();
    tft180_show_uint(90,40,menu_choice,1);
}

void menu_show()
{
    switch (menu_choice)
    {
    case Bizhang:

        tft180_show_string(90, 0, "bizhang");
        tft180_show_int(90,15,enconder_MAX,6);
        break;
    case OTS:

        tft180_show_string(90, 0, "OTS");
        tft180_show_uint(90,15,image_thereshold,3);
        break;
    case SPEED:

        tft180_show_string(90, 0, "SPEED");
        tft180_show_uint(90,15,speed_mode,3);
        break;
    case huandao:

        tft180_show_string(90, 0, "huandao");
        tft180_show_uint(90,15,huandao_Manual,3);
        break;

    case rump:

        tft180_show_string(90, 0, "rump");
        tft180_show_uint(90,15,rump_Manual,3);
        break;
    case out_garge:

        tft180_show_string(90, 0, "garge");
        tft180_show_uint(90,15,garge_Manual,3);
        break;
    case back_error:

        tft180_show_string(90, 0, "bk_error");
        tft180_show_float(90,15,cam_inroad,1,1);
        break;
    case ADC_Value:

        tft180_show_string(90, 0, "ADC_V");
        tft180_show_float(105,15,right_adc_1,2,1);
        tft180_show_float(105,30,right_adc_2,2,1);
        tft180_show_float(105,45,Mid_adc_1,2,1);
        tft180_show_float(105,60,left_adc_1,2,1);
        break;


    }

}
