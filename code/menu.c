#include "menu.h"

#define FLASH_SECTION_INDEX       (0)                                 // 存储数据用的扇区
#define FLASH_PAGE_INDEX          (11)                                // 存储数据用的页码 倒数第一个页码
/*
    flash的储存空间
    (1)页
    [0]~[8]:PID_para_you[3][3];
    [9]~[17]:PID_para_wu[3][3];
    (2)页
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

uint8 huandao_Manual;//左右环岛判断 0，自动判断 1，右环岛   2，左环岛
uint8 rump_Manual;//是否下坡减速
uint8 garge_Manual; // 0 左出库，左进库    1右出库，右出库
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
/*默认参数设置
 * bizhang 260000
 * OTS
 * SPEED 0,正常模式
 * 环岛 0 自动识别
 * rump 是否下坡减速
 *出库这个 看情况
 * backerror 看会赛道时摄像头误差×的系数，防止转向过猛
 * */
void flash_init(void)
{
          //flash_erase_page(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);//改自己数据时，把这行代码打开下载，下完后注释后再下一遍就是以刚刚调的数据开始调
            if(flash_check(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX))                      // 判断是否有数据
            {
                    flash_buffer_clear();
                    flash_read_page_to_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);           // 将数据从 flash 读取到缓冲区
                    enconder_MAX=flash_union_buffer[0].uint32_type;                             // 向缓冲区第 1 个位置写入  数据
                    image_thereshold=flash_union_buffer[1].uint8_type;                            // 向缓冲区第 2 个位置写入   数据                                 // 向缓冲区第 3 个位置写入  数据
                    speed_mode=flash_union_buffer[2].uint8_type;                                 // 向缓冲区第 4 个位置写入   数据
                    huandao_Manual= flash_union_buffer[3].uint8_type;                                    // 向缓冲区第 5 个位置写入   数据
                    rump_Manual=flash_union_buffer[4].uint8_type;                                   // 向缓冲区第 6 个位置写入    数据
                    garge_Manual=flash_union_buffer[5].uint8_type;
                    cam_inroad=flash_union_buffer[6].float_type;
            }
}

void set_value()
{

    if(menu_choice==Bizhang)//设置避障归一化最大值
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
    else if(menu_choice==OTS)//设置避障归一化最大值
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
    if (KEY3 == KEY_SHORT_PRESS||KEY4 == KEY_SHORT_PRESS)    // 写入数据
       {
           flash_buffer_clear();
           flash_union_buffer[0].uint32_type = enconder_MAX;                           // 向缓冲区第 1 个位置写入  数据
           flash_union_buffer[1].uint8_type = image_thereshold;                         // 向缓冲区第 2 个位置写入   数据                                 // 向缓冲区第 3 个位置写入  数据
           flash_union_buffer[2].uint8_type = speed_mode;                                 // 向缓冲区第 4 个位置写入   数据
           flash_union_buffer[3].uint8_type = huandao_Manual;                                    // 向缓冲区第 5 个位置写入   数据
           flash_union_buffer[4].uint8_type = rump_Manual;                                   // 向缓冲区第 6 个位置写入    数据
           flash_union_buffer[5].uint8_type = garge_Manual;
           flash_union_buffer[6].float_type = cam_inroad;
           flash_union_buffer[7].float_type = 0;
           flash_union_buffer[8].uint16_type = 0;
           flash_union_buffer[9].float_type = 0;




           if (flash_check(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX))                      // 判断是否有数据
               flash_erase_page(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);                // 擦除这一页
           flash_write_page_from_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);        // 向指定 Flash 扇区的页码写入缓冲区数据
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
