//-------------------------------------------------------------------------------------------------------------------
//  简介:八邻域图像处理
/*
 * kalman.h
 *
 *  Created on: 2023年5月1日
 *      Author: hanser
 */


//------------------------------------------------------------------------------------------------------------------
#include "img.h"

int jzh=59,sumMline;
int avrMline;
int zhongxin=39;
int x,y;
int Lline[60],Mline[60],Rline[60];
int L_lost_flag[60],R_lost_flag[60],yxh_flag,yxh;
uint8 curve_flag=0;
float curve_error=0;

int black;
uint8 quanzhi1[30]={
    5,5,5,5,5,//45
    10,10,10,10,10,//40
    30,30,30,30,30,//35
    20,20,20,20,20,
    1,1,1,1,1,//25
    1,1,1,1,1,//20
                   };
uint8 quanzhi2[30]={
    1,1,1,1,1,//45
    1,1,1,1,1,//40
    1,1,1,1,1,//35
    1,1,1,1,1,
    1,1,1,1,1,//25
    10,20,30,50,40,//20
                   };
int Mline_sum1,Mline_sum2,quanzhi_sum1,quanzhi_sum2;
float avr_Mline1;
int Mline_sum2,quanzhi_sum2,avr_Mline2;
float w_error1,w_error2;
int quan1_x,x1;

uint8 garage_flag_1[5];
uint8 garage_flag_2;
float garage_avg;
extern float Angle_z_temp;
//------------------------------------------------------------------------------------------------------------------
//  @brief     动态阈值
//  @since      v1.0
//------------------------------------------------------------------------------------------------------------------
uint8 otsuThreshold(uint8 *image, uint16 col, uint16 row)
{
#define GrayScale 256
    uint16 Image_Width  = col;
    uint16 Image_Height = row;
    int X; uint16 Y;
    uint8* data = image;
    int HistGram[GrayScale] = {0};

    uint32 Amount = 0;
    uint32 PixelBack = 0;
    uint32 PixelIntegralBack = 0;
    uint32 PixelIntegral = 0;
    int32 PixelIntegralFore = 0;
    int32 PixelFore = 0;
    double OmegaBack=0, OmegaFore=0, MicroBack=0, MicroFore=0, SigmaB=0, Sigma=0; // 类间方差;
    uint8 MinValue=0, MaxValue=0;
    uint8 Threshold = 0;


    for (Y = 0; Y <Image_Height; Y++) //Y<Image_Height改为Y =Image_Height；以便进行 行二值化
    {
        //Y=Image_Height;
        for (X = 0; X < Image_Width; X++)
        {
        HistGram[(int)data[Y*Image_Width + X]]++; //统计每个灰度值的个数信息
        }
    }




    for (MinValue = 0; MinValue < 256 && HistGram[MinValue] == 0; MinValue++) ;        //获取最小灰度的值
    for (MaxValue = 255; MaxValue > MinValue && HistGram[MinValue] == 0; MaxValue--) ; //获取最大灰度的值

    if (MaxValue == MinValue)
    {
        return MaxValue;          // 图像中只有一个颜色
    }
    if (MinValue + 1 == MaxValue)
    {
        return MinValue;      // 图像中只有二个颜色
    }

    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        Amount += HistGram[Y];        //  像素总数
    }

    PixelIntegral = 0;
    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        PixelIntegral += HistGram[Y] * Y;//灰度值总数
    }
    SigmaB = -1;
    for (Y = MinValue; Y < MaxValue; Y++)
    {
          PixelBack = PixelBack + HistGram[Y];    //前景像素点数
          PixelFore = Amount - PixelBack;         //背景像素点数
          OmegaBack = (double)PixelBack / Amount;//前景像素百分比
          OmegaFore = (double)PixelFore / Amount;//背景像素百分比
          PixelIntegralBack += HistGram[Y] * Y;  //前景灰度值
          PixelIntegralFore = PixelIntegral - PixelIntegralBack;//背景灰度值
          MicroBack = (double)PixelIntegralBack / PixelBack;//前景灰度百分比
          MicroFore = (double)PixelIntegralFore / PixelFore;//背景灰度百分比
          Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);//g
          if (Sigma > SigmaB)//遍历最大的类间方差g
          {
              SigmaB = Sigma;
              Threshold = (uint8)Y;
          }
    }
   return Threshold;
}
//------------------------------------------------------------------------------------------------------------------
//  @brief      图像二值化，这里用的是大津法二值化。
//  @since      v1.0
//------------------------------------------------------------------------------------------------------------------
uint8 bin_image[image_h][image_w];//图像数组
extern uint8 image_thereshold;
void turn_to_bin(void)
{
  uint8 i,j;
  black=0;
  for(i = 0;i<image_h;i++)
  {
      for(j = 0;j<image_w;j++)
      {
          if(mt9v03x_image[i][j]>image_thereshold)
              {
              mt9v03x_image[i][j] = white_pixel;
              }
          else
          {
              mt9v03x_image[i][j] = black_pixel;
              black++;
          }
      }
  }
}
//Mline,中线；

void jizhunhang()
{
    int a=0;
    zhongxin=avrMline;
    for(x=59;x>56;x--)
    {
        for(y=zhongxin;y>0;y--)
        {
            if(mt9v03x_image[x][y+1]==255&&mt9v03x_image[x][y]==0&&y!=Mline[x])
            {
                Lline[x]=y;
                L_lost_flag[x]=0;
                break;
            }
            else
            {
                Lline[x]=0;
                L_lost_flag[x]=1;
            }


        }
        for(y=zhongxin;y<79;y++)
        {
            if(mt9v03x_image[x][y-1]==255&&mt9v03x_image[x][y]==0&&y!=Mline[x])
            {
                Rline[x]=y;
                R_lost_flag[x]=0;
                break;
            }
            else
            {
                Rline[x]=79;
                R_lost_flag[x]=1;
            }

        }



           Mline[x]=(Rline[x]+Lline[x])/2;
           sumMline+=Mline[x];
           a=a+1;

    }
    if(x==56)
    {
        avrMline=sumMline/a;
        a=0;
        sumMline=0;
        Mline[57]=avrMline;
    }

}
void garage()
{
    garage_avg=0;
    garage_flag_2=0;
    for(x=50;x>45;x--)
    {
        for(y=3;y<75;y++)
        {
            if(mt9v03x_image[x][y-2]==0&&mt9v03x_image[x][y]==0&&mt9v03x_image[x][y+1]==255&&mt9v03x_image[x][y+3]==255)//黑白跳变点
            {
                garage_flag_1[50-x]++;//每一行看看有几个跳变点
            }
        }
    }
    for(x=0;x<5;x++)
    {
        garage_avg+=garage_flag_1[x];//给跳变点求个和
    }
    for(x=0;x<5;x++)
    {
        if(garage_avg>20)//如果没有识别到这个就把20改小点
        {
            if((garage_avg/5-garage_flag_1[x]<3)&&(garage_avg/5-garage_flag_1[3]>-3))//如果识别不了就把这个if去掉
            {
                garage_flag_2+=1;
                Angle_z_temp=0;

            }
        }
        garage_flag_1[x]=0;
    }


}

void findline()
{
    yxh_flag=0;

    for(x=56;x>20;x--)
    {
        if(x>20&&(mt9v03x_image[x][Mline[x]-2]==255||mt9v03x_image[x][Mline[x]+2]==255||Mline[x]<3||Mline[x]>77))
        yxh_flag=x;       /// 有效行判断
        for(y=Mline[x+1];y>0;y--)
        {
            if(mt9v03x_image[x][y]==0&&mt9v03x_image[x][y+1]==255&&y!=Mline[x])
            {
                Lline[x]=y;
                L_lost_flag[x]=0;
                break;
            }
            else
            {
                Lline[x]=0;
                L_lost_flag[x]=1;
            }
        }

        for(y=Mline[x+1];y<79;y++)
        {
            if(mt9v03x_image[x][y]==0&&mt9v03x_image[x][y-1]==255&&y!=Mline[x])
            {
                Rline[x]=y;
                R_lost_flag[x]=0;
                break;
            }
            else
            {
                Rline[x]=79;
                R_lost_flag[x]=1;
            }
       }

       Mline[x]=(Lline[x]+Rline[x])/2;

       if(abs(Mline[x]-Mline[x+1])>3&&abs(Mline[x+1]-Mline[x+2])>3)
           Mline[x]=(Mline[x]+Mline[x+1])/2;   ///中线防干扰
       if(L_lost_flag[x]==1&&R_lost_flag[x]==1)
           Mline[x]=Mline[x+1];

       if(yxh_flag>30)
           yxh=30;
       else
           yxh=yxh_flag;      ////有效行限制

    }


    for(x=57;x>59-yxh;x--)
    {
        Mline[x] = Mline[x+1]*0.3 + Mline[x]*0.7;
    }
    for(x=59-yxh;x<yxh;x++)
    {
        Mline[x] = Mline[x-1]*0.3 + Mline[x]*0.7;
    }

    for(x=57;x>20;x--)
    {
        mt9v03x_image[x][Mline[x]]=0;
        mt9v03x_image[47][43]=0;
        mt9v03x_image[47][35]=0;

    }
}
void jiaquan_error1()
{

    for(x1=50;x1>yxh;x1--)
    {

        Mline_sum1+=(Mline[x1]-39)*quanzhi1[quan1_x];
        Mline_sum2+=(Mline[x1]-39)*quanzhi2[quan1_x];

        quanzhi_sum1+=quanzhi1[quan1_x];
        quanzhi_sum2+=quanzhi2[quan1_x];
        quan1_x++;
    }

    if(x1==yxh)
    {
        avr_Mline1=Mline_sum1/quanzhi_sum1;
        curve_error=Mline_sum2/quanzhi_sum2;
        w_error1=avr_Mline1;
        Mline_sum1=0;
        Mline_sum2=0;
        quanzhi_sum1=0;
        quanzhi_sum2=0;
        quan1_x=0;
        tft180_show_uint(90,90,image_thereshold,3);
    }


}


