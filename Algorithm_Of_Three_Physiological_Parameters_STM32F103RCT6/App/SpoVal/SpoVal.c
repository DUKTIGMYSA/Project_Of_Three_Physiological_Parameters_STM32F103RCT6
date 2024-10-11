/*********************************************************************************************************
* 模块名称：SpoVal.c
* 摘    要：RunClock模块
* 当前版本：1.0.0
* 作    者：SZLY(COPYRIGHT 2018 - 2020 SZLY. All rights reserved.)
* 完成日期：2020年01月01日 
* 内    容：
* 注    意：none                                                                  
**********************************************************************************************************
* 取代版本：
* 作    者：
* 完成日期：
* 修改内容：
* 修改文件：
*********************************************************************************************************/

/*********************************************************************************************************
*                                              包含头文件
*********************************************************************************************************/
#include "SpoVal.h"
#include <stdio.h>

/*********************************************************************************************************
*                                              宏定义
*********************************************************************************************************/

/*********************************************************************************************************
*                                              内部变量
*********************************************************************************************************/
 
/*********************************************************************************************************
*                                              内部函数声明
*********************************************************************************************************/

/*********************************************************************************************************
*                                              内部函数实现
*********************************************************************************************************/
/*********************************************************************************************************
*                                              API函数实现
*********************************************************************************************************/
/*********************************************************************************************************
* 函数名称：InitPluse
* 函数功能：初始化RunClock模块 
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2018年01月01日
* 注    意：
*********************************************************************************************************/
void  InitSpoVal(void)
{

}

/*********************************************************************************************************
* 函数名称：SpoValNum
* 函数功能：计算血氧的有效值
* 输入参数：
*  			RED_penData			血氧红光采样数组（300个）
*			IR_penData			血氧红外采样数组（300个）
* 输出参数：void
* 返 回 值：Spo2				血氧有效值
* 创建日期：2018年01月01日
* 注    意：
*********************************************************************************************************/
i16 SpoValNum( u16* RED_penData, u16* IR_penData)
{
	i16 Spo2 = 0;         //血氧数值
  u16 RED_MAX = 0;      //红光血氧最大值
  u16 RED_MIN = 4095;   //红光血氧最小值
  u16 IR_MAX = 0;       //红外血氧最大值
  u16 IR_MIN = 4095;    //红光血氧最小值
  double R = 0;            //血氧R值的中间值
  u16 r_R = 0;          //R值的最终值
	u16 i=0;
  
  for(i=0;i<300;i++)  //最大值最小值计算
  {
     if(RED_MIN > RED_penData[i] && RED_penData[i] > 100)      //当RED_MIN大于当前数组的值时，更换RED_MIN
    {
      RED_MIN = RED_penData[i];
    }
    if(RED_MAX < RED_penData[i] && RED_penData[i] < 2000)      //当RED_MAX小于当前数组的值时，更换RED_MAX
    {
      RED_MAX = RED_penData[i];
    }
    if(IR_MIN > IR_penData[i] && IR_penData[i] > 200)        //当IR_MIN大于当前数组的值时，更换IR_MIN
    {
      IR_MIN = IR_penData[i];
    }
    if(IR_MAX < IR_penData[i] && RED_penData[i] < 2000)        //当IR_MAX小于当前数组的值时，更换IR_MAX
    {
      IR_MAX = IR_penData[i];
    }
  }
  
  
//  RED_Rng = RED_MAX - RED_MIN;
//  IR_Rng = IR_MAX - IR_MIN;
  R = ((RED_MAX - RED_MIN)*(IR_MAX+IR_MIN))*1.00/((IR_MAX - IR_MIN)*(RED_MAX+RED_MIN))*1.00;
  r_R = R*1000.00;
  RED_MAX = 0;
  RED_MIN = 4095;
  IR_MAX = 0;
  IR_MIN = 4095;

  //血氧数据的判断
  if(r_R <= 640)
  {
    Spo2 = 99;
  }
  else if (640 <r_R && r_R<=680)
  {
    Spo2 = 98;
  }
  else if (680 <r_R && r_R<=720)
  {
    Spo2 = 97;
  }
  else if (720 <r_R && r_R<=750)
  {
    Spo2 = 96;
  }
  else if (750 <r_R && r_R<=780)
  {
    Spo2 = 95;
  }
  else if (780 <r_R && r_R<=810)
  {
    Spo2 = 94;
  }
  else if (810 <r_R && r_R<=840)
  {
    Spo2 = 93;
  }
  else if (840 <r_R && r_R<=860)
  {
    Spo2 = 92;
  }
  else if (860 <r_R && r_R<=880)
  {
    Spo2 = 91;
  }
  else if (880 <r_R && r_R<=910)
  {
    Spo2 = 90;
  }
  else if (910 <r_R && r_R<=940)
  {
    Spo2 = 89;
  }
  else if (940 <r_R && r_R<=970)
  {
    Spo2 = 88;
  }
  else if (970 <r_R && r_R<=1000)
  {
    Spo2 = 87;
  }
  else if (1000 <r_R && r_R<=1030)
  {
    Spo2 = 86;
  }
  else if (1030 <r_R && r_R<=1060)
  {
    Spo2 = 85;
  }
  else if (1060 <r_R && r_R<=1080)
  {
    Spo2 = 84;
  }
  else if (1080 <r_R && r_R<=1100)
  {
    Spo2 = 83;
  }
  else if (1100 <r_R && r_R<=1120)
  {
    Spo2 = 82;
  }
  else if (1120 <r_R && r_R<=1150)
  {
    Spo2 = 81;
  }
  else if (1150 <r_R && r_R<=1170)
  {
    Spo2 = 80;
  }
  else if (1170 <r_R && r_R<=1200)
  {
    Spo2 = 79;
  }    
  else
  {
    Spo2 = -100;
  }
  return Spo2;
}


