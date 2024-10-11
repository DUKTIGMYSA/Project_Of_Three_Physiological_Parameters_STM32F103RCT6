/*********************************************************************************************************
* 模块名称：ProcKeyOne.h
* 摘    要：ProcKeyOne模块，进行独立按键处理模块初始化，以及独立按键处理函数实现
* 当前版本：1.0.0
* 作    者：SZLY(COPYRIGHT 2018 - 2020 SZLY. All rights reserved.)
* 完成日期：2020年01月01日    
* 内    容：
* 注    意：                                                                  
**********************************************************************************************************
* 取代版本：
* 作    者：
* 完成日期：
* 修改内容：
* 修改文件：
*********************************************************************************************************/
#ifndef _PROC_KEY_ONE_H_
#define _PROC_KEY_ONE_H_

/*********************************************************************************************************
*                                              包含头文件
*********************************************************************************************************/
#include "stm32f10x_conf.h"
#include "DataType.h"
#include "UART1.h"


/*********************************************************************************************************
*                                              宏定义
*********************************************************************************************************/

/*********************************************************************************************************
*                                              枚举结构体定义
*********************************************************************************************************/
typedef enum
{
  T_PA = 1,
  T_PB,
  T_SENS1,
  T_SENS2
}EnumKEYModel;

/*********************************************************************************************************
*                                              API函数声明
*********************************************************************************************************/
void  InitProcKeyOne(void);   //初始化ProcKeyOne模块

void  ProcKeyDownKey1(void);  //处理按键按下的事件，即按键按下的响应函数 
void  ProcKeyUpKey1(void);    //处理按键弹起的事件，即按键弹起的响应函数
void  ProcKeyDownKey2(void);  //处理按键按下的事件，即按键按下的响应函数 
void  ProcKeyUpKey2(void);    //处理按键弹起的事件，即按键弹起的响应函数
void  ProcKeyDownKey3(void);  //处理按键按下的事件，即按键按下的响应函数 
void  ProcKeyUpKey3(void);    //处理按键弹起的事件，即按键弹起的响应函数
u8 BoolKeyRenew(void);        //检测通道是否被切换成检验通道 
u8 BoolKeyNum(void);          //判断检测通道是1还是2 
i8 BoolKeyModel(void);        //判断是位于哪个通道
u8 GetStateNum(void);         //获取三参状态
#endif
