/*********************************************************************************************************
* 模块名称：LED.c
* 摘    要：LED模块
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

/*********************************************************************************************************
*                                              包含头文件
*********************************************************************************************************/
#include "LED.h"
#include "stm32f10x_conf.h"
/*********************************************************************************************************
*                                              宏定义
*********************************************************************************************************/

/*********************************************************************************************************
*                                              枚举结构体定义
*********************************************************************************************************/

/*********************************************************************************************************
*                                              内部变量
*********************************************************************************************************/

/*********************************************************************************************************
*                                              内部函数声明
*********************************************************************************************************/
static  void  ConfigLEDGPIO(void);  //配置LED的GPIO

/*********************************************************************************************************
*                                              内部函数实现
*********************************************************************************************************/
/*********************************************************************************************************
* 函数名称：ConfigLEDGPIO
* 函数功能：配置LED的GPIO 
* 输入参数：void 
* 输出参数：void
* 返 回 值：void
* 创建日期：2018年01月01日
* 注    意：
*********************************************************************************************************/
static  void  ConfigLEDGPIO(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;  //GPIO_InitStructure用于存放GPIO的参数
                                                                     
  //使能RCC相关时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); //使能GPIOC的时钟
                                                                                                                 
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_14;           //设置引脚
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //设置I/O输出速度
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;     //设置模式
  GPIO_Init(GPIOC, &GPIO_InitStructure);                //根据参数初始化LED1的GPIO

  GPIO_WriteBit(GPIOC, GPIO_Pin_14, Bit_SET);            //将LED1默认状态设置为点亮

  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_15;           //设置引脚
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //设置I/O输出速度
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;     //设置模式
  GPIO_Init(GPIOC, &GPIO_InitStructure);                //根据参数初始化LED2的GPIO

  GPIO_WriteBit(GPIOC, GPIO_Pin_15, Bit_SET);          //将LED2默认状态设置为熄灭
  
}

/*********************************************************************************************************
*                                              API函数实现
*********************************************************************************************************/
/*********************************************************************************************************
* 函数名称：InitLED
* 函数功能：初始化LED模块
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2018年01月01日
* 注    意：
*********************************************************************************************************/
void InitLED(void)
{
  ConfigLEDGPIO();  //配置LED的GPIO
}

/*********************************************************************************************************
* 函数名称：LEDFlicker
* 函数功能：LED闪烁函数
* 输入参数：cnt
* 输出参数：void
* 返 回 值：void
* 创建日期：2018年01月01日
* 注    意：LEDFlicker在Proc2msTask中调用，cnt为250时表示每500ms更改一次LED状态
*********************************************************************************************************/
void LEDFlicker(EnumKEYModel KEYModel)
{
  if(KEYModel == T_PA)      // LED1与LED2都亮
  {
    GPIO_WriteBit(GPIOC, GPIO_Pin_14, Bit_RESET);
    GPIO_WriteBit(GPIOC, GPIO_Pin_15, Bit_RESET);
  }
  else if(KEYModel == T_PB) // LED1与LED2都灭
  {
    GPIO_WriteBit(GPIOC, GPIO_Pin_14, Bit_SET);
    GPIO_WriteBit(GPIOC, GPIO_Pin_15, Bit_SET);
  }
  else if(KEYModel == T_SENS1)
  {
    // LED1亮，LED2灭
    GPIO_WriteBit(GPIOC, GPIO_Pin_14, Bit_RESET);
    GPIO_WriteBit(GPIOC, GPIO_Pin_15, Bit_SET);
  }
  else if(KEYModel == T_SENS2)
  {
    // LED1灭，LED2亮
    GPIO_WriteBit(GPIOC, GPIO_Pin_14, Bit_SET);
    GPIO_WriteBit(GPIOC, GPIO_Pin_15, Bit_RESET);
  }
}

/*********************************************************************************************************
* 函数名称：ConfigSPO2LEDGPIO
* 函数功能：配置血氧模块血氧探头LED的GPIO 
* 输入参数：void 
* 输出参数：void
* 返 回 值：void
* 创建日期：2018年01月01日
* 注    意：
*********************************************************************************************************/
void  ConfigSPO2LEDGPIO(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;  //GPIO_InitStructure用于存放GPIO的参数
                                                                     
  //使能RCC相关时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //使能GPIOC的时钟
                                                                                                                 
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;           //设置引脚
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //设置I/O输出速度
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;     //设置模式
  GPIO_Init(GPIOA, &GPIO_InitStructure);                //根据参数初始化LED1的GPIO

  GPIO_WriteBit(GPIOA, GPIO_Pin_2, Bit_SET);            //将LED1默认状态设置为点亮

  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;           //设置引脚
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //设置I/O输出速度
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;     //设置模式
  GPIO_Init(GPIOA, &GPIO_InitStructure);                //根据参数初始化LED2的GPIO

  GPIO_WriteBit(GPIOA, GPIO_Pin_3, Bit_RESET);          //将LED2默认状态设置为熄灭

}
