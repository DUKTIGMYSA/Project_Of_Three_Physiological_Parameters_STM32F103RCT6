/*********************************************************************************************************
* 模块名称：ADC.c
* 摘    要：ADC模块
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
#include "ADC.h"

/*********************************************************************************************************
*                                              宏定义
*********************************************************************************************************/
#define DOUBLEERROR -1.0
/*********************************************************************************************************
*                                              枚举结构体定义
*********************************************************************************************************/

/*********************************************************************************************************
*                                              内部变量
*********************************************************************************************************/
static u16 s_arrADC1Data;   //存放ADC转换结果数据
static StructU16CirQue  s_structADCCirQue;            //ADC循环队列
//static u16              s_arrADCBuf[ADC1_BUF_SIZE];   //ADC循环队列的缓冲区

/*********************************************************************************************************
*                                              内部函数声明
*********************************************************************************************************/
static void ConfigADC1(void);     //配置ADC1
static void ConfigDMA1Ch1(void);  //配置DMA通道1
//static void ConfigTimer3(u16 arr, u16 psc); //配置TIM3

/************************************************心电模块***************************************************/
static void ConfigEZero(void);    //配置E_ZERO
static void ConfigECGLEAD(void);  //配置导联脱落检测电路引脚
/**********************************************************************************************************/

/*********************************************************************************************************
*                                              内部函数实现
*********************************************************************************************************/
/*********************************************************************************************************
* 函数名称：ConfigE—Zero
* 函数功能：配置ZERO
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2018年01月01日
* 注    意：ADC123_IN1-PA1
**********************************************************************************************************/
static void ConfigEZero(void)
{                          
  GPIO_InitTypeDef  GPIO_InitStructure; //GPIO_InitStructure用于存放GPIO的参数
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); //使能GPIOC的时钟
                                                                                                                 
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_13;           //设置引脚
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //设置I/O输出速度
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;     //设置模式
  GPIO_Init(GPIOC, &GPIO_InitStructure);                //根据参数初始化ezero的GPIO

  GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_RESET);            //ezero设置为低电平
  
}

/*********************************************************************************************************
* 函数名称：ConfigECGLEAD
* 函数功能：配置导联脱落检测引脚:ECG_LEAD_OFF
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2018年01月01日
* 注    意：ECG_LEAD_OFF,PA7
**********************************************************************************************************/
static void ConfigECGLEAD(void)
{                          
  GPIO_InitTypeDef  GPIO_InitStructure; //GPIO_InitStructure用于存放GPIO的参数
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //使能GPIOC的时钟
                                                                                                                 
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7;           //设置引脚PA7
  //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //设置I/O输出速度
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;     //设置输入模式，上拉输入，默认问高电平：导联脱落
  GPIO_Init(GPIOA, &GPIO_InitStructure);                //根据参数初始化ECG_LEAF_OFF的GPIO

  //GPIO_WriteBit(GPIOA, GPIO_Pin_7, Bit_RESET);            //ezero设置为低电平
  
}

/*********************************************************************************************************
* 函数名称：ConfigADC1
* 函数功能：配置ADC1
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2018年01月01日
* 注    意：ADC123_IN1-PA1
**********************************************************************************************************/
static void ConfigADC1(void)
{                          
  GPIO_InitTypeDef  GPIO_InitStructure; //GPIO_InitStructure用于存放GPIO的参数
  ADC_InitTypeDef   ADC_InitStructure;  //ADC_InitStructure用于存放ADC的参数

  //使能RCC相关时钟
  RCC_ADCCLKConfig(RCC_PCLK2_Div6); //设置ADC时钟分频，ADCCLK=PCLK2/6=12MHz
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1  , ENABLE);  //使能ADC1的时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);  //使能GPIOA的时钟
 
/************************************************体温模块***************************************************/
  //配置体温模块T_VAL的GPIO
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6;    //设置引脚
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN; //设置输入类型
  GPIO_Init(GPIOA, &GPIO_InitStructure);  //根据参数初始化GPIO
/**********************************************************************************************************/
  
/************************************************血氧模块***************************************************/
  //配置血氧模块S_VSIG的GPIO
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5;    //设置引脚
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN; //设置输入类型
  GPIO_Init(GPIOA, &GPIO_InitStructure);  //根据参数初始化GPIO
/**********************************************************************************************************/

/************************************************心电模块***************************************************/
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;    //设置引脚PC4
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN; //设置输入类型
  GPIO_Init(GPIOC, &GPIO_InitStructure);  //根据参数初始化GPIO
/**********************************************************************************************************/
  
  //配置ADC1
  ADC_InitStructure.ADC_Mode               = ADC_Mode_Independent;  //设置为独立模式
  ADC_InitStructure.ADC_ScanConvMode       = ENABLE;                //使能扫描模式
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;               //禁止连续转换模式
  ADC_InitStructure.ADC_ExternalTrigConv   = ADC_ExternalTrigConv_T3_TRGO;  //使用TIM3触发
  ADC_InitStructure.ADC_DataAlign          = ADC_DataAlign_Right;   //设置为右对齐
  ADC_InitStructure.ADC_NbrOfChannel       = 1; //设置ADC的通道数目
  ADC_Init(ADC1, &ADC_InitStructure);

  ADC_DMACmd(ADC1, ENABLE);                   //使能ADC1的DMA
  ADC_ExternalTrigConvCmd(ADC1, ENABLE);      //使用外部事件启动ADC转换
  ADC_Cmd(ADC1, ENABLE);                      //使能ADC1
  
  ADC_ResetCalibration(ADC1);                 //启动ADC复位校准，即将RSTCAL赋值为1
  while(ADC_GetResetCalibrationStatus(ADC1)); //读取并判断RSTCAL，RSTCAL为0跳出while语句
  ADC_StartCalibration(ADC1);                 //启动ADC校准，即将CAL赋值为1
  while(ADC_GetCalibrationStatus(ADC1));      //读取并判断CAL，CAL为0跳出while语句
}

/*********************************************************************************************************
* 函数名称：ConfigDMA1Ch1
* 函数功能：配置DMA通道1
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2018年01月01日
* 注    意：
**********************************************************************************************************/
static void ConfigDMA1Ch1(void)
{
  DMA_InitTypeDef DMA_InitStructure;  //DMA_InitStructure用于存放DMA的参数
  
  //使能RCC相关时钟
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);  //使能DMA1的时钟
  
  //配置DMA1_Channel1
  DMA_DeInit(DMA1_Channel1);  //将DMA1_CH1寄存器设置为默认值
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(ADC1->DR);           //设置外设地址
  DMA_InitStructure.DMA_MemoryBaseAddr     = (uint32_t)&s_arrADC1Data;        //设置存储器地址
  DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralSRC;           //设置为外设到存储器模式
  DMA_InitStructure.DMA_BufferSize         = 1;                               //设置要传输的数据项数目
  DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;       //设置外设为非递增模式
  DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;            //设置存储器为递增模式
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //设置外设数据长度为半字
  DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;     //设置存储器数据长度为半字
  DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;               //设置为循环模式
  DMA_InitStructure.DMA_Priority           = DMA_Priority_Medium;             //设置为中等优先级
  DMA_InitStructure.DMA_M2M                = DMA_M2M_Disable;                 //禁止存储器到存储器访问
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);  //根据参数初始化DMA1_Channel1
  
  DMA_Cmd(DMA1_Channel1, ENABLE); //使能DMA1_Channel1
}

/*********************************************************************************************************
* 函数名称：ConfigTimer3
* 函数功能：配置定时器TIM3的参数
* 输入参数：arr-自动重装值，psc-时钟预分频数
* 输出参数：void
* 返 回 值：void
* 创建日期：2018年01月01日
* 注    意： 
**********************************************************************************************************/
void ConfigTimer3(u16 arr, u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure; //TIM_TimeBaseStructure用于存放TIM3的参数
//  NVIC_InitTypeDef NVIC_InitStructure;            //NVIC_InitStructure用于存放NVIC的参数
    
  //使能RCC相关时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);  //使能TIM3的时钟

  //配置TIM3
  TIM_TimeBaseStructure.TIM_Period        = arr;  //设置自动重装载值
  TIM_TimeBaseStructure.TIM_Prescaler     = psc;  //设置预分频器值
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;       //设置时钟分割：tDTS = tCK_INT
  TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up; //设置向上计数模式
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);               //根据参数初始化定时器  
  
  TIM_SelectOutputTrigger(TIM3,TIM_TRGOSource_Update);          //选择更新事件为触发输入
  
  TIM_ITConfig(TIM3, TIM_IT_Update,ENABLE);                     //使能定时器的更新中断
  
//  //配置NVIC
//  NVIC_InitStructure.NVIC_IRQChannel      = TIM3_IRQn;          //中断通道号
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     //设置抢占优先级
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;     //设置子优先级
//  NVIC_InitStructure.NVIC_IRQChannelCmd   = ENABLE;             //使能中断
//  NVIC_Init(&NVIC_InitStructure);                               //根据参数初始化NVIC
  
  TIM_Cmd(TIM3, ENABLE);  //使能定时器
}
 
/*********************************************************************************************************
* 函数名称：TIM3_IRQHandler
* 函数功能：TIM3中断服务函数
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2018年01月01日
* 注    意：
**********************************************************************************************************/
void TIM3_IRQHandler(void)
{  
  if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  //判断定时器更新中断是否发生
  {
    TIM_ClearITPendingBit(TIM3, TIM_FLAG_Update);     //清除定时器更新中断标志 
  }

//  WriteADCBuf(s_arrADC1Data);   //向ADC缓冲区写入数据
}

/*********************************************************************************************************
*                                              API函数实现
*********************************************************************************************************/
/*********************************************************************************************************
* 函数名称：InitADC
* 函数功能：初始化ADC模块
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2018年01月01日
* 注    意：
**********************************************************************************************************/
void InitADC(void)
{
  ConfigTimer3(1, 719);   //100KHz，计数到800为8ms
  ConfigADC1();             //配置ADC1
  ConfigDMA1Ch1();          //配置DMA1的通道1
  ConfigEZero();            //配置E_ZERO
  ConfigECGLEAD();          //配置导联脱落检测电路引脚

//  InitU16Queue(&s_structADCCirQue, s_arrADCBuf, ADC1_BUF_SIZE); //初始化ADC缓冲区  
}

/*********************************************************************************************************
* 函数名称：WriteADCBuf
* 函数功能：向ADC缓冲区写入数据
* 输入参数：d-待写入的数据
* 输出参数：void
* 返 回 值：成功标志位，1为成功，0为不成功
* 创建日期：2018年01月01日
* 注    意：
**********************************************************************************************************/
u8 WriteADCBuf(u16 d)
{
  u8 ok = 0;  //将读取成功标志位的值设置为0

  ok = EnU16Queue(&s_structADCCirQue, &d, 1); //入队
  
  return ok;  //返回读取成功标志位的值
}


/*********************************************************************************************************
* 函数名称：ReadADCBuf
* 函数功能：从ADC缓冲区读取数据
* 输入参数：p-读取的数据存放的首地址
* 输出参数：void
* 返 回 值：成功标志位，1为成功，0为不成功
* 创建日期：2018年01月01日
* 注    意：
**********************************************************************************************************/
void ReadADCBuf(u16* p)
{
//  u8 ok = 0;  //将读取成功标志位的值设置为0

//  ok = DeU16Queue(&s_structADCCirQue, p, 1); //出队
  
  *p = s_arrADC1Data;

//  return ok;  //返回读取成功标志位的值
}

/*********************************************************************************************************
* 函数名称：GetADC1Data（体温模块）
* 函数功能：获取ADC转换电压数据
* 输入参数：p-读取的数据存放的首地址
* 输出参数：void
* 返 回 值：成功标志位，1为成功，0为不成功
* 创建日期：2018年01月01日
* 注    意：
**********************************************************************************************************/
void GetADC1Data(u16* p)
{
  *p = s_arrADC1Data;
}

/*********************************************************************************************************
* 函数名称：ReadSPO2ADCBuf
* 函数功能：从ADC缓冲区读取数据
* 输入参数：p-读取的数据存放的首地址
* 输出参数：void
* 返 回 值：成功标志位，1为成功，0为不成功
* 创建日期：2018年01月01日
* 注    意：
**********************************************************************************************************/
void ReadSPO2ADCBuf(u16* p)
{
//  u8 ok = 0;  //将读取成功标志位的值设置为0

//  ok = DeU16Queue(&s_structADCCirQue, p, 1); //出队
  
  *p = s_arrADC1Data;

//  return ok;  //返回读取成功标志位的值
}

/*********************************************************************************************************
* 函数名称：ReadECGADCBuf
* 函数功能：从ADC缓冲区读取数据
* 输入参数：p-读取的数据存放的首地址
* 输出参数：void
* 返 回 值：成功标志位，1为成功，0为不成功
* 创建日期：2018年01月01日
* 注    意：
**********************************************************************************************************/
u16 ReadECGADCBuf(u16* p)
{
//  u8 ok = 0;  //将读取成功标志位的值设置为0

//  ok = DeU16Queue(&s_structADCCirQue, p, 1); //出队
  *p = s_arrADC1Data;
  return 1;  //返回读取成功标志位的值
}
