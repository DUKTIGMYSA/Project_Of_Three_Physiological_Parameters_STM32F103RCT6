/*********************************************************************************************************
* 模块名称：Temp.c
* 摘    要：体温参数板模块，包括模块初始化，以及根据传输电压计算电阻与温度值
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
#include "Temp.h"
#include "Main.h"

/*********************************************************************************************************
*                                              宏定义
*********************************************************************************************************/

/*********************************************************************************************************
*                                              枚举结构体定义
*********************************************************************************************************/            

/*********************************************************************************************************
*                                              内部变量
*********************************************************************************************************/    
extern u8 s_arrTempData[6];          // 表示的是PCT协议中体温数据包的数组
extern double arrTempVariate[10];    // 表示的是储存温度值的数组 
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
* 函数名称：InitTemp
* 函数功能：初始化温度参数板模块
* 输入参数：void
* 输出参数：void
* 返 回 值：void 
* 创建日期：2018年01月01日
* 注    意：
*********************************************************************************************************/
void  InitTemp(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;   //GPIO_InitStructure用于存放GPIO的参数
  
  //使能RCC相关时钟                                                            
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);     //使能GPIOB的时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);     //使能GPIOC的时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);      //使能AFIO的时钟
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);  //使能SWD禁用JTAG
  
  //配置T_PA的GPIO 
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;               //设置T_PA的引脚
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;          //设置T_PA的模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;         //设置T_PA的I/O口输出速度
  GPIO_Init(GPIOB, &GPIO_InitStructure);                    //根据参数初始化T_PA的GPIO
  
  //配置T_PB的GPIO
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;               //设置T_PB的引脚
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;          //设置T_PB的模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;         //设置T_PB的I/O口输出速度
  GPIO_Init(GPIOB, &GPIO_InitStructure);                    //根据参数初始化T_PB的GPIO
  
  //配置T_SENS1的GPIO
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8;               //设置T_SENS1的引脚
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;          //设置T_SENS1的模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;         //设置T_SENS1的I/O口输出速度
  GPIO_Init(GPIOC, &GPIO_InitStructure);                    //根据参数初始化T_SENS1的GPIO
  
  //配置T_SENS2的GPIO
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;               //设置T_SENS2的引脚
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;          //设置T_SENS2的模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;         //设置T_SENS2的I/O口输出速度
  GPIO_Init(GPIOC, &GPIO_InitStructure);                    //根据参数初始化T_SENS2的GPIO
  
  //配置T_SENS_OFF的GPIO
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5;               //设置T_SENS_OFF的引脚
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;            //设置T_SENS_OFF的模式
  GPIO_Init(GPIOB, &GPIO_InitStructure);                    //根据参数初始化T_SENS_OFF的GPIO
  
  //初始配置为低电平
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); 
  GPIO_ResetBits(GPIOB, GPIO_Pin_3);    // PB3 is set to 0;
  GPIO_ResetBits(GPIOB, GPIO_Pin_4);    // PB4 is set to 0;
  GPIO_ResetBits(GPIOC, GPIO_Pin_8);    // PC8 is set to 0;
  GPIO_ResetBits(GPIOC, GPIO_Pin_9);    // PC9 is set to 0;

}

/*********************************************************************************************************
* 函数名称：TempWriteBit
* 函数功能：打开任一体温测量通道
* 输入参数：void
* 输出参数：void
* 返 回 值：void 
* 创建日期：2018年01月01日
* 注    意：
*********************************************************************************************************/
void TempWriteBit(EnumKEYModel KEYModel)
{
  if(KEYModel == T_PA)                             
  {
    //开启T_PA通道
    GPIO_WriteBit(GPIOB,GPIO_Pin_3, Bit_SET);
    GPIO_WriteBit(GPIOB,GPIO_Pin_4, Bit_RESET);
    GPIO_WriteBit(GPIOC,GPIO_Pin_8, Bit_RESET);
    GPIO_WriteBit(GPIOC,GPIO_Pin_9, Bit_RESET);
  }
  else if(KEYModel == T_PB)                       
  {
    //开启T_PB通道
    GPIO_WriteBit(GPIOB,GPIO_Pin_3, Bit_RESET);
    GPIO_WriteBit(GPIOB,GPIO_Pin_4, Bit_SET);
    GPIO_WriteBit(GPIOC,GPIO_Pin_8, Bit_RESET);
    GPIO_WriteBit(GPIOC,GPIO_Pin_9, Bit_RESET);
  }
  else if(KEYModel == T_SENS1)
  {
    //开启T_SENS1通道
    GPIO_WriteBit(GPIOB,GPIO_Pin_3, Bit_RESET);
    GPIO_WriteBit(GPIOB,GPIO_Pin_4, Bit_RESET);
    GPIO_WriteBit(GPIOC,GPIO_Pin_8, Bit_SET);
    GPIO_WriteBit(GPIOC,GPIO_Pin_9, Bit_RESET);
  }
  else if(KEYModel == T_SENS2)
  {
    //开启T_SENS2通道
    GPIO_WriteBit(GPIOB,GPIO_Pin_3, Bit_RESET);
    GPIO_WriteBit(GPIOB,GPIO_Pin_4, Bit_RESET);
    GPIO_WriteBit(GPIOC,GPIO_Pin_8, Bit_RESET);
    GPIO_WriteBit(GPIOC,GPIO_Pin_9, Bit_SET);
  }
}

/*********************************************************************************************************
* 函数名称：ReckonToC1AndC2
* 函数功能：计算体温模块中C1与C2系数
* 输入参数：void
* 输出参数：void
* 返 回 值：void 
* 创建日期：2018年01月01日
* 注    意：
*********************************************************************************************************/
void ReckonToC1AndC2(double PA_Val,double PB_Val,double*C1,double*C2)
{
  const double R1 = 7350.0;  
  const double R2 = 510.0;
  
  *C1 = (R1 * R2 * (PB_Val - PA_Val)) / (R2 * PA_Val - R1 * PB_Val);
  *C2 = ((R1 - R2) * PA_Val * PB_Val) / (R1 * PB_Val - R2 * PA_Val);
}

/*********************************************************************************************************
* 函数名称：RextReckon
* 函数功能：计算体温探头阻值函数
* 输入参数：void
* 输出参数：void
* 返 回 值：void 
* 创建日期：2018年01月01日
* 注    意：
*********************************************************************************************************/
double RextReckon(double SENS_Val,double C1,double C2)
{
  double Rextnum0;      // 体温探头的电阻值
  Rextnum0 = (C1 * SENS_Val) / (C2 - SENS_Val);
  return Rextnum0;
}

/*********************************************************************************************************
* 函数名称：ChangeTempDataPacket
* 函数功能：体温数据滤波与初始化体温打包数据包中的体温数据
* 输入参数：void
* 输出参数：void
* 返 回 值：void 
* 创建日期：2018年01月01日
* 注    意：
*********************************************************************************************************/
void ChangeTempDataPacket(void)
{
  double DoubleTempData = 0.0;
  i16 TempData = 0.0;                 // 放大10倍发送到上位机的体温数据
  i8 i;                               // PCT协议中体温数据包的数组下标
  double TempMin = 0;                 // 采集到的体温数组里的体温最小值
  double TempMax = 0;                 // 采集到的体温数组里的体温最大值
  u8 TempMinArrNum,TempMaxArrNum;     // 采集到的体温最小值与最大值对应的数组下标
  u8 arrNum = GetArrNum();                // 获取储存温度值的数组的元素个数作为筛选体温数据for循环的次数
  u8 arrNum0 = arrNum;                // 储存温度值的数组的元素个数备份，为后续元素个数的改变做准备
  if((BitAction)(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5)) == 1)  // 出现探头脱落的情况
  {
    s_arrTempData[0] = 3;    // 体温探头状态
    for(i = 1;i < 5;i++)       // 体温通道1与2数据清空
    {
      s_arrTempData[i] = 0;
    }
  }
  else                                         // 体温探头连接
  {
      for(i = 0;i <= arrNum -1;i++)            // 挑选存放体温数据数组当中体温的最大值与最小值
      {
        if(arrTempVariate[i] <= 0.0)           // 将无效体温数据剔除
        {
          arrNum0--;                           // 储存温度值的数组的元素个数删除该数据     
          continue;
        }
        else                                    // 挑选存放体温数据数组当中体温的最大值与最小值
        {
          DoubleTempData += arrTempVariate[i];  
          if(i == 0)                            // 体温最值以及相关数组下标进行初始化
          {
            TempMin = arrTempVariate[i];
            TempMax = arrTempVariate[i];
            TempMaxArrNum = i;
            TempMinArrNum = i;
          }
          else                                  // 找出体温最小值与最大值以及相关数组下标
          {
            if(arrTempVariate[i] > TempMax)
            {
              TempMax = arrTempVariate[i];
              TempMaxArrNum = i;
            }
            if(arrTempVariate[i] < TempMin)
            {
            TempMin = arrTempVariate[i];
            TempMinArrNum = i;
            }
          }
        }
      }
      
      if((TempMax - TempMin) > 0.5)              // 当体温数组的最大值与最小值超过0.5摄氏度时则丢掉这两个值
      {
        DoubleTempData -= (TempMax + TempMin);
        arrNum0 -= 2;
        arrTempVariate[TempMaxArrNum] = 0;
        arrTempVariate[TempMinArrNum] = 0;
      }
  
      
      if(arrNum != 0)                            // 将处理好的体温数据乘以10即为发送到上位机的体温数据
      {
        TempData = (i16)(DoubleTempData / arrNum0  * 10);
      }
      
      if((BitAction)(GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_8)) == 1)  // 通道T_SENS1正常工作的数据包更新
      {      
        s_arrTempData[0] = 2;                          // 体温探头状态
        s_arrTempData[1] = (TempData >> 8) & 0xFF;     // 通道1高字节数据
        s_arrTempData[2] = (TempData) & 0xFF;          // 通道1低字节数据
        s_arrTempData[3] = 0;                          // 通道2高字节数据
        s_arrTempData[4] = 0;                          // 通道2低字节数据
      }
      else if((BitAction)(GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_9)) == 1)  //通道T_SENS2正常工作的数据包更新
      {
        s_arrTempData[0] = 1;                          // 体温探头状态
        s_arrTempData[1] = 0;                          // 通道1高字节数据
        s_arrTempData[2] = 0;                          // 通道1低字节数据
        s_arrTempData[3] = (TempData >> 8) & 0xFF;     // 通道2高字节数据
        s_arrTempData[4] = (TempData) & 0xFF;          // 通道2低字节数据
      }
      else                                             // 其他情况（严谨）
      {
      
      }
  }
    
}
