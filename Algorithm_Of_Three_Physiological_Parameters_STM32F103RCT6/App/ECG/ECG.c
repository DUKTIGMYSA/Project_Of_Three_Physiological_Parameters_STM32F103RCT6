/*********************************************************************************************************
* 模块名称：ECG.c
* 摘    要：ECG模块
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
#include "ECG.h"
#include "stm32f10x_conf.h"
#include "Timer.h"
#include "ADC.h"
#include "Filter.h"
#include "SendDataToHost.h"
//#include "LED.h"

/*********************************************************************************************************
*                                              宏定义
*********************************************************************************************************/

/*********************************************************************************************************
*                                              枚举结构体定义
*********************************************************************************************************/


/*********************************************************************************************************
*                                              内部变量
*********************************************************************************************************/
//u16 IBI;  //两次脉搏间隔
static u16 BPM;  //心率
static u8 s_iLeadOff;  //导联脱落变量：1：脱落；0：连接

/*********************************************************************************************************
*                                              内部函数实现
*********************************************************************************************************/
/*********************************************************************************************************
* 函数名称：GetArrMax
* 函数功能：计算最大值
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2018年01月01日
* 注    意：
*********************************************************************************************************/
static u16 GetArrMax(u16 arr[],int len)
{
  int i;
  int max = arr[0];
  for( i = 1;i < len - 1;i++)
  {
    if(max < arr[i])
    {       
      max = arr[i];
    }
  }
  return max;
}

/*********************************************************************************************************
* 函数名称：GetArrMin
* 函数功能：计算最小值
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2018年01月01日
* 注    意：
*********************************************************************************************************/
static u16 GetArrMin(u16 arr[],int len)
{
  int i;
  int min = arr[0];
  for( i = 1;i < len - 1;i++)
  {
    if(min > arr[i])
    {       
      min = arr[i];
    }
  }
  return min;
}

/*********************************************************************************************************
* 函数名称：CoutThreshold
* 函数功能：计算阈值
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2018年01月01日
* 注    意：
*********************************************************************************************************/
static float CoutThreshold(u16 Max,u16 Min)
{
  return (Max+Min)*0.52;
}




/*********************************************************************************************************
*                                              API函数实现
*********************************************************************************************************/
/*********************************************************************************************************
* 函数名称：InitWave
* 函数功能：初始化Wave模块 
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2018年01月01日
* 注    意：
*********************************************************************************************************/
/*********************************************************************************************************
* 函数名称：InitConfigTemp
* 函数功能：配置体温通道相关的GPIO、RCC
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2018年01月01日
* 注    意：
*********************************************************************************************************/
void  SendECGWaveAndHr(void)
{
  static u8 s_iCnt4 = 0;   //8ms计数器  
  static u16 adcData;       //原始心电波形adc数据 
  static int firstEffectivePulse  = 0;     //第一次有效脉搏，下标
  int secondEffectivePulse  = 0;     //第二次有效脉搏，下标
  static u16 maxAdcData;    //一个周期adc采样的最大值
  static u16 minAdcData;    //一个周期adc采样的最小值
  static u16 preThreshold;  //一个周期采样后计算的阈值
  //static u16 s_arrECGdata[2000]={0};//存放一个周期的原始心电数据ECG_ADC,用于滤波
  static u16 s_arrFilteredECGData[2000] = {0}; //长度2000的数组，存放滤波后的adc，用于计算心率
  static u16 Index = 0;        //数组下标
  static u16 DATA_SIZE = 2000; //数组长度
  static u8 s_bPreDataStatus = FALSE;  //前一个点的状态，默认FALSE
  static u8 s_bCurrDataStatus = FALSE;  //当前该点的状态，默认FALSE
  static int s_iPulseCnt;  //有效脉搏计数器
  static u16 refractoryPeriod = 93; // bpm=320的不应期的长度（以数据点为单位）
  
  static u8 s_arrECGWaveData[6] = {0}; //初始化心电波形数组,用于存放待打包的心电波形数据
  u8 waveDataHigh8;  //波形数据高字节
  u8 waveDataLow8;  //波形数据低字节
  double LowpassFIRVAL;  //低通滤波后的变量
  static float s_arrInputSignals_x[3] = {0};  // 用于存储过去输入信号的数组
  static float s_arrInputSignals_y[3] = {0};  // 用于存储过去输出信号的数组
  float adcAfterLowpass;  //低通后的数据，作为高通滤波的输入
  u16 filteredAdc;  //低通＋高通滤波结束后的心电数据
  
  s_iLeadOff = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7);//读取导联引脚电平状态
  s_iCnt4++;  //计数增加,用于发送心电数据包
  if(s_iLeadOff==0) //导联连接
  {
    if(ReadECGADCBuf(&adcData))  //2ms取出1个数据
    {      
      LowpassFIRVAL = LowpassFilter_FIR(adcData);  //低通FIR，滤除共频干扰
      adcAfterLowpass = (float)LowpassFIRVAL;
      highpassFilter_IIR(&adcAfterLowpass,s_arrInputSignals_x,s_arrInputSignals_y);//IIR高通,滤除基线漂移
      adcAfterLowpass += 2048.0;  //基线2048
      filteredAdc = (u16)adcAfterLowpass;
            
      
      //每4s计算一次心率
      //长度为2000的数组存放adc值，每2ms存入一个adc值，存满一次为一个周期采样
      s_arrFilteredECGData[Index] = filteredAdc;  
      Index++;  //每2ms计数+1，可通过下标差计算心率

      //计算阈值：存满数组就计算一次新的阈值，用于下一个数组的数据进行阈值判断，并判断当前这个adc是否满足阈值
      if(Index >= DATA_SIZE)  
      {
        Index = 0;
        maxAdcData = GetArrMax(s_arrFilteredECGData,DATA_SIZE);  //返回最大值
        minAdcData = GetArrMin(s_arrFilteredECGData,DATA_SIZE);  //返回最小值
        preThreshold = CoutThreshold(maxAdcData,minAdcData);  //计算当前一个周期1500个采样值的阈值 
      }

      s_bPreDataStatus = s_bCurrDataStatus;  //保存当前脉冲状态，初始为FALSE       
      if(filteredAdc > preThreshold)// 采样值大于阈值，脉冲状态标记为TRUE
      {
        s_bCurrDataStatus = TRUE;  //TRUE
      }
      else
      {
        s_bCurrDataStatus = FALSE;  //否则为FALSE
      }
      
      //当某个点的前一个点小于等于阈值且该点大于阈值，判断为有效脉冲
      if(s_bPreDataStatus == FALSE && s_bCurrDataStatus == TRUE) 
      {
        if (((Index - firstEffectivePulse ) > refractoryPeriod)||((firstEffectivePulse >Index)&&(2000-firstEffectivePulse +Index)>refractoryPeriod))  // 检查是否在不应期之后
        {        
          s_iPulseCnt++;    //有效脉冲计数+1  
        
          if((s_iPulseCnt % 2) == 1)  //第一次脉搏
          {
            firstEffectivePulse  = Index;  // 记录第一次有效脉搏时间 
          }
          if((s_iPulseCnt % 2) == 0)  // 第二次脉搏,计算心率
          {
            secondEffectivePulse  = Index;  // 记录第二次有效脉搏时间

            if(secondEffectivePulse  < firstEffectivePulse )
            {
              //IBI = (secondEffectivePulse  - firstEffectivePulse ) * 2; //两次脉搏间隔点数
              //BPM = 60000/IBI;
              BPM = 30000/(secondEffectivePulse  + 2000 - firstEffectivePulse );  //计算BPM
            }
            else
            {
              BPM = 30000/(secondEffectivePulse  - firstEffectivePulse );  //计算BPM
            }         
          }
          //lastPulseTime = Index; // 更新上一个脉冲的下标       
        }       
      }      
    }
    //LEDFlicker(250);//调用闪烁函数   
  }
  if(s_iLeadOff == 1)  //高电平：导联脱落 //导联脱落
  {
    filteredAdc = 2048;  //输出基线
  }
  if(s_iCnt4 >= 4 )  //达到8ms，发送一次心电波形数据包
  {      
    //12位adc值需用两个8位存放，即两个字节
    waveDataHigh8 = (filteredAdc >> 8) & 0xFF;  //右移8位，取高八位，实际是adc值的高4位
    waveDataLow8 = filteredAdc & 0xFF;  //取低8位
    
    //打包波形数据发送到上位机
    s_arrECGWaveData[0] = waveDataHigh8;  //高字节存放到数组第0位，对应数据包的DAT1
    s_arrECGWaveData[1] = waveDataLow8;  //低字节存放到数组第1位，对应数据包的DAT2
    SendWaveToHost(s_arrECGWaveData);  //发送心电波形数据包
    s_iCnt4 = 0;  //准备下次的循环
  }
  //Clr2msFlag();   //清除2ms标志

}

/*********************************************************************************************************
* 函数名称：Proc1SecTask
* 函数功能：1s处理任务，发送心率数据包和心电导联信息数据包
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2018年01月01日
* 注    意：
*********************************************************************************************************/
void  SendECGLeadData(void)
{ 
  u8 bpmHighBit8;  //高字节数据
  u8 bpmLowBit8;  //低字节数据
  static u8 s_arrEcgHrData[6] = {0};       //初始化心率数组,用于存放待打包的心率数据 
  static u8 s_arrEcgLeadOffData[6] = {0};  //初始化心点导联信息数组,用于存放待打包的心电导联信息数据 
//  static u8 s_iLeadOff;  //导联脱落变量：1或0
//  s_iLeadOff = GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7);  //读取ECG_LEAD_OFF引脚电平

  if(Get1SecFlag()) //判断1s标志状态
  {
    s_iLeadOff = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7);//读取导联电平
    
    if(s_iLeadOff == 0 )  //引脚低电平：导联连接
    {    
      s_arrEcgLeadOffData[0] = s_iLeadOff;
      
      if(( 28> BPM ) || ( BPM>253))  //判断BPM是否在有效范围30-240，误差±5％
      { 
        BPM = 400;        
      }
     }

      //if(s_iLeadOff == 1)  //高电平：导联脱落
    else  //引脚高电平：导联脱落 
    {
      s_iLeadOff = s_iLeadOff |0x0E;  //位1~3置1,因为电路无法判断具体是哪个导联脱落
      s_arrEcgLeadOffData[0] = s_iLeadOff;  //存放到数组第0位，对应数据包的DAT1
      
      BPM = 400; //无效心率值     
    } 
    SendEcgLeadToHost(s_arrEcgLeadOffData);  //发送心电导联信息数据包
    //打包心率数据并发送
    bpmHighBit8 = (BPM >> 8) & 0xFF;  //右移8位，取高八位，实际是adc值的高4位
    bpmLowBit8 = BPM & 0xFF;  //取低8位
    s_arrEcgHrData[0] = bpmHighBit8;  //心率高字节存放到数组第0位，对应数据包的DAT1
    s_arrEcgHrData[1] = bpmLowBit8;  //心率低字节存放到数组第1位，对应数据包的DAT2      
    SendEcgHrToHost(s_arrEcgHrData);  //发送心率数据包  
    
    //Clr1SecFlag();  //清除1s标志
  }    
}
