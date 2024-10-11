/*********************************************************************************************************
* 模块名称：Main.c
* 摘    要：主文件，包含软硬件初始化函数和main函数
* 当前版本：1.0.0
* 作    者：SZLY(COPYRIGHT 2018 - 2020 SZLY. All rights reserved.)
* 完成日期：2020年01月01日
* 内    容：
* 注    意：注意勾选Options for Target 'Target1'->Code Generation->Use MicroLIB，否则printf无法使用                                                                  
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
#include "Main.h"
#include "stm32f10x_conf.h"
#include "NVIC.h"
#include "SysTick.h"
#include "RCC.h"
#include "Timer.h"
#include "UART1.h"
#include "LED.h"
#include "Wave.h"
#include "ADC.h"
#include "KeyOne.h"
#include "ProcKeyOne.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
/*********************************************通信下载模块***************************************************/
#include "ProcHostCmd.h"
#include "PackUnpack.h"
#include "SendDataToHost.h"
/***********************************************************************************************************/

/************************************************体温模块***************************************************/
#include "Temp.h"
#include "TempRext.h"
/***********************************************************************************************************/

/************************************************血氧模块***************************************************/
#include "DAC.h"
#include "VDC.h"
#include "Bright.h"
#include "Pluse.h"
#include "SpoVal.h"
/**********************************************************************************************************/

/************************************************心电模块***************************************************/
#include "Filter.h"
/**********************************************************************************************************/

/*********************************************************************************************************
*                                              宏定义
*********************************************************************************************************/

/************************************************血氧模块***************************************************/
#define AVGLVBO_POINTNUM 8
#define MEANLVBO_POINTNUM 10
/**********************************************************************************************************/

/*********************************************************************************************************
*                                              内部变量
*********************************************************************************************************/

/************************************************体温模块***************************************************/
static u8 reckonBegin;      // 表示的是是否成功采集到S1/S2通道的电压数据，目的是是否计算后续对应探头的阻值与温度值

static i8 keyBegin;         
// 表示的是在1s内通道自动切换的轮数（一轮表示从PA通道开始按照keyNumber规定的通道自动切换回到PA通道，在1s内需要采集x个体温值说明在1s内通道自动切换的轮数为x轮）
// 规定1s内采集的体温值个数为5次，因此1s内通道自动切换的轮数为5轮，keyBegin为0时表示未采集到体温数据

static u8 RestoreJudge;     
// 表示的是当体温探头导联脱落时值为TRUE，为体温探头导联重新连接时从头开始采集数据做准备，此刻keyNumber与keyBegin的值都为0，表示通道停止自动切换且未采集到体温数据
// 当体温探头导联重新连接时值为FALSE，表示已完成通道切换状态与体温采集轮数的初始化，此刻keyNumber的值为1表示通道在PA通道，keyBegin的值为0表示未采集到体温数据

static u8 ReadTOffValue;    // 表示的是T_off位置采集的电平情况，1为高电平，0为低电平
static double PA_Val;       // 表示的是PA通道采集的电压值
static double PB_Val;       // 表示的是PB通道采集的电压值

// 表示的是通过PA与PB各自通道采集到的电压值计算得到的C1和C2系数
static double C1;
static double C2;

static double RextVariate;  // 表示的是通过PA、PB与S1或S2通道采集到的电压值计算得到的热敏探头的阻值
static u8 arrNum;           // 表示的是储存温度值的数组的元素个数
static u8 stateTempOff = 2;
/**********************************************************************************************************/

/************************************************血氧模块***************************************************/
static u16 RED_penData[300] = {0};    //3s一组存放红光数据
static u16 IR_penData[300] = {0};     //3s一组存放红外数据
static u16 s_iRED = 0;                    //临时存放红光数据
static u16 s_iIR = 0;                     //临时存放红外数据
static u16 REDlight = 1600;                  //存放红光光强数据
static u16 IRlight = 1400;                   //存放红外光强数据
static i16 Spo2 = 0;                         //血氧值
static int RED300Data[300] = {0};             //有300个红光数组的数据
static int REDMaxPeaks[60] = {0};             //红光数据的波峰的位置
static int REDMinPeaks[60] = {0};             //红光数据的波谷的位置
static int REDMax_len = 60;                   //红光波峰数组的长度
static int REDMin_len = 60;                   //红光波谷数组的长度
static float s_arrREDLightBufAVG[AVGLVBO_POINTNUM] = {0};        //红光数据均值滤波窗口
static float s_arrREDLightBufMEAN[MEANLVBO_POINTNUM] = {0};       //红光数据中值滤波窗口
static float RED_LvBo_avg = 0;    //滤波后数据的平均值
static float RED_LvBo_res = 0;  //滤波后结果
static float RED_LvBo_sum = 0;    //滤波窗口的数据和
static u8  ProSign = 0x00;      //脱落标志
static u8 Rate = 0;          //脉率
static float RED_LvBo_resNEW = 0; //IIR滤波器的结果
static float RED_LvBo_resPLUSE = 0;//均值滤波结果
/**********************************************************************************************************/

/*********************************************************************************************************
*                                              全局变量
*********************************************************************************************************/

/************************************************体温模块***************************************************/
u8      keyNumber;               // 表示的是通道自动切换的顺序（PA->PB->S1/S2通道->PA，当值为0时停止切换通道）
u8      keyChange;               // 表示的是是否通过按下按钮选择通道
u8      s_i20msBegin;            // 表示的是60ms任务当中通道自动切换功能已执行完毕，开启20ms数据采集的计时
double  arrTempVariate[10];      // 表示的是储存温度值的数组
u8      s_arrTempData[6] = {0};  // 表示的是PCT协议中体温数据包的数组并进行初始化
/**********************************************************************************************************/

/************************************************心电模块***************************************************/
int time1 = 0;     //第一次脉搏变量
int time2 = 0;     //第二次脉搏变量
u16 MaxAdvData;    //一个周期adc采样的最大值
u16 MinAdvData;    //一个周期adc采样的最小值
u16 PreThreshold;  //一个周期采样后计算的阈值
u16 adcData;       //心电波形adc数据 
u16 data[2000] = {0}; //长度1500的数组，存放滤波后的adc，用于计算心率
u16 ECGdata[2000]={0};//存放原始ECG_ADC,用于滤波
u16 Index = 0;        //数组下标
u16 DATA_SIZE = 2000; //数组长度
u8  PreDataStatus = FALSE;  //前一个点的状态，默认FALSE
u8 CurrDataStatus = FALSE;  //当前该点的状态，默认FALSE
int pulseCount;  //有效脉冲计数器
//u16 IBI;
u16 BPM;  //心率

u16 refractoryPeriod = 120; // 不应期的长度（以数据点为单位）
u16 lastPulseTime = 0;       // 记录上一个脉冲的下标
/**********************************************************************************************************/

/*********************************************************************************************************
*                                              枚举结构体定义
*********************************************************************************************************/

/*********************************************************************************************************
*                                              内部函数声明
*********************************************************************************************************/
static  void  InitSoftware(void);              // 初始化软件相关的模块
static  void  InitHardware(void);              // 初始化硬件相关的模块
static  void  Proc2msTask(void);               // 2ms处理任务

/************************************************体温模块***************************************************/
static  void  Proc60msTask(void);              // 60ms处理任务
static  void  Proc60msAfter20msTask(void);     // 60ms后紧跟着的20ms处理任务
static  void  Proc1SecTask(void);              // 1s处理任务
/**********************************************************************************************************/

/************************************************血氧模块***************************************************/
static  void  Proc500usTask(void);   //500us处理任务
static  void  Proc100msTask(void);   //100ms处理任务
/**********************************************************************************************************/

/************************************************心电模块***************************************************/
static u16 GetArrMax(u16 arr[],int len);   // 计算最大值
static u16 GetArrMin(u16 arr[],int len);   // 计算最小值
static float CoutThreshold(u16 Max,u16 Min);  // 计算阈值

/**********************************************************************************************************/
/*********************************************************************************************************
*                                              内部函数实现
*********************************************************************************************************/
/*********************************************************************************************************
* 函数名称：InitSoftware
* 函数功能：所有的软件相关的模块初始化函数都放在此函数中
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2018年01月01日
* 注    意：
*********************************************************************************************************/
static  void  InitSoftware(void)
{
  InitPackUnpack();         //初始化PackUnpack模块
  InitProcHostCmd();        //初始化ProcHostCmd模块
  InitSendDataToHost();     //初始化SendDataToHost模块
  InitVariable();           //初始化参数值
}

/*********************************************************************************************************
* 函数名称：InitHardware
* 函数功能：所有的硬件相关的模块初始化函数都放在此函数中
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2018年01月01日
* 注    意：
*********************************************************************************************************/
static  void  InitHardware(void)
{  
  SystemInit();       //系统初始化
  InitRCC();          //初始化RCC模块
  InitNVIC();         //初始化NVIC模块
  InitUART1(115200);  //初始化UART模块
  InitTimer();        //初始化Timer模块
  InitLED();          //初始化LED模块
  InitSysTick();      //初始化SysTick模块
  InitKeyOne();       //初始化KeyOne模块
  InitProcKeyOne();   //初始化ProcKeyOne模块
  InitDAC();          //初始化DAC模块
  InitADC();          //初始化ADC模块
  InitTemp();         //初始化Temp模块
  InitSpoVal();       //初始化SpoVal模块
  InitFilter();       //初始化滤波模块
}


/*********************************************************************************************************
* 函数名称：Proc2msTask
* 函数功能：2ms处理任务 
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2018年01月01日
* 注    意：
*********************************************************************************************************/
static  void  Proc2msTask(void)    // 按键扫描与识别（每10ms进行一次按键识别）
{ 
  static u8 s_iCnt = 0;
/************************************************血氧模块***************************************************/
  static u16 s_iCnt100 = 0;    //100次计数器
  u8 temp = 0;
  float LvBo_temp = 0;
  static u8 point_num = 0;     //当前数据点的位置
  static u16 s_iCnt300 = 0;       //300次计数器
  u8 i = 0,k = 0;                       
  static float s_iCnt5 = 0;          //5次计数器
  static u8 RedWaveDataToHost[4] = {0};//传输数组到主机
  static float t_arrREDlightBuff[MEANLVBO_POINTNUM] = {0.0};  //用于红光中值滤波窗口
/**********************************************************************************************************/
  
/************************************************心电模块***************************************************/
  static u8 s_iCnt4 = 0;   //计数器
  static u8 s_arrWaveData[6] = {0}; //初始化心电波形数组,用于存放待打包的心电波形数据
  static u8 High8;  //高字节数据
  static u8 Low8;  //低字节数据
  double d_LowpassFIR_value;
  static float px[3] = {0};
  static float py[3] = {0};
  float adc1;
  u16 adc2;
/**********************************************************************************************************/
  if(Get2msFlag())  //判断10ms标志状态
  {      
    if(s_iCnt >= 5)
    { 
      ScanKeyOne(KEY_NAME_KEY1, ProcKeyUpKey1, ProcKeyDownKey1);
      ScanKeyOne(KEY_NAME_KEY2, ProcKeyUpKey2, ProcKeyDownKey2);
      ScanKeyOne(KEY_NAME_KEY3, ProcKeyUpKey3, ProcKeyDownKey3);

      s_iCnt = 0;
    }
    else
    {
      s_iCnt++;
    }
    
    if(GetStateNum() == 2)  // 切换为心电模块
    {
      s_iCnt4++;  //计数增加,用于发送心电数据包
      if(ReadECGADCBuf(&adcData))  //2ms取出1个数据
      {      
        d_LowpassFIR_value = LowpassFilter_FIR(adcData);  //低通FIR，滤除共频干扰
        adc1 = (float)d_LowpassFIR_value;
        highpassFilter_IIR(&adc1,px,py);//IIR高通
        adc1 += 2200.0;
        adc2 = (u16)adc1;
        
        if(s_iCnt4 >= 4 )  //达到8ms，发送一次心电波形数据包
        {      
          //12位adc值需用两个8位存放，即两个字节
          High8 = (adc2 >> 8) & 0xFF;  //右移8位，取高八位，实际是adc值的高4位
          Low8 = adc2 & 0xFF;  //取低8位
        
          //打包波形数据发送到上位机
          s_arrWaveData[0] = High8;  //高字节存放到数组第0位，对应数据包的DAT1
          s_arrWaveData[1] = Low8;  //低字节存放到数组第1位，对应数据包的DAT2
          SendEcgWaveToHost(s_arrWaveData);  //发送心电波形数据包
          s_iCnt4 = 0;  //准备下次的循环
        }      
      
        //每3s计算一次心率
        //长度为1500的数组存放adc值，每2ms存入一个adc值，存满一次为一个周期采样
        data[Index] = adc2;  
        Index++;  //每2ms计数+1，可通过下标差计算心率

        //计算阈值：存满数组就计算一次新的阈值，用于下一个数组的数据进行阈值判断，并判断当前这个adc是否满足阈值
        if(Index >= DATA_SIZE)  
        {
          Index = 0;
          MaxAdvData = GetArrMax(data,DATA_SIZE);  //返回最大值
          MinAdvData = GetArrMin(data,DATA_SIZE);  //返回最小值
          PreThreshold = CoutThreshold(MaxAdvData,MinAdvData);  //计算当前一个周期1500个采样值的阈值 
        }

        PreDataStatus = CurrDataStatus;  //保存当前脉冲状态，初始为FALSE       
        if(adc2 > PreThreshold)// 采样值大于阈值，脉冲状态标记为TRUE
        {
          CurrDataStatus = TRUE;  //TRUE
        }
        else
        {
          CurrDataStatus = FALSE;  //否则为FALSE
        }
      
        //当某个点的前一个点小于等于阈值且该点大于阈值，判断为有效脉冲
        if(PreDataStatus == FALSE && CurrDataStatus == TRUE) 
        {
          if (((Index - lastPulseTime) > refractoryPeriod)||((lastPulseTime>Index)&&(2000-lastPulseTime+Index)>refractoryPeriod))  // 检查是否在不应期之后
          {        
            pulseCount++;    //有效脉冲计数+1  
        
            if((pulseCount % 2) == 1)  //第一次脉搏
            {
              time1 = Index;  // 记录第一次脉搏时间 
            }
            if((pulseCount % 2) == 0)  // 第二次脉搏,计算心率
            {
              time2 = Index;  // 记录第二次脉搏时间

              if(time2 < time1)
              {
                //IBI = (time2 - time1) * 2;
                //BPM = 60000/IBI;
                BPM = 30000/(time2 + 2000 - time1);  //计算BPM
              }
              else
              {
                BPM = 30000/(time2 - time1);  //计算BPM
              }         
            }
            lastPulseTime = Index; // 更新上一个脉冲的下标       
          }       
        }      
      }

//      LEDFlicker(250);//调用闪烁函数   
      
    }
    
    else if(GetStateNum() == 3)  // 切换为血氧模块
    { 
      RoughAdj(&s_iRED,&s_iIR);
      REDlight=AdjustLED(1);
      IRlight=AdjustLED(0);
      s_iCnt5++;                                  //计数增加   
   
      if(s_iCnt5 >= 5)                            
      {         
      //************************************
        //数组数据右移，滑动平均滤波
        RED_LvBo_resNEW = filter_1Order(s_iRED);//
        ///////////////////////////均值滤波窗口/////////////////////////////////////////////////////////
        for(i = AVGLVBO_POINTNUM-1; i > 0; i--) 
        {
          s_arrREDLightBufAVG[i]=s_arrREDLightBufAVG[i-1];
          RED_LvBo_sum=RED_LvBo_sum+s_arrREDLightBufAVG[i-1];
        }
      
        s_arrREDLightBufAVG[0]=RED_LvBo_resNEW;                 //ADC转换的值赋给数组第一个值
        //////////////////////////均值滤波//////////////////////////////////////////////////////////
        RED_LvBo_sum=RED_LvBo_sum+RED_LvBo_resNEW;
      
        RED_LvBo_res = RED_LvBo_sum/AVGLVBO_POINTNUM;
        ///////////////////////////中值滤波窗口//////////////////////////////////////////////////////
        for(i = MEANLVBO_POINTNUM-1; i > 0; i--) 
        {
          s_arrREDLightBufMEAN[i]=s_arrREDLightBufMEAN[i-1];
        }
        s_arrREDLightBufMEAN[0] = (u16)RED_LvBo_res;
        //////////////////////////中值滤波//////////////////////////////////////////////////////////
        for(i = 0;i <= MEANLVBO_POINTNUM-1;i++)
        {
          t_arrREDlightBuff[i] = s_arrREDLightBufMEAN[i];
        }
        for(i = 0;i < MEANLVBO_POINTNUM - i;i++)
        {
          for(k=0;k<MEANLVBO_POINTNUM-i-1;k++)
          {
            if(t_arrREDlightBuff[k]>t_arrREDlightBuff[k+1])
            {
              LvBo_temp = t_arrREDlightBuff[k];
              t_arrREDlightBuff[k] = t_arrREDlightBuff[k+1];
              t_arrREDlightBuff[k+1] = LvBo_temp;
            }
          }
        }
        RED_LvBo_resPLUSE = t_arrREDlightBuff[(MEANLVBO_POINTNUM+1)/2];  
        
        RED300Data[s_iCnt300] = RED_LvBo_resPLUSE;
      
        temp = ((RED_LvBo_res - 1300)*2)+10;         //将数据转为8位
        //printf("adc = %.2f\r\n",RED_LvBo_res);
        //printf("adc = %.1f\r\n",RED_LvBo_resNEW);
        //printf("adc = %d\r\n",(u16)RED_LvBo_resPLUSE);
        RED_LvBo_res = 0;
        RED_LvBo_sum = 0;
      
        RedWaveDataToHost[point_num] = (u8)temp;     
      //pct模块  
        point_num++;
        if(point_num >=4)
        {
        
          SendSpo2WaveToHost(RedWaveDataToHost,ProSign);                  //通过usart发送数据
          point_num = 0;
        }
      
        RED_LvBo_avg = RED_LvBo_avg + RED_LvBo_res;
      
        s_iCnt100++;
        if(s_iCnt100 >=100)
        {
          SendSpo2DataToHost((u8)0, (i16)Rate, (u8)Spo2);
        }
      
        //准备下次的循环
        s_iCnt300++;
        if(s_iCnt300>=300)
        {

//         RED_LvBo_avg = RED_LvBo_avg / 300;//计算均值

//船新版本
          find_peaks_and_troughs(RED300Data, 300, REDMaxPeaks, &REDMax_len, REDMinPeaks, &REDMin_len);
          Rate = GetPulse(REDMaxPeaks, &REDMax_len, REDMinPeaks, &REDMin_len,RED300Data);      
          //printf("[[1,%d]]\r\n",Rate);
          s_iCnt300 = 0;
        }
        s_iCnt5 = 0;
      
//        printf("REDlight = %d\n",REDlight);
//        printf("IRlight = %d\n",IRlight);
      }
    }
    
    else
    {
      
    }
    
    Clr2msFlag();   //清除2ms标志
  }
  
}

/*********************************************************************************************************
* 函数名称：Proc500usTask
* 函数功能：500us处理任务 
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2018年01月01日
* 注    意：
*********************************************************************************************************/
static  void  Proc500usTask(void)
{   
  static u16 s_iCnt300 = 0;       //300个点的计数器
  static u16 s_iCnt20 = 0;        //20个点的计数器
    if(Get500usFlag() == 1) //判断500us标志状态  
    {
      s_iCnt20++;
      if (s_iCnt20 == 2)
      {
        SetDACOut(REDlight);
        GPIO_SetBits(GPIOA,GPIO_Pin_2);   //开红光
        GPIO_ResetBits(GPIOA,GPIO_Pin_3); //关红外
      }
      else if (s_iCnt20 == 5)
      {
        ReadSPO2ADCBuf(&s_iRED);              //用ADC采样红光数据
        RED_penData[s_iCnt300] = s_iRED; //将红光数据保存到2s一组的数组中
      }
      else if (s_iCnt20 == 6)
      {
        GPIO_ResetBits(GPIOA,GPIO_Pin_2); //关红光
        GPIO_ResetBits(GPIOA,GPIO_Pin_3); //关红外
      }
      else if (s_iCnt20 == 10)
      {
        SetDACOut(IRlight);
        GPIO_ResetBits(GPIOA,GPIO_Pin_2); //关红光
        GPIO_SetBits(GPIOA,GPIO_Pin_3);   //开红外
      }
      else if (s_iCnt20 == 13)
      {
       ReadSPO2ADCBuf(&s_iIR);                //用ADC采样红外数据
       IR_penData[s_iCnt300] = s_iIR;     //将红外数据保存到2s一组的数组中
        
      }
      else if (s_iCnt20 == 14)
      {
        GPIO_ResetBits(GPIOA,GPIO_Pin_2); //关红光
        GPIO_ResetBits(GPIOA,GPIO_Pin_3); //关红外
      }
      else if (s_iCnt20 >= 20)
      {
//        printf("adc = %d\r\n",s_iRED);
        if(s_iRED >= 2040)
        {
          ProSign = 0x90;///////////////////////////////////////////没接探头标志+没接手指标志
        }
        else if (s_iRED <= 100)
        {
          ProSign = 0x80;///////////////////////////////////////////没接手指标志
        }
        else
        {
          ProSign = 0x00;///////////////////////////////////////////双接入
        }
        s_iCnt300++;
        s_iCnt20=0;
        if (s_iCnt300 >=300)
        {
          Spo2 = SpoValNum( RED_penData, IR_penData);
          //printf("[[4,%d]]\r\n",Spo2);
          s_iCnt300 = 0;
        }
          //printf("[[3,%d]]\r\n",REDlight);
          //printf("[[5,%d]]\r\n",IRlight);
      }
    }
  
      Clr500usFlag();  //清除500us标志
}       

/*********************************************************************************************************
* 函数名称：Proc100msTask
* 函数功能：2ms处理任务 
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2018年01月01日
* 注    意：
*********************************************************************************************************/
static  void  Proc100msTask(void)
{  

  if(Get100msFlag())  //判断100ms标志状态
  { 
    
    Clr100msFlag();   //清除100ms标志
  }
}

/*********************************************************************************************************
* 函数名称：Proc60msTask
* 函数功能：60ms处理任务
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2018年01月01日
* 注    意：
*********************************************************************************************************/
static  void  Proc60msTask(void)   // 通道切换功能，为后续采集各个电压数据和计算体温数据做准备
{ 
  if(Get60msFlag())                // 判断60ms标志状态
  {
    if(keyNumber == 1)             // 通道切换从PA通道开始
    {
      TempWriteBit(T_PA);          // PA通道相关引脚赋予高电平（工作）状态
    }
    else if(keyNumber == 2)        // 通道切换为PB通道
    {
      TempWriteBit(T_PB);          // PB通道相关引脚赋予高电平（工作）状态
    }
    else if(keyNumber == 3)        // 通道切换为S1/S2测量通道
    {
      if(BoolKeyRenew() == FALSE)  // 检查按键选择的通道是否为S1/S2测量通道
      {
        if(BoolKeyNum() == 1)      // 判断通道是S1还是S2通道
        {
          TempWriteBit(T_SENS1);   // S1通道相关引脚赋予高电平（工作）状态
        }
        else
        {
          TempWriteBit(T_SENS2);   // S2通道相关引脚赋予高电平（工作）状态
        }
      }
    }
    else                           // 通道停止自动切换
    {
      
    }
    
    if(keyNumber != 0)             // 当通道切换从每个通道结束之后
    {
      s_i20msBegin = TRUE;         // 开启20ms数据采集的计时
    }
      
    Clr60msFlag();  //清除60ms标志

  }
}

/*********************************************************************************************************
* 函数名称：Proc60msAfter20msTask
* 函数功能：60ms后紧跟着的20ms处理任务
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2018年01月01日
* 注    意：
*********************************************************************************************************/
static  void  Proc60msAfter20msTask(void)    // 采集每个通道的电压数据与计算探头阻值与体温数据
{ 
  static double SENS_Val;    // S1/S2通道的电压
  u16 adcTempData;            // 经过3V3电压换算的adc数据
  if(Get60msAfter20msFlag()) // 判断60ms后紧跟着的20ms标志状态
  {
    if(keyNumber == 1)       // 通道切换为PA通道
    {      
      GetADC1Data(&adcTempData); // 获取adc的数据

      PA_Val = adcTempData * 3.3 / 4095.0;    // 获取到的adc数据为PA通道的电压数据
    }
    else if(keyNumber == 2)  // 通道切换为PB通道
    {
      GetADC1Data(&adcTempData); // 获取adc的数据

      PB_Val = adcTempData * 3.3 / 4095.0;    // 获取到的adc数据为PB通道的电压数据
      ReckonToC1AndC2(PA_Val,PB_Val,&C1,&C2); //通过上述PA与PB各自通道获取到的电压数据从而计算出C1与C2系数
    }
    else if(keyNumber == 3)  // 通道切换为S1/S2通道
     {
      if((BoolKeyModel() == 1)||(BoolKeyModel() == 2))   // 确定按钮选择的通道为S1/S2通道
      {
        ReadTOffValue = (BitAction)GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5);  // 获取T_SENS_OFF引脚的电平变化（高与低）
        if(!ReadTOffValue)            // 当T_SENS_OFF引脚出现低电平（导联没有脱落的情况）
        {
          GetADC1Data(&adcTempData);      // 获取adc的数据 

          SENS_Val = adcTempData * 3.3 / 4095.0;       // 获取到的adc数据为S1/S2通道的电压数据          
          reckonBegin = TRUE;       // 成功采集到S1/S2通道的电压数据，可以计算对应探头的阻值与温度值
        }
        else                          // 当T_SENS_OFF引脚出现高电平（导联脱落的情况）
        {
          
        }
  
        // 采集完对应通道的电压数据后将通道恢复成初始按钮代表的通道，目的是要在串口波形小工具中实时显示该通道的电压数据
        if(BoolKeyModel() == 2)       // 将此刻通道恢复成S1通道  
        {
          TempWriteBit(T_SENS1);      // S1通道相关引脚赋予高电平（工作）状态
        }
        else                           // 将此刻通道恢复成S2通道  
        {
          TempWriteBit(T_SENS2);       // S2通道相关引脚赋予高电平（工作）状态
        }
      }
      else if(BoolKeyModel() == 3)     // 将此刻通道恢复成PA通道  
      {
        TempWriteBit(T_PA);            // PA通道相关引脚赋予高电平（工作）状态     
      }
      else if(BoolKeyModel() == 0)     // 将此刻通道恢复成PB通道  
      {
        TempWriteBit(T_PB);            // PB通道相关引脚赋予高电平（工作）状态
      }
      else                             // 其他情况（严谨）
      {
        
      }
    
    }
    else                               // 其他情况（严谨）
    {
      
    }
    
    keyNumber = (keyNumber + 1) & 3;                           // 通道切换为下一个通道（当keyNumber为0时通道切换结束）
    
    if(reckonBegin)                                            // 计算探头阻值与温度值，reckonBegin为1说明已成功采集到S1/S2通道的电压数据
    {
      reckonBegin = FALSE;                                     // 为下次判断是否成功采集到S1/S2通道的电压数据做准备
      RextVariate = RextReckon(SENS_Val,C1,C2);                // 计算探头阻值
      arrTempVariate[arrNum] = RextTurnToTemp(RextVariate);    // 计算温度值并储存在体温数组中
      arrNum++;                                                // 数组元素个数加1
      keyBegin++;                                              // 进入到下一轮
    }
    
    if((keyBegin <= 4) && (keyNumber == 0) && (ReadTOffValue == 0))  
  // 都满足以下条件时体温通道继续自动切换：
  // 1、1s时间内还没有采集完5个体温值
  // 2、已采集到体温数据
  // 3、导联不脱落
    {
      keyNumber = 1;       // 体温通道自动切换为PA通道
    }
    else if(ReadTOffValue) // 导联脱落时
    {
      keyBegin = 0;        // 回到未采集体温数据的状态
      keyNumber = 0;       // 通道停止自动切换     
    }
    
    Clr60msAfter20msFlag();  //清除60ms后紧跟着的20ms标志
  }
}

/*********************************************************************************************************
* 函数名称：Proc1SecTask
* 函数功能：1s处理任务 
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2018年01月01日
* 注    意：
*********************************************************************************************************/
static  void  Proc1SecTask(void)               
// 体温数据包的状态与数据更新并将打包好的体温数据包发送到上位机，还有其他变量状态的更新
{ 
/************************************************体温模块***************************************************/
  u16  adcTempData;                             // 已经过3V3电压换算的adc数据
  i8 i;                               // PCT协议中体温数据包的数组下标
/**********************************************************************************************************/
  
/************************************************心电模块***************************************************/
  static u8 BpmHighBit8;  //高字节数据
  static u8 BpmLowBit8;  //低字节数据
  static u8 s_arrEcgHrData[6] = {0};       //初始化心率数组,用于存放待打包的心率数据 
  static u8 s_arrEcgLeadOffData[6] = {0};  //初始化心点导联信息数组,用于存放待打包的心电导联信息数据 
  static u8 LeadOff;  //导联脱落变量：1或0
/***********************************************************************************************************/
  
  if(Get1SecFlag()) //判断1s标志状态
  {
    if(GetStateNum() == 1)    // 切换到体温模块
    {
      stateTempOff = 2;
      GetADC1Data(&adcTempData);                     // 获取在S1/S2通道下的adc数据
      
      ReadTOffValue = (BitAction)GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5);  // 获取T_SENS_OFF引脚的电平变化（高与低）
      
      if(keyChange)                            // 单片机按钮被按下选择通道
      {
        keyChange = FALSE;                     // 执行完按钮按下并在500ms时间中的任务后将值改为false
        keyBegin = 5;                          // 允许通道切换从起始值无通道开始按照keyNumber规定的通道切换顺序执行
      }
      
      if((BoolKeyModel() == 1)||(BoolKeyModel() == 2) ||(ReadTOffValue == 1))   // 处理体温值并发送体温数据到上位机
      // 以下三种情况满足if条件：
      // 1、选择S1、S2测量通道
      // 2、体温探头导联脱落
      { 
        ChangeTempDataPacket();                // 将采集到的5个体温值求平均值并把数据发送到PCT协议当中体温数据包的数组
        SendTempToHost(s_arrTempData);         // 发送数据包到上位机
      }
      
      InitValue();          // 初始化变量
        
    
      if(keyBegin == 5)       // 重置keyBegin与keyNumber值表示从头开始按照keyNumber规定的通道自动切换采集体温值
      {
        keyBegin = 0;         // 体温数据处于未采集的状态
        keyNumber = 1;        // 通道切换设置为PA通道
      }
    
      if(ReadTOffValue == 1)  // 体温探头导联脱落的情况
      {
        RestoreJudge = TRUE;  // 为体温探头导联重新连接时从头开始采集数据做准备
        keyBegin = 0;         // 体温数据处于未采集的状态
        keyNumber = 0;        // 通道切换停止
      }
      else                    // 体温探头导联重新连接的情况
      {
        if(RestoreJudge)      // 满足上一秒导联脱落且这一秒导联连接时重置keyBegin与keyNumber值表示从头开始按照keyNumber规定的通道自动切换采集体温值
        {
          keyBegin = 0;       // 体温数据处于未采集的状态
          keyNumber = 1;      // 通道切换设置为PA通道
          RestoreJudge = FALSE;  // 完成通道自动切换与体温采集轮数的初始化
        }
      }
    }
    
    else if(GetStateNum() == 2)  // 切换到心电模块
    {
      LeadOff = GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7);  //读取ECG_LEAD_OFF引脚电平
      //打包心率数据并发送
      BpmHighBit8 = (BPM >> 8) & 0xFF;  //右移8位，取高八位，实际是adc值的高4位
      BpmLowBit8 = BPM & 0xFF;  //取低8位
      s_arrEcgHrData[0] = BpmHighBit8;  //心率高字节存放到数组第0位，对应数据包的DAT1
      s_arrEcgHrData[1] = BpmLowBit8;  //心率低字节存放到数组第1位，对应数据包的DAT2
      SendEcgHrToHost(s_arrEcgHrData);  //发送心率数据包

      //打包心电导联信息数据并发送
      if(LeadOff == 0 )  //引脚低电平：导联连接
      {
        s_arrEcgLeadOffData[0] = LeadOff;
      }
      if(LeadOff == 1)  //高电平：导联脱落
      {
        LeadOff = LeadOff |0x0E;  //位1~3置1,因为电路无法判断具体是哪个导联脱落
        s_arrEcgLeadOffData[0] = LeadOff;  //存放到数组第0位，对应数据包的DAT1
      }
      SendEcgLeadToHost(s_arrEcgLeadOffData);  //发送心电导联信息数据包

    }
    
    if((GetStateNum() != 1)&&(stateTempOff != 0))
    {
      stateTempOff = (stateTempOff + 1) % 2;
      
      s_arrTempData[0] = 3;    // 体温探头状态
      for(i = 1;i < 5;i++)       // 体温通道1与2数据清空
      {
        s_arrTempData[i] = 0;
      }
      SendTempToHost(s_arrTempData);         // 发送数据包到上位机
//      printf("off\n");
    }
    
    Clr1SecFlag();  //清除1s标志
  }    
}

/*********************************************************************************************************
* 函数名称：main
* 函数功能：主函数 
* 输入参数：void
* 输出参数：void
* 返 回 值：int
* 创建日期：2018年01月01日
* 注    意：
*********************************************************************************************************/
int main(void)
{ 
  InitHardware();   //初始化硬件相关函数
  InitSoftware();   //初始化软件相关函数
  
  printf("Init System has been finished.\r\n" );  //打印系统状态

  while(1)
  {
    // 2ms处理任务：               按键扫描与识别（每10ms进行一次按键识别）              
    // 60ms处理任务：              通道切换功能，为后续采集各个电压数据和计算体温数据做准备
    // 60ms后紧跟着的20ms处理任务： 采集每个通道的电压数据与计算探头阻值与体温数据
    // 1s处理任务：                体温数据包的状态与数据更新并将打包好的体温数据包发送到上位机（在串口波形显示小工具中不需要这部分功能），还有其他变量状态的更新
    
/************************************************血氧模块***************************************************/
    if(GetStateNum() == 3)
    {
      Proc500usTask(); //500us处理任务     
      Proc100msTask();
    }
/**********************************************************************************************************/
   
    Proc2msTask();             // 2ms处理任务
    
/************************************************体温模块***************************************************/
    if(GetStateNum() == 1)
    {
      Proc60msTask();            // 60ms处理任务
      Proc60msAfter20msTask();   // 60ms后紧跟着的20ms处理任务
    }
/**********************************************************************************************************/
    Proc1SecTask();            // 1s处理任务   
  }
}

/*********************************************************************************************************
*                                              API函数实现
*********************************************************************************************************/
/*********************************************************************************************************
* 函数名称：InitVariable
* 函数功能：初始化参数
* 输入参数：
* 输出参数：
* 返 回 值：
* 创建日期：2018年01月01日
* 注    意：
*********************************************************************************************************/
void InitVariable(void)
{  
  InitValue();           // 初始化变量
  
  keyNumber = 0;         // 体温通道停止自动切换
  keyChange = FALSE;     // 没有按动按钮选择通道
  s_i20msBegin = FALSE;  // 60ms通道切换任务没开始，20ms数据采集计时关闭
  reckonBegin = FALSE;   // 没有采集到S1/S2通道的电压数据，不计算阻值与温度值
  RestoreJudge = FALSE;  // 不进行体温探头脱落从头开始采集数据的操作
  keyBegin = 0;          // 未采集到体温数据
  ReadTOffValue = 1;     // T_off位置电压为高电平
}

/*********************************************************************************************************
* 函数名称：InitValue
* 函数功能：初始化变量
* 输入参数：
* 输出参数：
* 返 回 值：
* 创建日期：2018年01月01日
* 注    意：
*********************************************************************************************************/
void InitValue(void)
{
  u8 i;
  
  PA_Val = 0.0;            // PA通道的val电压值
  PB_Val = 0.0;            // PB通道的val电压值
  C1 = 14700.0;            // C1系数
  C2 = 6.25;               // C2系数
  RextVariate = 0.0;       // 体温探头阻值
  arrNum = 0;              // 储存温度值的数组的元素个数

  for(i = 0;i < 10;i++)    // 储存温度值的数组初始化
  {
    arrTempVariate[i] = -10.0; 
  }
}

/*********************************************************************************************************
* 函数名称：GetArrNum
* 函数功能：获取储存温度值的数组的元素个数
* 输入参数：
* 输出参数：
* 返 回 值：
* 创建日期：2018年01月01日
* 注    意：
*********************************************************************************************************/
u8 GetArrNum(void)
{
  return arrNum;
}

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
