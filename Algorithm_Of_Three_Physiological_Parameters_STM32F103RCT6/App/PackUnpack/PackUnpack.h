/*********************************************************************************************************
* 模块名称：PackUnpack.h
* 摘    要：PackUnpack模块
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
#ifndef _PACK_UNPACK_H_
#define _PACK_UNPACK_H_

/*********************************************************************************************************
*                                              包含头文件
*********************************************************************************************************/
#include "DataType.h"
#include "UART1.h"

/*********************************************************************************************************
*                                              宏定义
*********************************************************************************************************/

/*********************************************************************************************************
*                                              枚举结构体定义
*********************************************************************************************************/
//包类型结构体
typedef struct
{
  u8 packModuleId;    //模块包ID
  u8 packHead;        //数据头
  u8 packSecondId;    //二级ID
  u8 arrData[6];      //包数据
  u8 checkSum;        //校验和
}StructPackType;

//枚举定义，定义模块ID，0x00～0x7F，不可以重复
typedef enum
{
  MODULE_SYS     = 0x01,  //系统信息
  MODULE_ECG     = 0x10,  //心电信息
  MODULE_RESP    = 0x11,  //呼吸信息
  MODULE_TEMP    = 0x12,  //体温信息
  MODULE_SPO2    = 0x13,  //血氧信息
  MODULE_NBP     = 0x14,  //无创血压信息

  MODULE_WAVE    = 0x71,  //wave模块信息

  MAX_MODULE_ID  = 0x80
}EnumPackID;

//定义二级ID，0x00～0xFF，因为是分属于不同的模块ID，因此不同模块ID的二级ID可以重复
//系统模块的二级ID
typedef enum 
{
  DAT_RST         = 0x01,         //系统复位信息
  DAT_SYS_STS     = 0x02,         //系统状态
  DAT_SELF_CHECK  = 0x03,         //系统自检结果
  DAT_CMD_ACK     = 0x04,         //命令应答
  
  CMD_RST_ACK     = 0x80,         //模块复位信息应答
  CMD_GET_POST_RSLT = 0x81,       //读取自检结果
  CMD_PAT_TYPE    = 0x90,         //病人类型设置  
}EnumSysSecondID;

//ECG模块的二级ID
typedef enum 
{
  DAT_ECG_WAVE    = 0x02,         //心电波形数据
  DAT_ECG_LEAD    = 0x03,         //心电导联信息
  DAT_ECG_HR      = 0x04,         //心率
  DAT_ST          = 0x05,         //ST值
  DAT_ST_PAT      = 0x06,         //ST模板波形
  
  CMD_LEAD_SYS    = 0x80,         //3/5导联设置
  CMD_LEAD_TYPE   = 0x81,         //导联方式设置
  CMD_FILTER_MODE = 0x82,         //心电滤波方式设置
  CMD_ECG_GAIN    = 0x83,         //ECG增益设置
  CMD_ECG_CAL     = 0x84,         //心电校准
  CMD_ECG_TRA     = 0x85,         //工频干扰抑制开关
  CMD_ECG_PACE    = 0x86,         //起搏分析开关
  CMD_ECG_ST_ISO  = 0x87,         //ST测量ISO、ST点
  CMD_ECG_CHANNEL = 0x88,         //心率计算通道
  CMD_ECG_LEADRN  = 0x89,         //心率重新计算  
}EnumECGSecondID;

//Resp模块的二级ID
typedef enum 
{
  DAT_RESP_WAVE   = 0x02,         //呼吸波形数据
  DAT_RESP_RR     = 0x03,         //呼吸率
  DAT_RESP_APNEA  = 0x04,         //窒息报警
  DAT_RESP_CVA    = 0x05,         //呼吸CVA报警信息
  
  CMD_RESP_GAIN   = 0x80,         //呼吸增益设置
  CMD_RESP_APNEA  = 0x81,         //呼吸窒息报警时间设置  
}EnumRespSecondID;
  
//Temp模块的二级ID
typedef enum 
{
  DAT_TEMP_DATA   = 0x02,         //体温数据
  
  CMD_TEMP        = 0x80,         //体温参数设置
}EnumTempSecondID;

//SPO2模块的二级ID
typedef enum 
{
  DAT_SPO2_WAVE   = 0x02,         //血氧波形
  DAT_SPO2_DATA   = 0x03,         //血氧数据
  
  CMD_SPO2        = 0x80,         //血氧参数设置
}EnumSPO2SecondID;

//NBP模块的二级ID
typedef enum 
{
  DAT_NBP_CUFPRE  = 0x02,         //无创血压实时数据
  DAT_NBP_END     = 0x03,         //无创血压测量结束
  DAT_NBP_RSLT1   = 0x04,         //无创血压测量结果1 
  DAT_NBP_RSLT2   = 0x05,         //无创血压测量结果2
  DAT_NBP_STS     = 0x06,         //无创血压状态
  
  CMD_NBP_START   = 0x80,         //NBP启动测量
  CMD_NBP_END     = 0x81,         //NBP中止测量
  CMD_NBP_PERIOD  = 0x82,         //NBP测量周期设置
  CMD_NBP_CALIB   = 0x83,         //NBP校准
  CMD_NBP_RST     = 0x84,         //NBP模块复位
  CMD_NBP_CHECK_LEAK  = 0x85,     //NBP漏气检测
  CMD_NBP_QUERY_STS   = 0x86,     //NBP查询状态
  CMD_NBP_FIRST_PRE   = 0x87,     //NBP首次充气压力设置
  CMD_NBP_CONT    = 0x88,         //开始5分钟的STAT血压测量
  CMD_NBP_RSLT    = 0x89,         //NBP查询上次测量结果  
}EnumNBPSecondID;

//wave模块的二级ID
typedef enum 
{   
  DAT_WAVE_WDATA  = 0x01,         //wave模块波形数据 
  
  CMD_GEN_WAVE    = 0x80,         //wave模块生成波形命令
}EnumWaveSecondID;

  
/*********************************************************************************************************
*                                              API函数声明
*********************************************************************************************************/
void  InitPackUnpack(void);               //初始化PackUnpack模块
u8    PackData(StructPackType* pPT);      //对数据进行打包，1-打包成功，0-打包失败
u8    UnPackData(u8 data);                //对数据进行解包，1-解包成功，0-解包失败

StructPackType  GetUnPackRslt(void);      //读取解包后数据包

#endif
