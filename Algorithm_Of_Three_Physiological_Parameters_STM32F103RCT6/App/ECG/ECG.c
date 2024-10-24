/*********************************************************************************************************
* ģ�����ƣ�ECG.c
* ժ    Ҫ��ECGģ��
* ��ǰ�汾��1.0.0
* ��    �ߣ�SZLY(COPYRIGHT 2018 - 2020 SZLY. All rights reserved.)
* ������ڣ�2020��01��01�� 
* ��    �ݣ�
* ע    �⣺                                                                  
**********************************************************************************************************
* ȡ���汾��
* ��    �ߣ�
* ������ڣ�
* �޸����ݣ�
* �޸��ļ���
*********************************************************************************************************/

/*********************************************************************************************************
*                                              ����ͷ�ļ�
*********************************************************************************************************/
#include "ECG.h"
#include "stm32f10x_conf.h"
#include "Timer.h"
#include "ADC.h"
#include "Filter.h"
#include "SendDataToHost.h"
//#include "LED.h"

/*********************************************************************************************************
*                                              �궨��
*********************************************************************************************************/

/*********************************************************************************************************
*                                              ö�ٽṹ�嶨��
*********************************************************************************************************/


/*********************************************************************************************************
*                                              �ڲ�����
*********************************************************************************************************/
//u16 IBI;  //�����������
static u16 BPM;  //����
static u8 s_iLeadOff;  //�������������1�����䣻0������

/*********************************************************************************************************
*                                              �ڲ�����ʵ��
*********************************************************************************************************/
/*********************************************************************************************************
* �������ƣ�GetArrMax
* �������ܣ��������ֵ
* ���������void
* ���������void
* �� �� ֵ��void
* �������ڣ�2018��01��01��
* ע    �⣺
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
* �������ƣ�GetArrMin
* �������ܣ�������Сֵ
* ���������void
* ���������void
* �� �� ֵ��void
* �������ڣ�2018��01��01��
* ע    �⣺
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
* �������ƣ�CoutThreshold
* �������ܣ�������ֵ
* ���������void
* ���������void
* �� �� ֵ��void
* �������ڣ�2018��01��01��
* ע    �⣺
*********************************************************************************************************/
static float CoutThreshold(u16 Max,u16 Min)
{
  return (Max+Min)*0.52;
}




/*********************************************************************************************************
*                                              API����ʵ��
*********************************************************************************************************/
/*********************************************************************************************************
* �������ƣ�InitWave
* �������ܣ���ʼ��Waveģ�� 
* ���������void
* ���������void
* �� �� ֵ��void
* �������ڣ�2018��01��01��
* ע    �⣺
*********************************************************************************************************/
/*********************************************************************************************************
* �������ƣ�InitConfigTemp
* �������ܣ���������ͨ����ص�GPIO��RCC
* ���������void
* ���������void
* �� �� ֵ��void
* �������ڣ�2018��01��01��
* ע    �⣺
*********************************************************************************************************/
void  SendECGWaveAndHr(void)
{
  static u8 s_iCnt4 = 0;   //8ms������  
  static u16 adcData;       //ԭʼ�ĵ粨��adc���� 
  static int firstEffectivePulse  = 0;     //��һ����Ч�������±�
  int secondEffectivePulse  = 0;     //�ڶ�����Ч�������±�
  static u16 maxAdcData;    //һ������adc���������ֵ
  static u16 minAdcData;    //һ������adc��������Сֵ
  static u16 preThreshold;  //һ�����ڲ�����������ֵ
  //static u16 s_arrECGdata[2000]={0};//���һ�����ڵ�ԭʼ�ĵ�����ECG_ADC,�����˲�
  static u16 s_arrFilteredECGData[2000] = {0}; //����2000�����飬����˲����adc�����ڼ�������
  static u16 Index = 0;        //�����±�
  static u16 DATA_SIZE = 2000; //���鳤��
  static u8 s_bPreDataStatus = FALSE;  //ǰһ�����״̬��Ĭ��FALSE
  static u8 s_bCurrDataStatus = FALSE;  //��ǰ�õ��״̬��Ĭ��FALSE
  static int s_iPulseCnt;  //��Ч����������
  static u16 refractoryPeriod = 93; // bpm=320�Ĳ�Ӧ�ڵĳ��ȣ������ݵ�Ϊ��λ��
  
  static u8 s_arrECGWaveData[6] = {0}; //��ʼ���ĵ粨������,���ڴ�Ŵ�������ĵ粨������
  u8 waveDataHigh8;  //�������ݸ��ֽ�
  u8 waveDataLow8;  //�������ݵ��ֽ�
  double LowpassFIRVAL;  //��ͨ�˲���ı���
  static float s_arrInputSignals_x[3] = {0};  // ���ڴ洢��ȥ�����źŵ�����
  static float s_arrInputSignals_y[3] = {0};  // ���ڴ洢��ȥ����źŵ�����
  float adcAfterLowpass;  //��ͨ������ݣ���Ϊ��ͨ�˲�������
  u16 filteredAdc;  //��ͨ����ͨ�˲���������ĵ�����
  
  s_iLeadOff = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7);//��ȡ�������ŵ�ƽ״̬
  s_iCnt4++;  //��������,���ڷ����ĵ����ݰ�
  if(s_iLeadOff==0) //��������
  {
    if(ReadECGADCBuf(&adcData))  //2msȡ��1������
    {      
      LowpassFIRVAL = LowpassFilter_FIR(adcData);  //��ͨFIR���˳���Ƶ����
      adcAfterLowpass = (float)LowpassFIRVAL;
      highpassFilter_IIR(&adcAfterLowpass,s_arrInputSignals_x,s_arrInputSignals_y);//IIR��ͨ,�˳�����Ư��
      adcAfterLowpass += 2048.0;  //����2048
      filteredAdc = (u16)adcAfterLowpass;
            
      
      //ÿ4s����һ������
      //����Ϊ2000��������adcֵ��ÿ2ms����һ��adcֵ������һ��Ϊһ�����ڲ���
      s_arrFilteredECGData[Index] = filteredAdc;  
      Index++;  //ÿ2ms����+1����ͨ���±���������

      //������ֵ����������ͼ���һ���µ���ֵ��������һ����������ݽ�����ֵ�жϣ����жϵ�ǰ���adc�Ƿ�������ֵ
      if(Index >= DATA_SIZE)  
      {
        Index = 0;
        maxAdcData = GetArrMax(s_arrFilteredECGData,DATA_SIZE);  //�������ֵ
        minAdcData = GetArrMin(s_arrFilteredECGData,DATA_SIZE);  //������Сֵ
        preThreshold = CoutThreshold(maxAdcData,minAdcData);  //���㵱ǰһ������1500������ֵ����ֵ 
      }

      s_bPreDataStatus = s_bCurrDataStatus;  //���浱ǰ����״̬����ʼΪFALSE       
      if(filteredAdc > preThreshold)// ����ֵ������ֵ������״̬���ΪTRUE
      {
        s_bCurrDataStatus = TRUE;  //TRUE
      }
      else
      {
        s_bCurrDataStatus = FALSE;  //����ΪFALSE
      }
      
      //��ĳ�����ǰһ����С�ڵ�����ֵ�Ҹõ������ֵ���ж�Ϊ��Ч����
      if(s_bPreDataStatus == FALSE && s_bCurrDataStatus == TRUE) 
      {
        if (((Index - firstEffectivePulse ) > refractoryPeriod)||((firstEffectivePulse >Index)&&(2000-firstEffectivePulse +Index)>refractoryPeriod))  // ����Ƿ��ڲ�Ӧ��֮��
        {        
          s_iPulseCnt++;    //��Ч�������+1  
        
          if((s_iPulseCnt % 2) == 1)  //��һ������
          {
            firstEffectivePulse  = Index;  // ��¼��һ����Ч����ʱ�� 
          }
          if((s_iPulseCnt % 2) == 0)  // �ڶ�������,��������
          {
            secondEffectivePulse  = Index;  // ��¼�ڶ�����Ч����ʱ��

            if(secondEffectivePulse  < firstEffectivePulse )
            {
              //IBI = (secondEffectivePulse  - firstEffectivePulse ) * 2; //���������������
              //BPM = 60000/IBI;
              BPM = 30000/(secondEffectivePulse  + 2000 - firstEffectivePulse );  //����BPM
            }
            else
            {
              BPM = 30000/(secondEffectivePulse  - firstEffectivePulse );  //����BPM
            }         
          }
          //lastPulseTime = Index; // ������һ��������±�       
        }       
      }      
    }
    //LEDFlicker(250);//������˸����   
  }
  if(s_iLeadOff == 1)  //�ߵ�ƽ���������� //��������
  {
    filteredAdc = 2048;  //�������
  }
  if(s_iCnt4 >= 4 )  //�ﵽ8ms������һ���ĵ粨�����ݰ�
  {      
    //12λadcֵ��������8λ��ţ��������ֽ�
    waveDataHigh8 = (filteredAdc >> 8) & 0xFF;  //����8λ��ȡ�߰�λ��ʵ����adcֵ�ĸ�4λ
    waveDataLow8 = filteredAdc & 0xFF;  //ȡ��8λ
    
    //����������ݷ��͵���λ��
    s_arrECGWaveData[0] = waveDataHigh8;  //���ֽڴ�ŵ������0λ����Ӧ���ݰ���DAT1
    s_arrECGWaveData[1] = waveDataLow8;  //���ֽڴ�ŵ������1λ����Ӧ���ݰ���DAT2
    SendWaveToHost(s_arrECGWaveData);  //�����ĵ粨�����ݰ�
    s_iCnt4 = 0;  //׼���´ε�ѭ��
  }
  //Clr2msFlag();   //���2ms��־

}

/*********************************************************************************************************
* �������ƣ�Proc1SecTask
* �������ܣ�1s�������񣬷����������ݰ����ĵ絼����Ϣ���ݰ�
* ���������void
* ���������void
* �� �� ֵ��void
* �������ڣ�2018��01��01��
* ע    �⣺
*********************************************************************************************************/
void  SendECGLeadData(void)
{ 
  u8 bpmHighBit8;  //���ֽ�����
  u8 bpmLowBit8;  //���ֽ�����
  static u8 s_arrEcgHrData[6] = {0};       //��ʼ����������,���ڴ�Ŵ�������������� 
  static u8 s_arrEcgLeadOffData[6] = {0};  //��ʼ���ĵ㵼����Ϣ����,���ڴ�Ŵ�������ĵ絼����Ϣ���� 
//  static u8 s_iLeadOff;  //�������������1��0
//  s_iLeadOff = GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7);  //��ȡECG_LEAD_OFF���ŵ�ƽ

  if(Get1SecFlag()) //�ж�1s��־״̬
  {
    s_iLeadOff = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7);//��ȡ������ƽ
    
    if(s_iLeadOff == 0 )  //���ŵ͵�ƽ����������
    {    
      s_arrEcgLeadOffData[0] = s_iLeadOff;
      
      if(( 28> BPM ) || ( BPM>253))  //�ж�BPM�Ƿ�����Ч��Χ30-240������5��
      { 
        BPM = 400;        
      }
     }

      //if(s_iLeadOff == 1)  //�ߵ�ƽ����������
    else  //���Ÿߵ�ƽ���������� 
    {
      s_iLeadOff = s_iLeadOff |0x0E;  //λ1~3��1,��Ϊ��·�޷��жϾ������ĸ���������
      s_arrEcgLeadOffData[0] = s_iLeadOff;  //��ŵ������0λ����Ӧ���ݰ���DAT1
      
      BPM = 400; //��Ч����ֵ     
    } 
    SendEcgLeadToHost(s_arrEcgLeadOffData);  //�����ĵ絼����Ϣ���ݰ�
    //����������ݲ�����
    bpmHighBit8 = (BPM >> 8) & 0xFF;  //����8λ��ȡ�߰�λ��ʵ����adcֵ�ĸ�4λ
    bpmLowBit8 = BPM & 0xFF;  //ȡ��8λ
    s_arrEcgHrData[0] = bpmHighBit8;  //���ʸ��ֽڴ�ŵ������0λ����Ӧ���ݰ���DAT1
    s_arrEcgHrData[1] = bpmLowBit8;  //���ʵ��ֽڴ�ŵ������1λ����Ӧ���ݰ���DAT2      
    SendEcgHrToHost(s_arrEcgHrData);  //�����������ݰ�  
    
    //Clr1SecFlag();  //���1s��־
  }    
}
