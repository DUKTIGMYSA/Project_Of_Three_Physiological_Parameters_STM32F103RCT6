/*********************************************************************************************************
* ģ�����ƣ�Temp.c
* ժ    Ҫ�����²�����ģ�飬����ģ���ʼ�����Լ����ݴ����ѹ����������¶�ֵ
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
#include "Temp.h"
#include "Main.h"

/*********************************************************************************************************
*                                              �궨��
*********************************************************************************************************/

/*********************************************************************************************************
*                                              ö�ٽṹ�嶨��
*********************************************************************************************************/            

/*********************************************************************************************************
*                                              �ڲ�����
*********************************************************************************************************/    
extern u8 s_arrTempData[6];          // ��ʾ����PCTЭ�����������ݰ�������
extern double arrTempVariate[10];    // ��ʾ���Ǵ����¶�ֵ������ 
/*********************************************************************************************************
*                                              �ڲ���������
*********************************************************************************************************/

  
/*********************************************************************************************************
*                                              �ڲ�����ʵ��
*********************************************************************************************************/

/*********************************************************************************************************
*                                              API����ʵ��
*********************************************************************************************************/
/*********************************************************************************************************
* �������ƣ�InitTemp
* �������ܣ���ʼ���¶Ȳ�����ģ��
* ���������void
* ���������void
* �� �� ֵ��void 
* �������ڣ�2018��01��01��
* ע    �⣺
*********************************************************************************************************/
void  InitTemp(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;   //GPIO_InitStructure���ڴ��GPIO�Ĳ���
  
  //ʹ��RCC���ʱ��                                                            
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);     //ʹ��GPIOB��ʱ��
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);     //ʹ��GPIOC��ʱ��
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);      //ʹ��AFIO��ʱ��
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);  //ʹ��SWD����JTAG
  
  //����T_PA��GPIO 
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;               //����T_PA������
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;          //����T_PA��ģʽ
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;         //����T_PA��I/O������ٶ�
  GPIO_Init(GPIOB, &GPIO_InitStructure);                    //���ݲ�����ʼ��T_PA��GPIO
  
  //����T_PB��GPIO
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;               //����T_PB������
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;          //����T_PB��ģʽ
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;         //����T_PB��I/O������ٶ�
  GPIO_Init(GPIOB, &GPIO_InitStructure);                    //���ݲ�����ʼ��T_PB��GPIO
  
  //����T_SENS1��GPIO
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8;               //����T_SENS1������
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;          //����T_SENS1��ģʽ
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;         //����T_SENS1��I/O������ٶ�
  GPIO_Init(GPIOC, &GPIO_InitStructure);                    //���ݲ�����ʼ��T_SENS1��GPIO
  
  //����T_SENS2��GPIO
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;               //����T_SENS2������
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;          //����T_SENS2��ģʽ
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;         //����T_SENS2��I/O������ٶ�
  GPIO_Init(GPIOC, &GPIO_InitStructure);                    //���ݲ�����ʼ��T_SENS2��GPIO
  
  //����T_SENS_OFF��GPIO
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5;               //����T_SENS_OFF������
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;            //����T_SENS_OFF��ģʽ
  GPIO_Init(GPIOB, &GPIO_InitStructure);                    //���ݲ�����ʼ��T_SENS_OFF��GPIO
  
  //��ʼ����Ϊ�͵�ƽ
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); 
  GPIO_ResetBits(GPIOB, GPIO_Pin_3);    // PB3 is set to 0;
  GPIO_ResetBits(GPIOB, GPIO_Pin_4);    // PB4 is set to 0;
  GPIO_ResetBits(GPIOC, GPIO_Pin_8);    // PC8 is set to 0;
  GPIO_ResetBits(GPIOC, GPIO_Pin_9);    // PC9 is set to 0;

}

/*********************************************************************************************************
* �������ƣ�TempWriteBit
* �������ܣ�����һ���²���ͨ��
* ���������void
* ���������void
* �� �� ֵ��void 
* �������ڣ�2018��01��01��
* ע    �⣺
*********************************************************************************************************/
void TempWriteBit(EnumKEYModel KEYModel)
{
  if(KEYModel == T_PA)                             
  {
    //����T_PAͨ��
    GPIO_WriteBit(GPIOB,GPIO_Pin_3, Bit_SET);
    GPIO_WriteBit(GPIOB,GPIO_Pin_4, Bit_RESET);
    GPIO_WriteBit(GPIOC,GPIO_Pin_8, Bit_RESET);
    GPIO_WriteBit(GPIOC,GPIO_Pin_9, Bit_RESET);
  }
  else if(KEYModel == T_PB)                       
  {
    //����T_PBͨ��
    GPIO_WriteBit(GPIOB,GPIO_Pin_3, Bit_RESET);
    GPIO_WriteBit(GPIOB,GPIO_Pin_4, Bit_SET);
    GPIO_WriteBit(GPIOC,GPIO_Pin_8, Bit_RESET);
    GPIO_WriteBit(GPIOC,GPIO_Pin_9, Bit_RESET);
  }
  else if(KEYModel == T_SENS1)
  {
    //����T_SENS1ͨ��
    GPIO_WriteBit(GPIOB,GPIO_Pin_3, Bit_RESET);
    GPIO_WriteBit(GPIOB,GPIO_Pin_4, Bit_RESET);
    GPIO_WriteBit(GPIOC,GPIO_Pin_8, Bit_SET);
    GPIO_WriteBit(GPIOC,GPIO_Pin_9, Bit_RESET);
  }
  else if(KEYModel == T_SENS2)
  {
    //����T_SENS2ͨ��
    GPIO_WriteBit(GPIOB,GPIO_Pin_3, Bit_RESET);
    GPIO_WriteBit(GPIOB,GPIO_Pin_4, Bit_RESET);
    GPIO_WriteBit(GPIOC,GPIO_Pin_8, Bit_RESET);
    GPIO_WriteBit(GPIOC,GPIO_Pin_9, Bit_SET);
  }
}

/*********************************************************************************************************
* �������ƣ�ReckonToC1AndC2
* �������ܣ���������ģ����C1��C2ϵ��
* ���������void
* ���������void
* �� �� ֵ��void 
* �������ڣ�2018��01��01��
* ע    �⣺
*********************************************************************************************************/
void ReckonToC1AndC2(double PA_Val,double PB_Val,double*C1,double*C2)
{
  const double R1 = 7350.0;  
  const double R2 = 510.0;
  
  *C1 = (R1 * R2 * (PB_Val - PA_Val)) / (R2 * PA_Val - R1 * PB_Val);
  *C2 = ((R1 - R2) * PA_Val * PB_Val) / (R1 * PB_Val - R2 * PA_Val);
}

/*********************************************************************************************************
* �������ƣ�RextReckon
* �������ܣ���������̽ͷ��ֵ����
* ���������void
* ���������void
* �� �� ֵ��void 
* �������ڣ�2018��01��01��
* ע    �⣺
*********************************************************************************************************/
double RextReckon(double SENS_Val,double C1,double C2)
{
  double Rextnum0;      // ����̽ͷ�ĵ���ֵ
  Rextnum0 = (C1 * SENS_Val) / (C2 - SENS_Val);
  return Rextnum0;
}

/*********************************************************************************************************
* �������ƣ�ChangeTempDataPacket
* �������ܣ����������˲����ʼ�����´�����ݰ��е���������
* ���������void
* ���������void
* �� �� ֵ��void 
* �������ڣ�2018��01��01��
* ע    �⣺
*********************************************************************************************************/
void ChangeTempDataPacket(void)
{
  double DoubleTempData = 0.0;
  i16 TempData = 0.0;                 // �Ŵ�10�����͵���λ������������
  i8 i;                               // PCTЭ�����������ݰ��������±�
  double TempMin = 0;                 // �ɼ����������������������Сֵ
  double TempMax = 0;                 // �ɼ�����������������������ֵ
  u8 TempMinArrNum,TempMaxArrNum;     // �ɼ�����������Сֵ�����ֵ��Ӧ�������±�
  u8 arrNum = GetArrNum();                // ��ȡ�����¶�ֵ�������Ԫ�ظ�����Ϊɸѡ��������forѭ���Ĵ���
  u8 arrNum0 = arrNum;                // �����¶�ֵ�������Ԫ�ظ������ݣ�Ϊ����Ԫ�ظ����ĸı���׼��
  if((BitAction)(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5)) == 1)  // ����̽ͷ��������
  {
    s_arrTempData[0] = 3;    // ����̽ͷ״̬
    for(i = 1;i < 5;i++)       // ����ͨ��1��2�������
    {
      s_arrTempData[i] = 0;
    }
  }
  else                                         // ����̽ͷ����
  {
      for(i = 0;i <= arrNum -1;i++)            // ��ѡ��������������鵱�����µ����ֵ����Сֵ
      {
        if(arrTempVariate[i] <= 0.0)           // ����Ч���������޳�
        {
          arrNum0--;                           // �����¶�ֵ�������Ԫ�ظ���ɾ��������     
          continue;
        }
        else                                    // ��ѡ��������������鵱�����µ����ֵ����Сֵ
        {
          DoubleTempData += arrTempVariate[i];  
          if(i == 0)                            // ������ֵ�Լ���������±���г�ʼ��
          {
            TempMin = arrTempVariate[i];
            TempMax = arrTempVariate[i];
            TempMaxArrNum = i;
            TempMinArrNum = i;
          }
          else                                  // �ҳ�������Сֵ�����ֵ�Լ���������±�
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
      
      if((TempMax - TempMin) > 0.5)              // ��������������ֵ����Сֵ����0.5���϶�ʱ�򶪵�������ֵ
      {
        DoubleTempData -= (TempMax + TempMin);
        arrNum0 -= 2;
        arrTempVariate[TempMaxArrNum] = 0;
        arrTempVariate[TempMinArrNum] = 0;
      }
  
      
      if(arrNum != 0)                            // ������õ��������ݳ���10��Ϊ���͵���λ������������
      {
        TempData = (i16)(DoubleTempData / arrNum0  * 10);
      }
      
      if((BitAction)(GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_8)) == 1)  // ͨ��T_SENS1�������������ݰ�����
      {      
        s_arrTempData[0] = 2;                          // ����̽ͷ״̬
        s_arrTempData[1] = (TempData >> 8) & 0xFF;     // ͨ��1���ֽ�����
        s_arrTempData[2] = (TempData) & 0xFF;          // ͨ��1���ֽ�����
        s_arrTempData[3] = 0;                          // ͨ��2���ֽ�����
        s_arrTempData[4] = 0;                          // ͨ��2���ֽ�����
      }
      else if((BitAction)(GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_9)) == 1)  //ͨ��T_SENS2�������������ݰ�����
      {
        s_arrTempData[0] = 1;                          // ����̽ͷ״̬
        s_arrTempData[1] = 0;                          // ͨ��1���ֽ�����
        s_arrTempData[2] = 0;                          // ͨ��1���ֽ�����
        s_arrTempData[3] = (TempData >> 8) & 0xFF;     // ͨ��2���ֽ�����
        s_arrTempData[4] = (TempData) & 0xFF;          // ͨ��2���ֽ�����
      }
      else                                             // ����������Ͻ���
      {
      
      }
  }
    
}
