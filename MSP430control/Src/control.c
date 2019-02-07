/**
  ******************************************************************************
  * @file    control.c 
  * @brief   本源码包括遥控器所有控制函数。通过调用控制函数进行数据的处理
  * @functionList   
  *     1.
  *
  * @currentVersion  V1.1
  * @author  北京中科浩电科技有限公司   www.bj-zkhd.com  
  * @date    2017-11-5
  * @Update  
  * 
  * @historyVersion        @author        @date
  *    
  *  
  ******************************************************************************
  * @attention
  *
  * 本程所提供的源代码均为本程序作者编写，仅供公司内部使用。使用本程序的用户必须明白，
  * 我们不能保证所提供的源码的准确性、安全性和完整性，因此我们将不负责承担任何直接、间 
  * 接因使用这些源码对自己和他人造成任何形式的损失或伤害。任何人在使用本代码时，请注明
  * 作者和出处。
  * <h2><center>    版权所有(C) 2017 北京中科浩电科技有限公司    </center></h2>
  ******************************************************************************
  */  

#include "control.h"
#include "usart.h"
#include "adc.h"
#include "sys.h"
#include "key.h"
#include "led.h"
#include "beep.h"
#include "ctrl_Flash.h"
#include "flash_gzj.h"
#include "nrf24l01.h"
#include "string.h"
#include "gcs.h"
#include "communication.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"

#define RC_DEAD_ZONE_RANGE		(100)
#define RC_DEAD_ZONE_RANGE_HALF (RC_DEAD_ZONE_RANGE / 2)

extern NRF_Mannager_t NRF_Mannager;
extern uint16_t Batt_Flag = 0;
extern send_Data_u RC_sendData = {0};
extern uint16_t adcBuf[7] = {0};

RC_Para_t RC_Para = {0};
RC_Raw_t RC_Raw = {0};

YesNo_t Failure = No;
YesNo_t Success = Yes;

// 遥控数据校验
extern uint8_t RC_Parc_CRC(RC_Para_t* pPara)
{
    uint8_t i = 0;
    uint8_t crc = 0;
    uint8_t* pParaBuf = (void*)0;
    
    pParaBuf = (uint8_t*)pPara;
    
    crc = pParaBuf[0];
    
    for(i = 1; i < sizeof(RC_Para_t) - 2; i++)
	{
        crc ^= pParaBuf[i];
    }
    return crc;
}

// 获取ADC的值
void get_ADC_value()
{
    /*****ADC 通道对应功能*******
    *     ADC1_CH1    Yaw       *
    *     ADC1_CH2    Throttle  *
    *     ADC1_CH8    Roll      *
    *     ADC1_CH9    Pitch     *
    *     ADC1_CH3    Left      *
    *     ADC1_CH4    Right     *
    *     ADC1_CH7    Batt      *
    *****************************/
    
    static uint16_t i, j;
	
	for(i = Yaw; i < Remote_CH_Num; i++)
	{
		adcBuf[i]  = 0;
	}

	for (j = 0; j < 10; j++)
	{
		for(i = Yaw; i < Remote_CH_Num; i++)
		{
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, 0xffff);
			adcBuf[i] += HAL_ADC_GetValue(&hadc1);
		}
	}
	
	HAL_ADC_Stop(&hadc1);
	
	for(i = Yaw; i < Remote_CH_Num; i++)
	{
		adcBuf[i] = adcBuf[i] /10;
	}
//	
//    printf("Yaw = %5d    ", adcBuf[Yaw]);    
//    printf("Throttle = %5d    ", adcBuf[Throttle]);
//    printf("Roll  = %5d    ", adcBuf[Roll]);
//    printf("Pitch = %5d    ", adcBuf[Pitch]);
//    printf("Left = %5d     ", adcBuf[Left]);
//    printf("Right = %5d    Batt = %5d\r\n", adcBuf[Right], adcBuf[Batt]);
}

/*函数名：KeyRCCalibration(void)
  功能：实现遥控手动校准。
*/
void KeyRCCalibration(void)
{
	static int i = 0;
	static int j = 0;
	YesNo_t Waiting_Calibration = No;
	
	typedef struct
	{        
        uint16 cache_min[ADCRecordTime];
        uint16 cache_max[ADCRecordTime];
        
		uint16_t min_sum;
        uint16_t mid_sum;
        uint16_t max_sum;
        
        uint16_t min;
        uint16_t mid;
        uint16_t max;
	}RC_Cal_Ch_Cache_t;
	
	RC_Cal_Ch_Cache_t rol = {0};
	RC_Cal_Ch_Cache_t pit = {0};
	RC_Cal_Ch_Cache_t thr = {0};
	RC_Cal_Ch_Cache_t yaw = {0};
	
	if (Failure == Read_RC_Offset_From_Flash(&RC_Para))
	{
		Waiting_Calibration = Yes;
	}else if (Yes == is_sw2_pressed())
	{
		Waiting_Calibration = Yes;
	}
	
	if(Yes == Waiting_Calibration)
	{
		BEEP_ON;
		LED_STU_OFF;
		LED_RF_OFF;
		LED_POWER_OFF;
	
		while(Yes == is_sw2_pressed())
		{
			HAL_Delay(50);
		}
		BEEP_OFF;		
		HAL_Delay(100);
				
		rol.mid_sum = 0;
		pit.mid_sum = 0;
		thr.mid_sum = 0;
		yaw.mid_sum = 0;
		
		// 采集ADC各通道中值10次,累加中值。
		for (i = 0; i < SamplingFrequency; i++)           
		{
			get_ADC_value();

			rol.mid_sum += adcBuf[Roll];
			pit.mid_sum += adcBuf[Pitch];
			thr.mid_sum += adcBuf[Throttle];
			yaw.mid_sum += adcBuf[Yaw]; 
			
			HAL_Delay(5);
		}
		
		 // 得到中点的结果	
		rol.mid = rol.mid_sum / ADCRecordTime;          
		pit.mid = pit.mid_sum / ADCRecordTime;
		thr.mid = thr.mid_sum / ADCRecordTime;
		yaw.mid = yaw.mid_sum / ADCRecordTime;	
				
		// 目前只得到中点的值
		for (i = 0; i <ADCRecordTime; i++)
		{
			rol.cache_max[i] = rol.mid;
			pit.cache_max[i] = pit.mid;
			thr.cache_max[i] = thr.mid;
			yaw.cache_max[i] = yaw.mid;
			
			rol.cache_min[i] = rol.mid;
			pit.cache_min[i] = pit.mid;
			thr.cache_min[i] = thr.mid;
			yaw.cache_min[i] = yaw.mid;
		}

		// 看到灯亮 开始拨动摇杆到最大范围
		LED_RF_ON;
		LED_POWER_ON;
		
		while(!is_sw2_pressed())
		{
			get_ADC_value();
			led_STU_state = 1;
			// 横滚 处理
			for(j = 0; j < ADCRecordTime; j++)
			{
				if(adcBuf[Roll] > rol.cache_max[j])
				{
					rol.cache_max[j] = adcBuf[Roll];
					break;
				}
			}            
			for(j = 0; j < ADCRecordTime; j++)
			{
				if(adcBuf[Roll] < rol.cache_min[j])
				{
					rol.cache_min[j] = adcBuf[Roll];
					break;
				}
			}
			
			// 俯仰 处理
			for(j = 0; j < ADCRecordTime; j++)
			{
				if(adcBuf[Pitch] > pit.cache_max[j])
				{
					pit.cache_max[j] = adcBuf[Pitch];
					break;
				}
			}            
			for(j = 0; j < ADCRecordTime; j++)
			{
				if(adcBuf[Pitch] < pit.cache_min[j])
				{
					pit.cache_min[j] = adcBuf[Pitch];
					break;
				}
			}
			
			// 油门 处理
			for(j = 0; j < ADCRecordTime; j++)
			{
				if(adcBuf[Throttle] > thr.cache_max[j])
				{
					thr.cache_max[j] = adcBuf[Throttle];
					break;
				}
			}            
			for(j = 0; j < ADCRecordTime; j++)
			{
				if(adcBuf[Throttle] < thr.cache_min[j])
				{
					thr.cache_min[j] = adcBuf[Throttle];
					break;
				}
			}
			
			// 航向 处理
			for(j = 0; j < ADCRecordTime; j++)
			{
				if(adcBuf[Yaw] > yaw.cache_max[j])
				{
					yaw.cache_max[j] = adcBuf[Yaw];
					break;
				}
			}            
			for(j = 0; j < ADCRecordTime; j++)
			{
				if(adcBuf[Yaw] < yaw.cache_min[j])
				{
					yaw.cache_min[j] = adcBuf[Yaw];
					break;
				}
			}
			
			HAL_Delay(5);		
		}
		
		// MAX MIN采集完毕
		BEEP_ON;
		LED_RF_OFF;
		LED_POWER_OFF;
		LED_STU_OFF;
		BEEP_OFF;
		
		// 累加MAX MIN值
		 rol.min_sum = 0;
		 pit.min_sum = 0;
		 thr.min_sum = 0;
		 yaw.min_sum = 0;
		 
		 rol.max_sum = 0;
		 pit.max_sum = 0;
		 thr.max_sum = 0;
		 yaw.max_sum = 0;
		for (i = 0; i < ADCRecordTime; i++)
		{
			rol.min_sum += rol.cache_min[i];
			pit.min_sum += pit.cache_min[i];
			thr.min_sum += thr.cache_min[i];
			yaw.min_sum += yaw.cache_min[i];
						   
			rol.max_sum += rol.cache_max[i];
			pit.max_sum += pit.cache_max[i];
			thr.max_sum += thr.cache_max[i];
		    yaw.max_sum += yaw.cache_max[i];
		}
		                   
		// MAX Min求平均
		rol.min = rol.min_sum / ADCRecordTime;
		pit.min = pit.min_sum / ADCRecordTime;
		thr.min = thr.min_sum / ADCRecordTime;
		yaw.min = yaw.min_sum / ADCRecordTime;
				  		          
		rol.max = rol.max_sum / ADCRecordTime;
		pit.max = pit.max_sum / ADCRecordTime;
		thr.max = thr.max_sum / ADCRecordTime;
		yaw.max = yaw.max_sum / ADCRecordTime;
		   
        RC_Para.rol.Min = rol.min;
        RC_Para.rol.Mid = rol.mid;
        RC_Para.rol.Max = rol.max;
            
        RC_Para.pit.Min = pit.min;
        RC_Para.pit.Mid = pit.mid;
        RC_Para.pit.Max = pit.max;
               
        RC_Para.thr.Min = thr.min;
        RC_Para.thr.Mid = thr.mid;
        RC_Para.thr.Max = thr.max;
          
        RC_Para.yaw.Min = yaw.min;
        RC_Para.yaw.Mid = yaw.mid;
        RC_Para.yaw.Max = yaw.max;

		RC_Para.crc = RC_Parc_CRC(&RC_Para);
		
		// 保存数据到Flash区
		Save_RC_Offset_To_Flash(&RC_Para);		
	}
}


// get linear eqution（根据已知点确定）
static uint16_t Get_Linear_Equation(uint16_t x, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
	uint16_t y = 0;
	
	 /*
        (y - y0) / (x - x0) = (y1 - y0) / (x1 - x0)
        
        y = ((x - x0) * (y1 - y0)) / (x1 - x0) + y0
      */
	
	y = (((uint32)x - (uint32)x0) * ((uint32)y1 - (uint32)y0)) / ((uint32)x1 - (uint32)x0) + (uint32)y0;
	return y;
}

// Raw to Target (0~4096 to 1000~2000)
static uint16_t Joystick_Raw_To_Traget(uint16_t Raw, const RecordADCExtremumVal_t* pRaw, const JoystickCal_Target_t* pTarget)
{
	uint16_t target = 0;
	
	if (Raw < (pRaw->Min + RC_DEAD_ZONE_RANGE))
	{
		Raw = pRaw->Min + RC_DEAD_ZONE_RANGE;
	}else if (Raw > (pRaw->Max - RC_DEAD_ZONE_RANGE))
	{
		Raw = pRaw->Max - RC_DEAD_ZONE_RANGE;
	}
	
	if (Raw <= pRaw->Mid)
	{
		//已知点(Raw，1000)  (Raw,1500)
		target = Get_Linear_Equation(  Raw, (pRaw->Min + RC_DEAD_ZONE_RANGE), 1000, 
											 pRaw->Mid, 1500  );
	}else
	{
		//已知点(Raw,1500)   (Raw,2000)
		target = Get_Linear_Equation(  Raw, pRaw->Mid, 1500,
											(pRaw->Max - RC_DEAD_ZONE_RANGE), 2000  );
	}
	
	if( (target >= (pTarget->Mid - RC_DEAD_ZONE_RANGE_HALF)) 
		&& (target <= (pTarget->Mid + RC_DEAD_ZONE_RANGE_HALF)) )
    {
		target = pTarget->Mid;
    }
    return target;
}

/*
	函数名：LoadRCdata();
	功能：转换ADC的值并通过串口1发送给蓝牙。
*/
void LoadRCdata(void)
{
    static uint16_t CH_Yaw      = 0;
	static uint16_t CH_Throttle = 0;
	static uint16_t CH_Roll     = 0;
    static uint16_t CH_Pitch    = 0;
	static uint16_t CH_Left     = 0;
	static uint16_t CH_Right    = 0;
    
    uint8_t GCS_SendBuff[40];
	
	//滤波后的ADC值
	get_ADC_value();
	RC_Raw.Filter_Rol   = adcBuf[Roll];
	RC_Raw.Filter_Pit   = adcBuf[Pitch];
	RC_Raw.Filter_Thr   = adcBuf[Throttle];
	RC_Raw.Filter_Yaw   = adcBuf[Yaw];
	RC_Raw.Filter_Left  = adcBuf[Left];
	RC_Raw.Filter_Right = adcBuf[Right];
	RC_Raw.Filter_Batt  = adcBuf[Batt];
	
	//转换ADC值为1000~2000
	CH_Roll     = 3000 - ( Joystick_Raw_To_Traget(RC_Raw.Filter_Rol, &RC_Para.rol, &JoystickCal_Target) );
	CH_Pitch    = Joystick_Raw_To_Traget(RC_Raw.Filter_Pit, &RC_Para.pit, &JoystickCal_Target);
	CH_Throttle = Joystick_Raw_To_Traget(RC_Raw.Filter_Thr, &RC_Para.thr, &JoystickCal_Target);
	CH_Yaw      = 3000 - ( Joystick_Raw_To_Traget(RC_Raw.Filter_Yaw, &RC_Para.yaw, &JoystickCal_Target) );
	
	CH_Left = 1000 + (1000 * RC_Raw.Filter_Left) / 4096;
    CH_Left = (CH_Left <= 1000) ? 1000 : CH_Left;
    CH_Left = (CH_Left >= 2000) ? 2000 : CH_Left;
		
	CH_Right = 1000 + (1000 * RC_Raw.Filter_Right) / 4096;
    CH_Right = (CH_Right <= 1000) ? 1000 : CH_Right;
    CH_Right = (CH_Right >= 2000) ? 2000 : CH_Right;
	
	Batt_Flag = (RC_Raw.Filter_Batt <= 2000) ? 1 : 0;                              //低电压判断
	
	RC_sendData.sendData.YAW      = CH_Yaw;
	RC_sendData.sendData.THROTTLE = CH_Throttle;
	RC_sendData.sendData.ROLL     = CH_Roll;
	RC_sendData.sendData.PITCH    = CH_Pitch;
	RC_sendData.sendData.LEFT     = CH_Left;
	RC_sendData.sendData.RIGHT    = CH_Right;
    
	RC_sendData.sendData.Start = 0xAAAA;
    
    if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(UNLOCK_GPIO_Port,UNLOCK_Pin))
	{
        RC_sendData.sendData.ContrlBit = 0x0001;
    }else
    {
        RC_sendData.sendData.ContrlBit = 0x0000;
    }
    
	RC_sendData.sendData.Stop  = 0x5555;
	
    GCS_SendBuff[0] = 0xAA;
    GCS_SendBuff[1] = 26;
    GCS_SendBuff[2] = 0x00;
    GCS_SendBuff[3] = 0x2;
    GCS_SendBuff[4] = 0x0;
    GCS_SendBuff[5] = 0x0;
    memcpy(GCS_SendBuff+6,RC_sendData.RC_Sendbuff,sizeof(RC_sendData.RC_Sendbuff));
    
    led_RF_state = 1;
    enQueue(&NRF_Mannager.qTx,RC_sendData.RC_Sendbuff,32);
    enQueue(&USB_Send_Queue,GCS_SendBuff,sizeof(RC_sendData.RC_Sendbuff)+6);
}

/*
	函数名：SW_control();
	功能：获取开关状态。
*/
void SW_control()
{
    uint8_t sw = 0;
	uint8_t Arm_pos = 0;
	uint8_t CaliFlag_pos = 0;
	static uint16_t  SW_two = 0x03e8;
	static uint16_t SW_three = 0x05dc;
    
    sw = SW_Scan();
	
    if(sw)
    {
        switch(sw)
        {
            case SW1_PRES:
			    if (Arm_up == 1)
				{
					Arm_pos = 0;
				}else
				{
					Arm_pos = 1;
					
					beep_state = 1;
				}
            break;
            case SW2_PRES:
                
            break;
            case SW3_PRES:
			    if (sw1_up == 1)
				{
					SW_two = 1000;
				}else
				{
					SW_two = 2000;
				}
            break;
            case SW4_PRES:
				if (sw2_up == 1)
				{
					SW_three = 1500;
				}else
				{
					SW_three = 2000;
				}
            break;
            case SW5_PRES:
				if (CaliFlag_up == 1)
				{
					SW_three = 1500;
				}else
				{
					SW_three = 1000;
				}
            break;
        }
    }
	
	RC_sendData.sendData.Arm_State = Arm_pos;
	RC_sendData.sendData.CaliFlag = CaliFlag_pos;
	RC_sendData.sendData.SW_TWO = SW_two;
	RC_sendData.sendData.SW_THREE = SW_three;
}

/******************* (C) 版权所有 2017 北京中科浩电科技有限公司 *******************/
