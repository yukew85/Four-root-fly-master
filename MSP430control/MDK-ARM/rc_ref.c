#include "rc.h"
#include "gpio.h"
#include "timer.h"
#include "adc.h"
#include "data_filter.h"
#include "usart.h"
#include "board.h"
#include "flash.h"

// RC_Switch2_Left:                             P3.5
#define RC_SWITCH2_LEFT_PORT                    GPIO_PORT_P3
#define RC_SWITCH2_LEFT_PIN                     GPIO_PIN5
#define RC_SWITCH2_LEFT_STATUS_UP               GPIO_INPUT_PIN_HIGH

// RC_Switch2_Right:                            P3.6
#define RC_SWITCH2_RIGHT_PORT                   GPIO_PORT_P3
#define RC_SWITCH2_RIGHT_PIN                    GPIO_PIN6
#define RC_SWITCH2_RIGHT_STATUS_UP              GPIO_INPUT_PIN_HIGH

// RC_Key_Joystick_Left:                        P3.7
#define RC_KEY_JOYSTICK_LEFT_PORT               GPIO_PORT_P3
#define RC_KEY_JOYSTICK_LEFT_PIN                GPIO_PIN7
#define RC_KEY_JOYSTICK_LEFT_STATUS_PRESSED     GPIO_INPUT_PIN_LOW

// RC_Key_Joystick_Right:                       P8.2
#define RC_KEY_JOYSTICK_RIGHT_PORT              GPIO_PORT_P8
#define RC_KEY_JOYSTICK_RIGHT_PIN               GPIO_PIN2
#define RC_KEY_JOYSTICK_RIGHT_STATUS_PRESSED    GPIO_INPUT_PIN_LOW

// RC_LED_Bat_Up:                               P2.3
#define RC_LED_BAT_ALARM_UP_PORT                GPIO_PORT_P2
#define RC_LED_BAT_ALARM_UP_PIN                 GPIO_PIN3
#define RC_LED_BAT_ALARM_UP_LEVEL_ON            0

// RC_LED_Bat_Down:                             P2.6
#define RC_LED_BAT_ALARM_DOWN_PORT              GPIO_PORT_P2
#define RC_LED_BAT_ALARM_DOWN_PIN               GPIO_PIN6
#define RC_LED_BAT_ALARM_DOWN_LEVEL_ON          0

// RC_Beep:                                     P8.1
#define RC_BEEP_PORT                            GPIO_PORT_P8
#define RC_BEEP_PIN                             GPIO_PIN1
#define RC_BEEP_LEVEL_ON                        0

#define JOYSTICK_R_X_CH                         ADC_Ch00
#define JOYSTICK_R_Y_CH                         ADC_Ch01

#define JOYSTICK_L_X_CH                         ADC_Ch03
#define JOYSTICK_L_Y_CH                         ADC_Ch02

#define BAT_ALARM_UP_CH                         ADC_Ch04
#define BAT_ALARM_DN_CH                         ADC_Ch05

#define RANDOM_CH                               ADC_Ch06

#define RC_RAW_NUM                              4

#define BATTERY_ALARM_VAL                       (3.600) // 锂电池报警电压

#define JOYSTICK_DEADZONE_RAW_END               (100)   // 摇杆原始值边界敏感范围

#define JOYSTICK_DEADZONE_TARGET_MID            (100)   // 摇杆目标值中点敏感范围
#define JOYSTICK_DEADZONE_TARGET_MID_HALF       (JOYSTICK_DEADZONE_TARGET_MID / 2)

// 遥控器未经处理的 ADC 原始值
typedef struct
{
    // 采集的原始值
    uint16 joystick_left_x[RC_RAW_NUM];         // 左侧摇杆 横向
    uint16 joystick_left_y[RC_RAW_NUM];         // 左侧摇杆 纵向
    
    uint16 joystick_right_x[RC_RAW_NUM];        // 右侧摇杆 横向
    uint16 joystick_right_y[RC_RAW_NUM];        // 右侧摇杆 纵向
    
    uint16 v_bat_all[RC_RAW_NUM];               // 2S 锂电池 ADC (被电阻分压到了真实值的 1/4)
    uint16 v_bat_down[RC_RAW_NUM];              // 下面 1S 锂电池 ADC (被电阻分压到了真实值的 1/2)
    
    uint16 random;
    
    // 滤波后的原始值
    uint16 filter_joystick_left_x;
    uint16 filter_joystick_left_y;
    
    uint16 filter_joystick_right_x;
    uint16 filter_joystick_right_y;
    
    Switch2_Status_t switch2_left;
    Switch2_Status_t switch2_right;
    
    KeyStatus_t joystick_key_left;
    KeyStatus_t joystick_key_right;
    
    uint16 filter_v_bat_all;
    uint16 filter_v_bat_down;
}RC_Raw_t;

typedef struct
{
    uint16 min;
    uint16 mid;
    uint16 max;
}JoystickCal_Raw_Para_t;

typedef struct
{
    uint16 min;
    uint16 mid;
    uint16 max;
}JoystickCal_Target_t;

const static uint8 RC_Para_Head[] = {"MSP430RC_Param"};

typedef struct
{
    uint8 head[sizeof(RC_Para_Head)];
    
    JoystickCal_Raw_Para_t rol;
    JoystickCal_Raw_Para_t pit;
    JoystickCal_Raw_Para_t thr;
    JoystickCal_Raw_Para_t yaw;
    
    uint16 crc;                  // 前面内容逐字节按位异或结果
}RC_Para_t;

static RC_Raw_t RC_Raw = {0};
static uint16 RC_ADC_Raw_Cnt = 0;

static KeyStatus_t JoystickKey_PowerOn_Status_Left = KeyPressed;
static KeyStatus_t JoystickKey_PowerOn_Status_Right = KeyPressed;

static uint8 RC_Msg[18] = {0xA0, 0x01, 0x10, (0xA0 ^ 0x01 ^ 0x10)};

static RC_Para_t RC_Para;

static const JoystickCal_Target_t JoystickCal_Target = 
{
    .min = 1000,
    .mid = 1500,
    .max = 2000
};

extern RC_t RC = {0};

static uint8 RC_Parc_CRC(RC_Para_t* pPara)
{
    uint8 i = 0;
    uint8 crc = 0;
    uint8* pParaBuf = (void*)0;
    
    pParaBuf = (uint8*)pPara;
    
    crc = pParaBuf[0];
    
    for(i=1; i<sizeof(RC_Para_t)-2; i++)
        crc ^= pParaBuf[i];
    
    return crc;
}

static uint16 Joystick_Raw_2_Target(uint16 Raw, const JoystickCal_Raw_Para_t* pRaw, const JoystickCal_Target_t* pTarget)
{
    uint16 target = 0;
    
    if(Raw < (pRaw->min + JOYSTICK_DEADZONE_RAW_END))
      Raw = pRaw->min + JOYSTICK_DEADZONE_RAW_END;
    else if(Raw > (pRaw->max - JOYSTICK_DEADZONE_RAW_END))
      Raw = pRaw->max - JOYSTICK_DEADZONE_RAW_END;
    
    if(Raw <= pRaw->mid)
    {
        // (raw_min, 1000), (raw_mid, 1500)
        target = Get_Linear_Function_y_uint16(  Raw,
                                                (pRaw->min + JOYSTICK_DEADZONE_RAW_END), 1000,
                                                pRaw->mid, 1500);
    }
    else
    {
        // (pRaw->mid, 1500), (raw_max, 2000)
        target = Get_Linear_Function_y_uint16(  Raw,
                                                pRaw->mid, 1500,
                                                (pRaw->max - JOYSTICK_DEADZONE_RAW_END), 2000);
    }
    
    if(     (target >= (pTarget->mid - JOYSTICK_DEADZONE_TARGET_MID_HALF))
        &&  (target <= (pTarget->mid + JOYSTICK_DEADZONE_TARGET_MID_HALF))  )
    {
      target = pTarget->mid;
    }
    
    return target;
}

extern void RC_Switch2_Left_Init(void)
{
    GPIO_setAsInputPin(RC_SWITCH2_LEFT_PORT, RC_SWITCH2_LEFT_PIN);
    //GPIO_enableInterrupt(RC_SWITCH2_LEFT_PORT, RC_SWITCH2_LEFT_PIN);
}

extern Switch2_Status_t RC_Switch2_Left_Status(void)
{
    if(RC_SWITCH2_LEFT_STATUS_UP == GPIO_getInputPinValue(RC_SWITCH2_LEFT_PORT, RC_SWITCH2_LEFT_PIN))
        return Switch2_Up;
    else
        return Swtich2_Down;
}

extern void RC_Switch2_Right_Init(void)
{
    GPIO_setAsInputPin(RC_SWITCH2_RIGHT_PORT, RC_SWITCH2_RIGHT_PIN);
    //GPIO_enableInterrupt(RC_SWITCH2_RIGHT_PORT, RC_SWITCH2_RIGHT_PIN);
}

extern Switch2_Status_t RC_Switch2_Right_Status(void)
{
    if(RC_SWITCH2_RIGHT_STATUS_UP == GPIO_getInputPinValue(RC_SWITCH2_RIGHT_PORT, RC_SWITCH2_RIGHT_PIN))
        return Switch2_Up;
    else
        return Swtich2_Down;
}

extern void RC_Switch2_All_Init(void)
{
    RC_Switch2_Left_Init();
    RC_Switch2_Right_Init();
}

extern void RC_Key_Joystick_Left_Init(void)
{
    if(RC_KEY_JOYSTICK_LEFT_STATUS_PRESSED)
    {
        GPIO_setAsInputPinWithPullDownResistor(RC_KEY_JOYSTICK_LEFT_PORT, RC_KEY_JOYSTICK_LEFT_PIN);
        //GPIO_selectInterruptEdge(RC_KEY_JOYSTICK_LEFT_PORT, RC_KEY_JOYSTICK_LEFT_PIN, GPIO_LOW_TO_HIGH_TRANSITION);
    }
    else
    {
        GPIO_setAsInputPinWithPullUpResistor(RC_KEY_JOYSTICK_LEFT_PORT, RC_KEY_JOYSTICK_LEFT_PIN);
        //GPIO_selectInterruptEdge(RC_KEY_JOYSTICK_LEFT_PORT, RC_KEY_JOYSTICK_LEFT_PIN, GPIO_HIGH_TO_LOW_TRANSITION);
    }
    
    //GPIO_enableInterrupt(RC_KEY_JOYSTICK_LEFT_PORT, RC_KEY_JOYSTICK_LEFT_PIN);
    
    JoystickKey_PowerOn_Status_Left = RC_Key_Joystick_Left_Status();
}

extern KeyStatus_t RC_Key_Joystick_Left_Status(void)
{
    if(RC_KEY_JOYSTICK_LEFT_STATUS_PRESSED == GPIO_getInputPinValue(RC_KEY_JOYSTICK_LEFT_PORT, RC_KEY_JOYSTICK_LEFT_PIN))
        return KeyPressed;
    else
        return KeyRelease;
}

extern KeyStatus_t RC_Key_Joystick_Left_PowerOn_Status(void)
{
    return JoystickKey_PowerOn_Status_Left;
}

extern void RC_Key_Joystick_Right_Init(void)
{
    if(RC_KEY_JOYSTICK_RIGHT_STATUS_PRESSED)
    {
        GPIO_setAsInputPinWithPullDownResistor(RC_KEY_JOYSTICK_RIGHT_PORT, RC_KEY_JOYSTICK_RIGHT_PIN);
        //GPIO_selectInterruptEdge(RC_KEY_JOYSTICK_RIGHT_PORT, RC_KEY_JOYSTICK_RIGHT_PIN, GPIO_LOW_TO_HIGH_TRANSITION);
    }
    else
    {
        GPIO_setAsInputPinWithPullUpResistor(RC_KEY_JOYSTICK_RIGHT_PORT, RC_KEY_JOYSTICK_RIGHT_PIN);
        //GPIO_selectInterruptEdge(RC_KEY_JOYSTICK_RIGHT_PORT, RC_KEY_JOYSTICK_RIGHT_PIN, GPIO_HIGH_TO_LOW_TRANSITION);
    }
    
    //GPIO_enableInterrupt(RC_KEY_JOYSTICK_RIGHT_PORT, RC_KEY_JOYSTICK_RIGHT_PIN);
    
    JoystickKey_PowerOn_Status_Right = RC_Key_Joystick_Right_Status();
}

extern KeyStatus_t RC_Key_Joystick_Right_Status(void)
{
    if(RC_KEY_JOYSTICK_RIGHT_STATUS_PRESSED == GPIO_getInputPinValue(RC_KEY_JOYSTICK_RIGHT_PORT, RC_KEY_JOYSTICK_RIGHT_PIN))
        return KeyPressed;
    else
        return KeyRelease;
}

extern KeyStatus_t RC_Key_Joystick_Right_PowerOn_Status(void)
{
    return JoystickKey_PowerOn_Status_Right;
}

extern void RC_Key_All_Init(void)
{
    RC_Key_Joystick_Left_Init();
    RC_Key_Joystick_Right_Init();
}

extern void RC_LED_Bat_Alarm_Up_Init(void)
{
    GPIO_setAsOutputPin(RC_LED_BAT_ALARM_UP_PORT, RC_LED_BAT_ALARM_UP_PIN);
    RC_LED_Bat_Alarm_Up_Off();
}

extern void RC_LED_Bat_Alarm_Up_On(void)
{
    if(RC_LED_BAT_ALARM_UP_LEVEL_ON)
        GPIO_setOutputHighOnPin(RC_LED_BAT_ALARM_UP_PORT, RC_LED_BAT_ALARM_UP_PIN);
    else
        GPIO_setOutputLowOnPin(RC_LED_BAT_ALARM_UP_PORT, RC_LED_BAT_ALARM_UP_PIN);
}

extern void RC_LED_Bat_Alarm_Up_Off(void)
{
    if(RC_LED_BAT_ALARM_UP_LEVEL_ON)
        GPIO_setOutputLowOnPin(RC_LED_BAT_ALARM_UP_PORT, RC_LED_BAT_ALARM_UP_PIN);
    else
        GPIO_setOutputHighOnPin(RC_LED_BAT_ALARM_UP_PORT, RC_LED_BAT_ALARM_UP_PIN);
}

extern void RC_LED_Bat_Alarm_Up_Toogle(void)
{
    GPIO_toggleOutputOnPin(RC_LED_BAT_ALARM_UP_PORT, RC_LED_BAT_ALARM_UP_PIN);
}

extern void RC_LED_Bat_Alarm_Up_Set(LEDStatus_t status)
{
    if(RC_LED_BAT_ALARM_UP_LEVEL_ON == status)
        RC_LED_Bat_Alarm_Up_On();
    else
        RC_LED_Bat_Alarm_Up_Off();
}

extern void RC_LED_Bat_Alarm_Down_Init(void)
{
    GPIO_setAsOutputPin(RC_LED_BAT_ALARM_DOWN_PORT, RC_LED_BAT_ALARM_DOWN_PIN);
    RC_LED_Bat_Alarm_Down_Off();
}

extern void RC_LED_Bat_Alarm_Down_On(void)
{
    if(RC_LED_BAT_ALARM_DOWN_LEVEL_ON)
        GPIO_setOutputHighOnPin(RC_LED_BAT_ALARM_DOWN_PORT, RC_LED_BAT_ALARM_DOWN_PIN);
    else
        GPIO_setOutputLowOnPin(RC_LED_BAT_ALARM_DOWN_PORT, RC_LED_BAT_ALARM_DOWN_PIN);
}

extern void RC_LED_Bat_Alarm_Down_Off(void)
{
    if(RC_LED_BAT_ALARM_DOWN_LEVEL_ON)
        GPIO_setOutputLowOnPin(RC_LED_BAT_ALARM_DOWN_PORT, RC_LED_BAT_ALARM_DOWN_PIN);
    else
        GPIO_setOutputHighOnPin(RC_LED_BAT_ALARM_DOWN_PORT, RC_LED_BAT_ALARM_DOWN_PIN);
}

extern void RC_LED_Bat_Alarm_Down_Toogle(void)
{
    GPIO_toggleOutputOnPin(RC_LED_BAT_ALARM_DOWN_PORT, RC_LED_BAT_ALARM_DOWN_PIN);
}

extern void RC_LED_Bat_Alarm_Down_Set(LEDStatus_t status)
{
    if(RC_LED_BAT_ALARM_DOWN_LEVEL_ON == status)
        RC_LED_Bat_Alarm_Down_On();
    else
        RC_LED_Bat_Alarm_Down_Off();
}

extern void RC_LED_All_Init(void)
{
    RC_LED_Bat_Alarm_Up_Init();
    RC_LED_Bat_Alarm_Down_Init();
}

extern void RC_LED_All_On(void)
{
    RC_LED_Bat_Alarm_Up_On();
    RC_LED_Bat_Alarm_Down_On();
}

extern void RC_LED_All_Off(void)
{
    RC_LED_Bat_Alarm_Up_Off();
    RC_LED_Bat_Alarm_Down_Off();
}

extern void RC_LED_All_Toogle(void)
{
    RC_LED_Bat_Alarm_Up_Toogle();
    RC_LED_Bat_Alarm_Down_Toogle();
}

extern void RC_LED_All_Set(LEDStatus_t status)
{
    RC_LED_Bat_Alarm_Up_Set(status);
    RC_LED_Bat_Alarm_Down_Set(status);
}

extern void RC_Beep_Init(void)
{
    GPIO_setAsOutputPin(RC_BEEP_PORT, RC_BEEP_PIN);
    RC_Beep_Off();
}

extern void RC_Beep_On(void)
{
    if(RC_BEEP_LEVEL_ON)
        GPIO_setOutputHighOnPin(RC_BEEP_PORT, RC_BEEP_PIN);
    else
        GPIO_setOutputLowOnPin(RC_BEEP_PORT, RC_BEEP_PIN);
}

extern void RC_Beep_Off(void)
{
    if(RC_BEEP_LEVEL_ON)
        GPIO_setOutputLowOnPin(RC_BEEP_PORT, RC_BEEP_PIN);
    else
        GPIO_setOutputHighOnPin(RC_BEEP_PORT, RC_BEEP_PIN);
}

extern void RC_Beep_Toogle(void)
{
    GPIO_toggleOutputOnPin(RC_BEEP_PORT, RC_BEEP_PIN);
}

extern void RC_Beep_Set(LEDStatus_t status)
{
    if(RC_BEEP_LEVEL_ON == status)
        RC_Beep_On();
    else
        RC_Beep_Off();
}

extern void RC_Msg_Update(void)
{
    uint8 i;
    
    RC_Msg[0] = 0xA0;
    RC_Msg[1] = 0x01;
    RC_Msg[2] = 0x10;
    RC_Msg[3] = RC_Msg[0] ^ RC_Msg[1] ^ RC_Msg[2];
    
    // 左摇杆 上下 先低后高
    RC_Msg[4] = UINT16_BYTE_L(RC.joystick_left.y);
    RC_Msg[5] = UINT16_BYTE_H(RC.joystick_left.y);
    
    // 左摇杆 水平 先低后高
    RC_Msg[6] = UINT16_BYTE_L(RC.joystick_left.x);
    RC_Msg[7] = UINT16_BYTE_H(RC.joystick_left.x);
    
    // 右摇杆 上下 先低后高
    RC_Msg[8] = UINT16_BYTE_L(RC.joystick_right.y);
    RC_Msg[9] = UINT16_BYTE_H(RC.joystick_right.y);
    
    // 右摇杆 水平 先低后高
    RC_Msg[10] = UINT16_BYTE_L(RC.joystick_right.x);
    RC_Msg[11] = UINT16_BYTE_H(RC.joystick_right.x);
    
    // 左开关 先低后高
    RC_Msg[12] = UINT16_BYTE_L(RC.switch2_left);
    RC_Msg[13] = UINT16_BYTE_H(RC.switch2_left);
    
    // 右开关 先低后高
    RC_Msg[14] = UINT16_BYTE_L(RC.switch2_right);
    RC_Msg[15] = UINT16_BYTE_H(RC.switch2_right);
    
    // 随机数(使用 ADC 第 6 通道噪声产生)
    RC_Msg[16] = (RC_Msg[16] << 2) | (RC.random & 0x03);
    
    // 所有数据校验和 低字节
    RC_Msg[17]=RC_Msg[0];
    for(i=1; i<17; i++)
    {
        RC_Msg[17] ^= RC_Msg[i];
    }
}

extern void RC_Init(void)
{
    uint16 i;
    
    for(i=0; i<sizeof(RC_Para_Head); i++)
    {
        RC_Para.head[i] = RC_Para_Head[i];
    }
  
    RC_Switch2_All_Init();
    
    RC_Key_All_Init();
    
    RC_LED_All_Init();
    
    RC_Beep_Init();
}

extern void RC_Update(void)
{
    static uint64 Last_ms = 0;
    
    if((ms_Since_PowerOn() - Last_ms) >= 5)
    {
        Last_ms = ms_Since_PowerOn();
        
        // Raw Cache
        RC_Raw.joystick_left_x[RC_ADC_Raw_Cnt] = ADC_Ch(JOYSTICK_L_X_CH);
        RC_Raw.joystick_left_y[RC_ADC_Raw_Cnt] = ADC_Ch(JOYSTICK_L_Y_CH);
        
        RC_Raw.joystick_right_x[RC_ADC_Raw_Cnt] = ADC_Ch(JOYSTICK_R_X_CH);
        RC_Raw.joystick_right_y[RC_ADC_Raw_Cnt] = ADC_Ch(JOYSTICK_R_Y_CH);
        
        RC_Raw.v_bat_all[RC_ADC_Raw_Cnt] = ADC_Ch(BAT_ALARM_UP_CH);
        RC_Raw.v_bat_down[RC_ADC_Raw_Cnt] = ADC_Ch(BAT_ALARM_DN_CH);
        
        RC_Raw.random = ADC_Ch(RANDOM_CH);
        
        RC_ADC_Raw_Cnt++;        
        if(RC_ADC_Raw_Cnt >= RC_RAW_NUM)
            RC_ADC_Raw_Cnt = 0;
        
        RC_Raw.switch2_left = RC_Switch2_Left_Status();
        RC_Raw.switch2_right = RC_Switch2_Right_Status();
        
        RC_Raw.joystick_key_left = RC_Key_Joystick_Left_Status();
        RC_Raw.joystick_key_right = RC_Key_Joystick_Right_Status();
        
        // Raw Filter
        RC_Raw.filter_joystick_left_x = Data_Filter_uint16(RC_Raw.joystick_left_x, RC_RAW_NUM);
        RC_Raw.filter_joystick_left_y = Data_Filter_uint16(RC_Raw.joystick_left_y, RC_RAW_NUM);
        
        RC_Raw.filter_joystick_right_x = Data_Filter_uint16(RC_Raw.joystick_right_x, RC_RAW_NUM);
        RC_Raw.filter_joystick_right_y = Data_Filter_uint16(RC_Raw.joystick_right_y, RC_RAW_NUM);
        
        RC_Raw.filter_v_bat_all = Data_Filter_uint16(RC_Raw.v_bat_all, RC_RAW_NUM);
        RC_Raw.filter_v_bat_down = Data_Filter_uint16(RC_Raw.v_bat_down, RC_RAW_NUM);
        
        // Convert to Target Value
        RC.joystick_left.x = Joystick_Raw_2_Target(RC_Raw.filter_joystick_left_x, &RC_Para.yaw, &JoystickCal_Target);
        RC.joystick_left.y = Joystick_Raw_2_Target(RC_Raw.filter_joystick_left_y, &RC_Para.thr, &JoystickCal_Target);
        
        RC.joystick_right.x = Joystick_Raw_2_Target(RC_Raw.filter_joystick_right_x, &RC_Para.rol, &JoystickCal_Target);
        RC.joystick_right.y = Joystick_Raw_2_Target(RC_Raw.filter_joystick_right_y, &RC_Para.pit, &JoystickCal_Target);
        
        RC.switch2_left = (Swtich2_Down == RC_Raw.switch2_left) ? 1000 : 2000;
        RC.switch2_right = (Swtich2_Down == RC_Raw.switch2_right) ? 1000 : 2000;
        
        RC.joystick_left.key = (KeyRelease == RC_Raw.joystick_key_left) ? 1000 : 2000;
        RC.joystick_right.key = (KeyRelease == RC_Raw.joystick_key_right) ? 1000 : 2000;        
        
        RC.random = RC_Raw.random;
        
        RC.v_bat_all = RC_Raw.filter_v_bat_all * 4.0 * 3.3 / 4096;
        RC.v_bat_down = RC_Raw.filter_v_bat_down * 2.0 * 3.3 / 4096;
        RC.v_bat_up = RC.v_bat_all - RC.v_bat_down;
        
        // Update Battery Alarm LED status
        if(RC.v_bat_up < BATTERY_ALARM_VAL)
            RC_LED_Bat_Alarm_Up_On();
        else
            RC_LED_Bat_Alarm_Up_Off();
        
        if(RC.v_bat_down < BATTERY_ALARM_VAL)
            RC_LED_Bat_Alarm_Down_On();
        else
            RC_LED_Bat_Alarm_Down_Off();
        
        // Update RC Message
        RC_Msg_Update();
    }
}

extern void RC_Send_Msg(void)
{
    static uint64 Last_ms = 0;
    
    if((ms_Since_PowerOn() - Last_ms) >= 20)
    {
        Last_ms = ms_Since_PowerOn();
        
        if(KeyPressed == RC_Key_Joystick_Right_PowerOn_Status())
        {
            sprintf(    (char*)UCA1_TxBuf, "%4d[%4d]  %4d[%4d]  %4d[%4d]  %4d[%4d]  A%4d  B%4d  L%4d  R%4d  %5.3fV[%4d]  %5.3fV  %5.3fV[%4d]\r\n",
                        RC.joystick_right.x,
                        RC_Raw.filter_joystick_right_x,
                        RC.joystick_right.y,
                        RC_Raw.filter_joystick_right_y,
                        RC.joystick_left.y,
                        RC_Raw.filter_joystick_left_y,
                        RC.joystick_left.x,
                        RC_Raw.filter_joystick_left_x,
                        RC.switch2_left,
                        RC.switch2_right,
                        RC.joystick_left.key,
                        RC.joystick_right.key,
                        RC.v_bat_all,
                        RC_Raw.filter_v_bat_all,
                        RC.v_bat_up,
                        RC.v_bat_down,
                        RC_Raw.filter_v_bat_down
                    );
            
            Usart_A1_Send(UCA1_TxBuf, strlen((char const*)UCA1_TxBuf));
        }
        else
        {
            Usart_A1_Send(RC_Msg, sizeof(RC_Msg));
        }
    }
}

extern void RC_Save_Para_To_Flash(void)
{
    Flash_Write_SegA((uint8*)&RC_Para, sizeof(RC_Para_t));
}

extern OptSta_t RC_Load_Para_From_Flash(void)
{
    uint8 i;
    
    RC_Para_t* pPara;
    
    // 读取
    Flash_Read_Seg_A();
    
    pPara = (RC_Para_t*)SegA;
    
    // 检查信息标识头
    for(i=0; i<sizeof(RC_Para_Head); i++)
    {
        if(pPara->head[i] != RC_Para_Head[i])
            return Failure;
    }
    
    // 检查校验
    if(pPara->crc != RC_Parc_CRC(pPara))
        return Failure;
    
    RC_Para.rol.min = pPara->rol.min;
    RC_Para.rol.mid = pPara->rol.mid;
    RC_Para.rol.max = pPara->rol.max;
    
    RC_Para.pit.min = pPara->pit.min;
    RC_Para.pit.mid = pPara->pit.mid;
    RC_Para.pit.max = pPara->pit.max;
    
    RC_Para.thr.min = pPara->thr.min;
    RC_Para.thr.mid = pPara->thr.mid;
    RC_Para.thr.max = pPara->thr.max;
    
    RC_Para.yaw.min = pPara->yaw.min;
    RC_Para.yaw.mid = pPara->yaw.mid;
    RC_Para.yaw.max = pPara->yaw.max;
  
    return Success;
}

extern void RC_Calibration(void)
{
    /*
        程序运行时，会自动从 Flassh 里面加载参数
        若从 Flash 里面读取的参数错误
        或者上电时，按下开发板上的 S1 或者 S2
        都将进行遥控摇杆校准

        看到电压报警灯均亮起
        开始拨动摇杆到最大范围

        再次按下开发板上的 S1 或者 S2 退出校准
    */

    #define RC_CAL_CACHE_LEN    10
  
    typedef struct
    {
        uint16 raw;
        
        uint16 cache_min[RC_CAL_CACHE_LEN];
        uint16 cache_max[RC_CAL_CACHE_LEN];
        
        uint16 min_sum;
        uint16 mid_sum;
        uint16 max_sum;
        
        uint16 min;
        uint16 mid;
        uint16 max;
    }RC_Cal_Ch_Cache_t;
  
    YesNo_t Waiting_For_Calibration = No;
    
    uint16 i, j;
    
    RC_Cal_Ch_Cache_t  RC_Cal_Rol;
    RC_Cal_Ch_Cache_t  RC_Cal_Pit;
    RC_Cal_Ch_Cache_t  RC_Cal_Thr;
    RC_Cal_Ch_Cache_t  RC_Cal_Yaw;
    
    // 加载参数失败 则提示进行校准
    if(Failure == RC_Load_Para_From_Flash())
    {
        Waiting_For_Calibration = Yes;
    }    
    // 强制要求进行校准
    else if((KeyPressed == Brd_Key_S1_PowerOn_Status()) ||  (KeyPressed == Brd_Key_S2_PowerOn_Status()))
    {
        Waiting_For_Calibration = Yes;
        
        // 等待按键释放
        while((KeyPressed == Brd_Key_S1_Status()) ||  (KeyPressed == Brd_Key_S2_Status()))
            RC_Beep_On();
        
        RC_Beep_Off();
    }
    
    if(Yes == Waiting_For_Calibration)
    {
        // 延时 让 ADC 等稳定
        delay_ms(500);
        
        RC_Cal_Rol.mid_sum = 0;
        RC_Cal_Pit.mid_sum = 0;
        RC_Cal_Thr.mid_sum = 0;
        RC_Cal_Yaw.mid_sum = 0;
        
        // 采集中立点值
        for(i=0; i<RC_CAL_CACHE_LEN; i++)
        {
            RC_Cal_Rol.mid_sum += ADC_Ch(JOYSTICK_R_X_CH);
            RC_Cal_Pit.mid_sum += ADC_Ch(JOYSTICK_R_Y_CH);
            RC_Cal_Thr.mid_sum += ADC_Ch(JOYSTICK_L_Y_CH);
            RC_Cal_Yaw.mid_sum += ADC_Ch(JOYSTICK_L_X_CH);
            
            delay_ms(10);
        }
        
        RC_Cal_Rol.mid = RC_Cal_Rol.mid_sum / RC_CAL_CACHE_LEN;
        RC_Cal_Pit.mid = RC_Cal_Pit.mid_sum / RC_CAL_CACHE_LEN;
        RC_Cal_Thr.mid = RC_Cal_Thr.mid_sum / RC_CAL_CACHE_LEN;
        RC_Cal_Yaw.mid = RC_Cal_Yaw.mid_sum / RC_CAL_CACHE_LEN;
        
        // 将中值赋给各个通道缓存值
        for(i=0; i<RC_CAL_CACHE_LEN; i++)
        {
            RC_Cal_Rol.cache_min[i] = RC_Cal_Rol.mid;
            RC_Cal_Pit.cache_min[i] = RC_Cal_Pit.mid;
            RC_Cal_Thr.cache_min[i] = RC_Cal_Thr.mid;
            RC_Cal_Yaw.cache_min[i] = RC_Cal_Yaw.mid;
            
            RC_Cal_Rol.cache_max[i] = RC_Cal_Rol.mid;
            RC_Cal_Pit.cache_max[i] = RC_Cal_Pit.mid;
            RC_Cal_Thr.cache_max[i] = RC_Cal_Thr.mid;
            RC_Cal_Yaw.cache_max[i] = RC_Cal_Yaw.mid;
        }
        
        // 采集各个通道最大值、最小值
        RC_LED_Bat_Alarm_Up_On();
        RC_LED_Bat_Alarm_Down_On();
        
        // 采集各个通道最大值、最小值
        while((KeyRelease == Brd_Key_S1_Status()) &&  (KeyRelease == Brd_Key_S2_Status()))
        {
            RC_Cal_Rol.raw = ADC_Ch(JOYSTICK_R_X_CH);
            RC_Cal_Pit.raw = ADC_Ch(JOYSTICK_R_Y_CH);
            RC_Cal_Thr.raw = ADC_Ch(JOYSTICK_L_Y_CH);
            RC_Cal_Yaw.raw = ADC_Ch(JOYSTICK_L_X_CH);
            
            // 横滚 处理            
            for(j=0; j<RC_CAL_CACHE_LEN; j++)
            {
                if(RC_Cal_Rol.raw > RC_Cal_Rol.cache_max[j])
                {
                    RC_Cal_Rol.cache_max[j] = RC_Cal_Rol.raw;
                    break;
                }
            }
            
            for(j=0; j<RC_CAL_CACHE_LEN; j++)
            {
                if(RC_Cal_Rol.raw < RC_Cal_Rol.cache_min[j])
                {
                    RC_Cal_Rol.cache_min[j] = RC_Cal_Rol.raw;
                    break;
                }
            }
            // 俯仰 处理            
            for(j=0; j<RC_CAL_CACHE_LEN; j++)
            {
                if(RC_Cal_Pit.raw > RC_Cal_Pit.cache_max[j])
                {
                    RC_Cal_Pit.cache_max[j] = RC_Cal_Pit.raw;
                    break;
                }
            }
            
            for(j=0; j<RC_CAL_CACHE_LEN; j++)
            {
                if(RC_Cal_Pit.raw < RC_Cal_Pit.cache_min[j])
                {
                    RC_Cal_Pit.cache_min[j] = RC_Cal_Pit.raw;
                    break;
                }
            }
            // 油门 处理            
            for(j=0; j<RC_CAL_CACHE_LEN; j++)
            {
                if(RC_Cal_Thr.raw > RC_Cal_Thr.cache_max[j])
                {
                    RC_Cal_Thr.cache_max[j] = RC_Cal_Thr.raw;
                    break;
                }
            }
            
            for(j=0; j<RC_CAL_CACHE_LEN; j++)
            {
                if(RC_Cal_Thr.raw < RC_Cal_Thr.cache_min[j])
                {
                    RC_Cal_Thr.cache_min[j] = RC_Cal_Thr.raw;
                    break;
                }
            }
            // 航向 处理            
            for(j=0; j<RC_CAL_CACHE_LEN; j++)
            {
                if(RC_Cal_Yaw.raw > RC_Cal_Yaw.cache_max[j])
                {
                    RC_Cal_Yaw.cache_max[j] = RC_Cal_Yaw.raw;
                    break;
                }
            }
            
            for(j=0; j<RC_CAL_CACHE_LEN; j++)
            {
                if(RC_Cal_Yaw.raw < RC_Cal_Yaw.cache_min[j])
                {
                    RC_Cal_Yaw.cache_min[j] = RC_Cal_Yaw.raw;
                    break;
                }
            }
            
            delay_ms(1);
        }
        
        // 最大值最小值采集完毕
        RC_LED_Bat_Alarm_Up_Off();
        RC_LED_Bat_Alarm_Down_Off();
        
        // 新的校准值
        RC_Cal_Rol.min_sum = 0;
        RC_Cal_Pit.min_sum = 0;
        RC_Cal_Thr.min_sum = 0;
        RC_Cal_Yaw.min_sum = 0;
        
        RC_Cal_Rol.max_sum = 0;
        RC_Cal_Pit.max_sum = 0;
        RC_Cal_Thr.max_sum = 0;
        RC_Cal_Yaw.max_sum = 0;
        
        // 计算累加和
        for(i=0; i<RC_CAL_CACHE_LEN; i++)
        {
            RC_Cal_Rol.min_sum += RC_Cal_Rol.cache_min[i];
            RC_Cal_Pit.min_sum += RC_Cal_Pit.cache_min[i];
            RC_Cal_Thr.min_sum += RC_Cal_Thr.cache_min[i];
            RC_Cal_Yaw.min_sum += RC_Cal_Yaw.cache_min[i];
            
			
            RC_Cal_Rol.max_sum += RC_Cal_Rol.cache_max[i];
            RC_Cal_Pit.max_sum += RC_Cal_Pit.cache_max[i];
            RC_Cal_Thr.max_sum += RC_Cal_Thr.cache_max[i];
            RC_Cal_Yaw.max_sum += RC_Cal_Yaw.cache_max[i];
        }
        
        RC_Cal_Rol.min = RC_Cal_Rol.min_sum / RC_CAL_CACHE_LEN;
        RC_Cal_Pit.min = RC_Cal_Pit.min_sum / RC_CAL_CACHE_LEN;
        RC_Cal_Thr.min = RC_Cal_Thr.min_sum / RC_CAL_CACHE_LEN;
        RC_Cal_Yaw.min = RC_Cal_Yaw.min_sum / RC_CAL_CACHE_LEN;
        
        RC_Cal_Rol.max = RC_Cal_Rol.max_sum / RC_CAL_CACHE_LEN;
        RC_Cal_Pit.max = RC_Cal_Pit.max_sum / RC_CAL_CACHE_LEN;
        RC_Cal_Thr.max = RC_Cal_Thr.max_sum / RC_CAL_CACHE_LEN;
        RC_Cal_Yaw.max = RC_Cal_Yaw.max_sum / RC_CAL_CACHE_LEN;
        
    
        RC_Para.rol.min = RC_Cal_Rol.min;
        RC_Para.rol.mid = RC_Cal_Rol.mid;
        RC_Para.rol.max = RC_Cal_Rol.max;
        
        RC_Para.pit.min = RC_Cal_Pit.min;
        RC_Para.pit.mid = RC_Cal_Pit.mid;
        RC_Para.pit.max = RC_Cal_Pit.max;
        
        RC_Para.thr.min = RC_Cal_Thr.min;
        RC_Para.thr.mid = RC_Cal_Thr.mid;
        RC_Para.thr.max = RC_Cal_Thr.max;
        
        RC_Para.yaw.min = RC_Cal_Yaw.min;
        RC_Para.yaw.mid = RC_Cal_Yaw.mid;
        RC_Para.yaw.max = RC_Cal_Yaw.max;
        
        RC_Para.crc = RC_Parc_CRC(&RC_Para);
        
        // Save to Flash
        RC_Save_Para_To_Flash();
    }
}

extern void RC_LED_Beep_Test(void)
{
    uint64 _ms_start;
    
    RC_LED_Bat_Alarm_Up_On();
    RC_LED_Bat_Alarm_Up_Off();
    RC_LED_Bat_Alarm_Up_On();
    RC_LED_Bat_Alarm_Up_Off();
    
    RC_LED_Bat_Alarm_Down_On();
    RC_LED_Bat_Alarm_Down_Off();
    RC_LED_Bat_Alarm_Down_On();
    RC_LED_Bat_Alarm_Down_Off();
    
    RC_Beep_On();
    RC_Beep_Off();
    RC_Beep_On();
    RC_Beep_Off();
    
    _ms_start = ms_Since_PowerOn();
    
    
    RC_LED_Bat_Alarm_Up_On();
    RC_LED_Bat_Alarm_Down_On();    
    RC_Beep_On();
    
    while((ms_Since_PowerOn() - _ms_start) < 50)
        ;
    
    RC_LED_Bat_Alarm_Up_Off();
    RC_LED_Bat_Alarm_Down_Off();
    RC_Beep_Off();
}

