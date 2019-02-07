#include "iic.h"
#include "gpio.h"
#include "ucs.h"
#include "timer.h"
#include "led.h"

#define IIC_TIMEOUT     IIC_Delay_Cnt_Max

volatile static uint32  IIC_Delay_Cnt_Max = 500;

extern void IIC_Init(uint8 ClockSource, uint32 IIC_Clock_Hz)
{
    float f_src_f_iic = 1.0;
    uint32 ClockSource_Hz = 0;
    
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3, GPIO_PIN0);
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3, GPIO_PIN1);
    
    USCI_B_I2C_initMasterParam USCI_B_I2C_initMasterParam_cache;
    
    if(USCI_B_I2C_CLOCKSOURCE_SMCLK == ClockSource)
    {
        USCI_B_I2C_initMasterParam_cache.selectClockSource = USCI_B_I2C_CLOCKSOURCE_SMCLK;
        ClockSource_Hz = UCS_getSMCLK();
    }
    else    //(USCI_B_I2C_CLOCKSOURCE_ACLK == ClockSource)
    {
        USCI_B_I2C_initMasterParam_cache.selectClockSource = USCI_B_I2C_CLOCKSOURCE_ACLK;
        ClockSource_Hz = UCS_getACLK();
    }
    
    USCI_B_I2C_initMasterParam_cache.i2cClk = ClockSource_Hz;
    f_src_f_iic = (float)ClockSource_Hz / (float)IIC_Clock_Hz;
    IIC_Delay_Cnt_Max = (uint32)(f_src_f_iic * 10);
    
    if(100000 == IIC_Clock_Hz)
    {
        USCI_B_I2C_initMasterParam_cache.dataRate = USCI_B_I2C_SET_DATA_RATE_100KBPS;
    }
    else    //(400000 == IIC_Clock_Hz)
    {
        USCI_B_I2C_initMasterParam_cache.dataRate = USCI_B_I2C_SET_DATA_RATE_400KBPS;
    }
    
    USCI_B_I2C_initMaster(USCI_B0_BASE, &USCI_B_I2C_initMasterParam_cache);
    
    USCI_B_I2C_enable(USCI_B0_BASE);
    
    USCI_B_I2C_disableInterrupt(USCI_B0_BASE, USCI_B_I2C_STOP_INTERRUPT);
    USCI_B_I2C_disableInterrupt(USCI_B0_BASE, USCI_B_I2C_START_INTERRUPT);
    USCI_B_I2C_disableInterrupt(USCI_B0_BASE, USCI_B_I2C_RECEIVE_INTERRUPT);
    USCI_B_I2C_disableInterrupt(USCI_B0_BASE, USCI_B_I2C_TRANSMIT_INTERRUPT);
    USCI_B_I2C_disableInterrupt(USCI_B0_BASE, USCI_B_I2C_NAK_INTERRUPT);
    USCI_B_I2C_disableInterrupt(USCI_B0_BASE, USCI_B_I2C_ARBITRATIONLOST_INTERRUPT);
}

extern uint8 IICreadBytes(uint8 dev, uint8 reg, uint8 length, uint8 *data)
{
    uint8 ret = 0;
    uint8 i = 0;
    volatile static uint8 cnt = 0;
    
    cnt = 0;
    USCI_B_I2C_setSlaveAddress(USCI_B0_BASE, dev);
    
    if(USCI_B_I2C_masterSendMultiByteStartWithTimeout(USCI_B0_BASE, reg, IIC_TIMEOUT))
    {
        // Can't lack
      // fuck!
        while(!USCI_B_I2C_getInterruptStatus(USCI_B0_BASE, USCI_B_I2C_TRANSMIT_INTERRUPT))
            ;
        
        // Can't clear transmit interrupt here, will cause error!!!
        
        USCI_B_I2C_masterReceiveMultiByteStart(USCI_B0_BASE);
        
        for(i=0; i<length-1; i++)
        {
            cnt++;
            while(!USCI_B_I2C_getInterruptStatus(USCI_B0_BASE, USCI_B_I2C_RECEIVE_INTERRUPT))
                ;
            data[i] = USCI_B_I2C_masterReceiveMultiByteNext(USCI_B0_BASE);
        }
        
        while(!USCI_B_I2C_getInterruptStatus(USCI_B0_BASE, USCI_B_I2C_RECEIVE_INTERRUPT))
            ;
        cnt++;
        ret = USCI_B_I2C_masterReceiveMultiByteFinishWithTimeout(USCI_B0_BASE, &data[i], IIC_TIMEOUT);
    }
    
    return ret;
}

extern uint8 IIC_Read_One_Byte(uint8 dev, uint8 reg)
{
    uint8 data = 0;
    
    IICreadBytes(dev, reg, 1, &data);
      
    return data;
}

extern unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data)
{
    uint8 temp;
    
    temp =  IICwriteBytes(dev, reg, 1, &data);
    
    return temp;
}

extern uint8 IICwriteBytes(uint8 dev, uint8 reg, uint8 length, uint8* data)
{
    uint8 ret = 0;
 	uint8 i = 0;
    
    USCI_B_I2C_setSlaveAddress(USCI_B0_BASE, dev);
    
    if(USCI_B_I2C_masterSendMultiByteStartWithTimeout(USCI_B0_BASE, reg, IIC_TIMEOUT))
    {
        ret = 1;
        
        while(!USCI_B_I2C_getInterruptStatus(USCI_B0_BASE, USCI_B_I2C_TRANSMIT_INTERRUPT))
            ;
        // Can't clear transmit interrupt here, will cause error!!!
        
        for(i=0; i<length-1; i++)
        {
            if(STATUS_FAIL == USCI_B_I2C_masterSendMultiByteNextWithTimeout(USCI_B0_BASE, data[i], IIC_TIMEOUT))
            {
                while(!USCI_B_I2C_getInterruptStatus(USCI_B0_BASE, USCI_B_I2C_TRANSMIT_INTERRUPT))
                    ;
                
                ret = 0;
                break;
            }
        }
        
        if(STATUS_FAIL == USCI_B_I2C_masterSendMultiByteFinishWithTimeout(USCI_B0_BASE, data[i], IIC_TIMEOUT))
            ret = 0;
        
        while(!USCI_B_I2C_getInterruptStatus(USCI_B0_BASE, USCI_B_I2C_TRANSMIT_INTERRUPT))
            ;
        // must clear at end, otherwise cause error!!!
        USCI_B_I2C_clearInterrupt(USCI_B0_BASE, USCI_B_I2C_TRANSMIT_INTERRUPT);
    }
    
    return ret;
}

extern uint8 IICwriteBit(uint8 dev, uint8 reg, uint8 bitNum, uint8 data)
{
    uint8 b;
    
    IICreadBytes(dev, reg, 1, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    
    return IICwriteByte(dev, reg, b);
}

extern uint8 IICwriteBits(uint8 dev, uint8 reg, uint8 bitStart, uint8 length, uint8 data)
{
    uint8 b;
    
    if(IICreadBytes(dev, reg, 1, &b))
    {
        uint8 mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
        data <<= (8 - length);
        data >>= (7 - bitStart);
        b &= mask;
        b |= data;
        
        return IICwriteByte(dev, reg, b);
    }
    
    return 0;
}

extern void IIC_Test(void)
{
    IIC_Init(USCI_B_I2C_CLOCKSOURCE_ACLK, 400000);
    
    USCI_B_I2C_setSlaveAddress(USCI_B0_BASE, 0x68);
    
    delay_ms(1000);
    
    for( ; ; )
    {
        USCI_B_I2C_masterSendSingleByteWithTimeout(USCI_B0_BASE, 0xD1, 500);
        
        delay_ms(1000);
    }
}
