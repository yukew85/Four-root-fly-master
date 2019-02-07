#include "data_filter.h"

extern void Data_Quick_Sort_uint16(uint16* pDatas, uint16 Num)
{
    uint16 CntL, CntR;
    uint16 Compare;
    
    if(Num < 2)
        return;
	
    CntL = 0;
    CntR = Num - 1;
	
	Compare = pDatas[CntL];  		// first item saved, and be used for comparer
   
    while(CntL < CntR)
    {
        while(CntL < CntR)
        {
            if(pDatas[CntR] < Compare)
            {
                pDatas[CntL] = pDatas[CntR];
                break;
            }
            CntR--;
        }
        
        while(CntL < CntR)
        {
            if(pDatas[CntL] > Compare)
            {
                pDatas[CntR] = pDatas[CntL];
                break;
            }
            CntL++;
        }
    }
    
    pDatas[CntL++] = Compare;                           // rember to cover the last saved element with Comparer
    Data_Quick_Sort_uint16(&pDatas[0], CntL);           // pDatas[0]        ~   pDatas[CntL-1]
    Data_Quick_Sort_uint16(&pDatas[CntL], Num-(CntL));  // pDatas[CntL] ~   pDatas[Num-1]
}

extern uint16 Data_Filter_uint16(uint16* pDataBuf, uint16 Num)
{
    #define DATA_NUM_MAX    100
    
    static uint16 DataCache[DATA_NUM_MAX];
    static uint32 Sum;
    uint16 i;
    
    if(Num > DATA_NUM_MAX)
      return 0;
    
    if(1 == Num)
    {
        return pDataBuf[0];
    }
    else if(2 == Num)
    {
        Sum = (pDataBuf[0] + pDataBuf[1]) / 2;
        return Sum;
    }
    else if(Num >= 3)
    {
        for(i=0; i<Num; i++)
            DataCache[i] = pDataBuf[i];
        
        Data_Quick_Sort_uint16(DataCache, Num);
        Sum = 0.00;
        for(i=1; i<=Num-2; i++)
            Sum += DataCache[i];
        
        return Sum/(Num-2);
    }
    
    return 0;
}

extern uint16 Get_Linear_Function_y_uint16(uint16 x, uint16 x0, uint16 y0, uint16 x1, uint16 y1)
{
    /*
        (y - y0) / (x - x0) = (y1 - y0) / (x1 - x0)
        
        y = ((x - x0) * (y1 - y0)) / (x1 - x0) + y0
    */
  
    return(((uint32)x - (uint32)x0) * ((uint32)y1 - (uint32)y0)) / ((uint32)x1 - (uint32)x0) + (uint32)y0;
}

