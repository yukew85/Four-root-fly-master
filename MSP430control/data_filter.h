#ifndef __DATA_FILTER_H__
#define __DATA_FILTER_H__

#include "data_type.h"

extern void Data_Quick_Sort_uint16(uint16* pDataBuf, uint16 Num);
extern uint16 Data_Filter_uint16(uint16* pDataBuf, uint16 Num);
extern uint16 Get_Linear_Function_y_uint16(uint16 x, uint16 x0, uint16 y0, uint16 x1, uint16 y1);

#endif  // __DATA_FILTER_H__

