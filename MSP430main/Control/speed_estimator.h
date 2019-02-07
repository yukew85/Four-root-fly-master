/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：speed_estimator.h
  * 摘    要：
  *
  * 当前版本：V1.0
  * 作    者：北京中科浩电科技有限公司研发部 
  * 完成日期：    
  * 修改说明：
  * 
  *
  * 历史版本：
  *
  *
  *******************************************************************************/
#ifndef __STATE_ESTIMATOR_H
#define __STATE_ESTIMATOR_H
//外部文件引用
#include "include.h"

//函数声明
void UpdateAltSpeed(float dt);
void ResetAltSpeed(void);		 

#endif 

/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/
