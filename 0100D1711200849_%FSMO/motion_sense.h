/*******************************************************************************

  Copyright (C), 2017-2027, TOPBAND Co., Ltd.
  File name 	: path_tracking.h
  Author		: George 	
  Version		: V1.0.0	  
  Date			: 2017/07/21
  Description	: path_tracking 
  
  History:		  
				  
	1. Date 		:
	   Author		:
	   Modification :
	2. ...
	
*******************************************************************************/
#ifndef __MOTION_SENSE_H__
#define __MOTION_SENSE_H__

#include "stm32f4xx.h"
#include "usart_driver.h"
#include "motion_types.h"
#include "motion_math.h"
#include "sonar.h"
#include "global.h"
#include "typedef.h"
#include "pi.h"

#define MTN_SONAR_VALID_TOP 2.5
#define MTN_SONAR_VALID_BOT 0
#define WINDOW_LENGTH 30

extern T_sonar g_mtn_sonar;

void Motion_Get_Sensor(T_motion_sense* obj);
void Motion_Get_Position_2D(T_motion_sense* obj);
void Motion_Get_Position_3D(T_motion_sense* obj);

#endif
