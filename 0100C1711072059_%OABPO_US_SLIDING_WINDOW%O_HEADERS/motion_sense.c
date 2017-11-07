/*******************************************************************************

  Copyright (C), 2017-2027, TOPBAND Co., Ltd.
  File name 	: path_tracking.c
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
#include "motion_sense.h"

#define MTN_SONAR_VALID_TOP 2.5
#define MTN_SONAR_VALID_BOT 0
#define WINDOW_LENGTH 10
/* macros *********************************************************************/
/* static variables ***********************************************************/
static __inline void Tracking_Norm_2D(float* x, float* y)
{
	float length = sqrtf((*x)*(*x) + (*y)*(*y));
	if(length != 0)
	{
		*x = *x / length;
		*y = *y / length;
	}
}

void Motion_Get_Position_2D(T_motion_sense* obj)
{
	//extern float rot_vec[3];
	float x = rot_vec[0],y = rot_vec[1];// x, y is temp var
	Motion_Norm_2D(&x,&y);
	obj->dir_x = x;
	obj->dir_y = y;
	
	x = pos_ned_m[0];
	y = pos_ned_m[1];
	obj->pos_x = x;
	obj->pos_y = y;

}

float mtn_window_left[WINDOW_LENGTH] = {0};
float mtn_window_right[WINDOW_LENGTH] = {0};
int g_mtn_window_idx = 0;
void ultrasonic_sliding_window(T_motion_sense* obj)
{
	mtn_window_left[g_mtn_window_idx] = obj->sonar_l;
	mtn_window_right[g_mtn_window_idx] = obj->sonar_l;
	int i=0;
	int sum_left=0;
	int sum_right=0;
	for(i=0; i<WINDOW_LENGTH; i++)
	{
		sum_left += mtn_window_left[i];
		sum_right += mtn_window_right[i];
	}
	obj->sonar_l = sum_left / WINDOW_LENGTH;
	obj->sonar_r = sum_right / WINDOW_LENGTH;
	g_mtn_window_idx++;
	if(g_mtn_window_idx >= 10)
	{
		g_mtn_window_idx = 0;
	}
	
}

void Motion_Get_Sensor(T_motion_sense* obj)
{
	//get mag sensor data
	int32_t left 	= leftsensor_data;
	int32_t right = rightsensor_data;
	
  get_sonar_info(&g_mtn_sonar);
	obj->sonar_l = g_mtn_sonar.left/1000.0;
	obj->sonar_r = g_mtn_sonar.right/1000.0;
	ultrasonic_sliding_window(obj);
	if(obj->sonar_l >= MTN_SONAR_VALID_TOP)
		obj->sonar_l = MTN_SONAR_VALID_TOP;
	if(obj->sonar_l <= MTN_SONAR_VALID_BOT)
		obj->sonar_l = MTN_SONAR_VALID_BOT;
	if(obj->sonar_r >= MTN_SONAR_VALID_TOP)
		obj->sonar_r = MTN_SONAR_VALID_TOP;
	if(obj->sonar_r <= MTN_SONAR_VALID_BOT)
		obj->sonar_r = MTN_SONAR_VALID_BOT;
//	if(left < 0)
//	{
//		if(left > -MAG_LINE_MIN)
//		{
//			obj->side_l = MOTION_MAG_LINE_MISSING;
//			obj->value_l = 0;
//		}
//		else
//		{
//			obj->side_l = MOTION_MAG_LINE_OUTSIDE;
//			obj->value_l = -left;
//		}
//	}
//	else
//	{
//		if(left < MAG_LINE_MIN)
//		{
//			obj->side_l = MOTION_MAG_LINE_MISSING;
//			obj->value_l = 0;
//		}
//		else
//		{
//			obj->side_l = MOTION_MAG_LINE_INSIDE;
//			obj->value_l = left;
//		}
//	}
//	
//	if(right <0)
//	{
//		if(right > -MAG_LINE_MIN)
//		{
//			obj->side_r = MOTION_MAG_LINE_MISSING;
//			obj->value_r = 0;
//		}
//		else
//		{
//			obj->side_r = MOTION_MAG_LINE_OUTSIDE;
//			obj->value_r = -right;
//		}
//	}
//	else
//	{
//		if(right < MAG_LINE_MIN)
//		{
//			obj->side_r = MOTION_MAG_LINE_MISSING;
//			obj->value_r = 0;
//		}
//		else
//		{
//			obj->side_r = MOTION_MAG_LINE_INSIDE;
//			obj->value_r = right;
//		}
//	}

if(left < 0)
	{
			obj->side_l = MOTION_MAG_LINE_OUTSIDE;
			obj->value_l = -left;
	}
	else
	{
			obj->side_l = MOTION_MAG_LINE_INSIDE;
			obj->value_l = left;
	}
	
	if(right <0)
	{
			obj->side_r = MOTION_MAG_LINE_OUTSIDE;
			obj->value_r = -right;
	}
	else
	{
			obj->side_r = MOTION_MAG_LINE_INSIDE;
			obj->value_r = right;
	}
}
