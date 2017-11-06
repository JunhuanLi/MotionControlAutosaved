/*******************************************************************************

  Copyright (C), 2017-2027, TOPBAND Co., Ltd.
  File name 	: pi.h
  Author		: George 	
  Version		: V1.0.0	  
  Date			: 2017/07/21
  Description	: motion_control 
  
  History:		  
				  
	1. Date 		:
	   Author		:
	   Modification :
	2. ...
	
*******************************************************************************/

#ifndef __MOTION_CONTROL_H__
#define __MOTION_CONTROL_H__

/* includes *******************************************************************/

#include "stm32f4xx.h"
#include "typedef.h"
#include "usart_driver.h"
#include "math.h"

#include "motion_imu.h"
#include "motion_types.h"
#include "motion_sense.h"
#include "motion_mag_line.h"

#include "hardware.h"

#include "motor_control.h"
#include "bumper_front.h"
#include "global_planner.h"

/* macros *********************************************************************/

typedef enum
{
	MOTION_EXCEPTION_NONE = 0,
	MOTION_EXCEPTION_OBSTACLE,
	MOTION_EXCEPTION_SLIP,
	MOTION_EXCEPTION_TRAPPED
}T_motion_exception_type;

typedef enum
{
	MOTION_STATE_IDLE = 0,
	MOTION_STATE_P2P,
	MOTION_STATE_ZIGZAG,
	MOTION_STATE_MAGLINE,
//	MOTION_STATE_RANDOM,
//	MOTION_STATE_INVOLUTE,
//	MOTION_STATE_FIND_MAGLINE,
//	MOTION_STATE_ALIGN_BASE,
	MOTION_STATE_OUT_STATION
}T_motion_state_type;

typedef enum
{
	T_MOTION_ZIGZAG_STATE_IDLE = 0,
	T_MOTION_ZIGZAG_STATE_LINE,
	T_MOTION_ZIGZAG_STATE_BACK,
	T_MOTION_ZIGZAG_STATE_TURN,
	T_MOTION_ZIGZAG_STATE_ONTOWIRE,
	T_MOTION_ZIGZAG_STATE_BYPASS,
	T_MOTION_ZIGZAG_STATE_EXCEPTION
}T_motion_zigzag_state_type;

typedef enum
{
	T_MOTION_ZIGZAG_TURN_COUNTERCLOCKWISE = 0,
	T_MOTION_ZIGZAG_TURN_CLOCKWISE
}T_motion_zigzag_turn_dir_type;

typedef enum
{
	T_MOTION_ZIGZAG_GO_FOWARD = 0,
	T_MOTION_ZIGZAG_GO_REVERSE
}T_motion_zigzag_foward_reverse_type;

typedef struct
{
	T_motion_zigzag_turn_dir_type					turn_dir;
	T_motion_zigzag_foward_reverse_type		f_r;				//foward or reverse
	T_motion_zigzag_state_type 						state;
	float																	blade_bodywidth_ratio;
	float																	blade_overlaping_ratio;
	float																	target_vel;
	float																	heading_x;
	float																	heading_y;

}T_motion_zigzag;

typedef struct
{
	uint8_t										enable;
	T_motion_tracker					tracker;
	T_motion_state_type				motion_state;
	T_motion_exception_type		exception;
	T_motion_zigzag						zigzag;
}T_motion;

typedef struct
{
	//unsigned char left_sensor_former;
	//unsigned char right_sensor_former;
	unsigned char left_sensor_now;
	unsigned char right_sensor_now;
	unsigned char left_sensor_old[6];
	unsigned char right_sensor_old[6];
	
	unsigned char left_sensor_change;
	unsigned char right_sensor_change;
}MAG_STATUE;
 
extern MAG_STATUE mag_state;
extern T_motion motion;



/* static variables ***********************************************************/
/* funcitons ******************************************************************/
void Motion_Init(T_motion* motion,uint8_t en);
void Motion_Run(T_motion* motion);
void Motion_Process_Obstacle(T_motion* motion);

void Motion_Zigzag_Init(T_motion* motion,float blade_bodywidth_ratio,float blade_overlaping_ratio); 
void Motion_Zigzag_Start(T_motion* motion,float speed,float heading_x,float heading_y,T_motion_zigzag_turn_dir_type turn_dir);

void mag_sensor_initial(void);

void motion_cover(void);
void mtn_onto_wire(T_gp_side enterSide, float rot_vel, float linear_vel);
void Motion_Process_Motor_Speed(void);
float tune_linear_vel(void);
float get_dist_from_sensors(void);
//void obstacle_avoidance(T_motion* motion, float radius);
void obstacle_avoidance(float linear_vel, float angular_vel);
void action_params_print(void);
void processing_bump(void);
void get_decision(T_gp_trigger *trigger_ptr, T_gp_decision *decision_ptr);
void out_station(float v);
#endif /* __MOTION_CONTROL_H__ */
