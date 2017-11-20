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
	MOTION_STATE_WIRE,
//	MOTION_STATE_RANDOM,
//	MOTION_STATE_INVOLUTE,
//	MOTION_STATE_FIND_MAGLINE,
//	MOTION_STATE_ALIGN_BASE,
	MOTION_STATE_OUT_STATION
}T_motion_state_type;

typedef enum
{
	T_MOTION_ZIGZAG_STATE_NONE = 0,
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

typedef enum 
{
	FIRST_ROTATION = 0,
	AUTO_ARC,
	SECOND_ROTATION,
	OA_FINISHED
}T_oa_stage;

typedef enum
{
	NOT_BMPD = 0,
	BUMPED,
	BACK
//	BACK_FINISHED	
}T_bump_stage;

typedef enum
{
	CLEAR = 0,
	DECS,
	W8_RM,
	OBS_RMD
}T_os_stage;

typedef enum
{
	RUN = 0,
	BMPD,
	OBST_SNSD
}T_motion_stage;
	
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
extern unsigned short g_counter;

extern struct rt_messagequeue mt_mq; 
extern struct rt_messagequeue gp_mq; 
extern struct rt_messagequeue gp_app_mq; 


/*vehicle parameters*/
#define LEFT_WHEEL_RADIUS 0.0915 //meter
#define RIGHT_WHEEL_RADIUS 0.0915 //meter
#define VEHICLE_BODY_WIDTH 0.35 //meter
#define WHEEL_OMEGA_MAX 6.98 //maximum angluar velocity of the wheels 3000/45(rpm) == 6.98(rad/s)
#define VEHICLE_BODY_OMEGA_MAX 3.65 //maximum angular velocity of the vehicle 0.63837/0.35/2 ~= 3.65 rad/s 
/*motion parameters*/
//#define BACK_TIME_CNT_B4_UTURN 100

#define BACK_TCNT_B4_UT 100     // back time count before u turn
#define BACK_LNR_VEL_AFT_BMP -0.1 //linear back velocity after bump
#define BACK_TCNT_AFT_BMP 100    // back time count after bump
#define BACK_LNR_VEL_AFT_DPT -0.15  // linear back velocity after depart
#define OT_DIST_INFLU_RATIO 1.5
#define ROT_VEL_OW 0.3             // rotate velocity when onto wire
#define LNR_VEL_OW 0								// linear velocity when onto wire

#define OS_BOT_VEL 0.1               // the bottom velocity when obstacle sensed
#define OA_DECELERATION_DIST 1 // the distance the vehicle going to decelerate
#define OA_AVOIDANCE_DIST 0.35  // the distance the vehicle going to avoid the obstacle or going to bump it
#define OA_RESUME_DIST 0.45
#define HEADING_CTRL_LINEAR_VEL 0.3 // the linear heading control velocity 
#define ZIGZAG_UTURN_LINEAR_VEL 0.2  //u turn linear velocity in zigzag state
#define BACK_LNR_VEL_B4_UT -0.2  //linear back velocity before u turn

#define ARC_LINEAR_VEL 0.2   // the linear velocity in arc
#define ARC_ANGULAR_VEL 0.5		//the angular velocity in arc
#define ARC_FINISH_TIME 8 // arc finish time (in second)
#define ARC_RADIUS 0.5 // finish_time ~= 90/(omega*57.3) radius ~= finish_time * velocity/2
#define MTN_THREAD_TIME 0.015

#define ULTRASONIC_ON
/* funcitons ******************************************************************/
void Motion_Init(T_motion* motion,uint8_t en);
void Motion_Run(T_motion* motion);

void Motion_Zigzag_Init(T_motion* motion,float blade_bodywidth_ratio,float blade_overlaping_ratio); 
void Motion_Zigzag_Start(T_motion* motion,float speed,float heading_x,float heading_y,T_motion_zigzag_turn_dir_type turn_dir);

void mag_sensor_initial(void);

void mtn_FSM(void);

T_bool bump_check(void);
T_bool bk_after_bump(void);
void get_os_decision(void);
void get_or_decision(void);
void get_rt_decision(void);
T_bool obst_check(void);
void get_tuned_vel();
float tune_linear_vel(void);
float get_dist_from_sensors(void);
T_bool get_bump_state(void);
void mtn_obstacle_avoidance(void);

void app_mtn_assigner(void);
void ws_mtn_assigner(void);
void dc_mtn_assigner(void);
void rt_mtn_assigner(void);
void pr_mtn_assigner(void);
void os_mtn_assigner(void);
void or_mtn_assigner(void);

void mtn_path_tracker(void);
void action_params_print(void);
void mtn_cover_FSM(void);
void mtn_onto_wire(T_gp_side enterSide, float rot_vel, float linear_vel);
void Motion_Process_Motor_Speed(void);
void get_decision(T_gp_trigger *trigger_ptr, T_gp_decision *decision_ptr);
void out_station(float v);
void heading_control(void);
void set_motor_zero(void);
//void obstacle_avoidance(T_motion* motion, float radius);
#endif /* __MOTION_CONTROL_H__ */
