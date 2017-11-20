
/*************************************************
  Copyright (C), 2017-2027, TOPBAND Co., Ltd.
  File name:		thread_motion.c
  Author:			Raymond
  Date:				2017.7.7
  Version:        
  Description:    // 用于详细说明此程序文件完成的主要功能，与其他模块
                  // 或函数的接口，输出值、取值范围、含义及参数间的控
                  // 制、顺序、独立或依赖等关系
  History:        // 修改历史记录列表，每条修改记录应包括修改日期、修改
                  // 者及修改内容简述  
    1. Date:			2017.9.3
       Author:		Nate
       Modification:zigzag for map planner test
    2. ...
*************************************************/

#include "stm32f4xx.h"
#include <rtthread.h>
#include <stdio.h>
#include "mower_common.h"
#include "motion_control.h"


extern T_bool g_end_of_to_pose;
extern T_bump_stage g_bump_stage;
extern T_os_stage g_os_stage;
unsigned short g_counter = 0;
char mt_msg_pool[64];

/*在分配堆栈空间时，必须要对其*/
ALIGN(RT_ALIGN_SIZE)
char thread_motion_stack[4096];

struct rt_thread thread_motion;
struct rt_messagequeue mt_mq; 

extern T_motion_stage g_mtn_stage;	
extern T_frontBumper g_Bumper;
extern T_gp_trigger g_trigger;
extern T_gp_decision g_decision;
extern T_bool g_bumped;
extern T_motion motion;
extern rt_uint8_t start_move;
extern T_bool g_pause_motion;
/************************************/

void mower_motion_thread(void* parameter)
{	
	rt_uint32_t recved;

	mag_sensor_initial();
	
	rt_thread_delay(1000); //need be removed later
	Motion_Init(&motion, 1);
	
	rt_kprintf("\r\n Thread Motion Initializing...");
	
	rt_kprintf("\r\n g_att_valid == %d", g_att_valid);
	while(g_att_valid < 2)
	{
		
		hw_ms_delay(1);
	}
	rt_kprintf("\r\n g_att_valid ok...");hw_ms_delay(1);
	
	rt_kprintf("\r\n wait for button press...");
//	while(1 == start_move)
//	{
//		
//	}
	rt_kprintf("\r\n button pressed, motion control start...\n");
	
	g_trigger = INIT_PROC;
	get_decision(&g_trigger, &g_decision);
	action_params_print();
  //rt_kprintf("action:%d\n", g_decision.action);
	if(g_decision.action == DEPART)
			motion.motion_state = MOTION_STATE_OUT_STATION;
	else
		rt_kprintf("Motion recvd invalid initial action.");
    /*onto wire test block*/
//    g_decision.action = ONTO_WIRE;
//    g_decision.params.onto_wire.fin_vec[0] = 1.0;
//    g_decision.params.onto_wire.fin_vec[1] = 0.0;
    
//	while(1)
//	{
//		rt_event_recv(&sys_event, SYS_EVN_MOTION, RT_EVENT_FLAG_AND|RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved);
//		Motion_Get_Position_2D(&motion.tracker.sense);
//		Motion_Get_Sensor(&motion.tracker.sense);


//    if(motion.motion_state == MOTION_STATE_WIRE)
//		{
//			motion.tracker.target_vel = 0.4;
//			motion.tracker.line_vel = 0.4;
//			
//			Motion_Mag_Line_Test(&motion.tracker);
//			
//			record_track(); // global_planner_record 
//    }
//    else
//    {
//       onto_wire(LEFT_STEERING, 0.1);
//       rt_kprintf("INSIDE ONTO WIRE\n");
//    }
//		//set_velocity(0.0, 0.0);
//		Motion_Process_Motor_Speed();
//		update_motor_control();
//		
//	}

    /*endof onto wire test block*/ 

		/*back to base test*/
//		T_bool bumper_flag = FALSE;
//		while(1)
//		{
//		rt_event_recv(&sys_event, SYS_EVN_MOTION, RT_EVENT_FLAG_AND|RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved);

//			if(g_att_valid == 2)
//			{	
//			
//				Motion_Get_Position_2D(&motion.tracker.sense);
//				Motion_Get_Sensor(&motion.tracker.sense);
//				
//				get_front_bumper_info(&g_Bumper);
//				if( !bumper_flag )
//				{
//					Motion_Mag_Line_Test(&motion.tracker);
//					if( g_Bumper.left == 0)
//					{
//						set_velocity(0, 0);
//						bumper_flag = TRUE;
//					}
//				
//				}
//				Motion_Process_Motor_Speed();
//				update_motor_control();
//				
//				update_map_nav();
//			}
//		}
/*endof back to base test*/

/*mag sensor test*/
//while (1)
//{
//	rt_event_recv(&sys_event, SYS_EVN_MOTION, RT_EVENT_FLAG_AND|RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved);

//	if(g_att_valid == 2)
//	{	
//		mag_sensor_update();
//		printf("left: %d  right: %d\n", leftsensor_data, rightsensor_data);
//	}
//}

/*endof mag sensor test*/

/*sonar test*/
//while(1)
//{
//	rt_event_recv(&sys_event, SYS_EVN_MOTION, RT_EVENT_FLAG_AND|RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved);

//	if(g_att_valid == 2)
//	{	
//	
//		Motion_Get_Position_2D(&motion.tracker.sense);
//		Motion_Get_Sensor(&motion.tracker.sense);
//		
//		printf("left_dist: %.4f, right_dist: %.4f \n", motion.tracker.sense.sonar_l, motion.tracker.sense.sonar_r );
//		motion.tracker.line_vel = 0.0;
//		motion.tracker.angular_vel = 0;
//		
//		Motion_Process_Motor_Speed();
//		update_motor_control();
//		
//		update_map_nav();
//	}
//}
/*endof sonar test*/

/*rotate angle test*/
//while(1)
//{
//	rt_event_recv(&sys_event, SYS_EVN_MOTION, RT_EVENT_FLAG_AND|RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved);

//	if(g_att_valid == 2)
//	{	
//	
//		Motion_Get_Position_2D(&motion.tracker.sense);
//		Motion_Get_Sensor(&motion.tracker.sense);
//		
//		if(!motion.tracker.path_imu.rotationFinished)
//			rotate_angle(&motion.tracker, 90, LEFT_STEERING);
//		else
//		{
//			motion.tracker.line_vel = 0.1;
//			motion.tracker.angular_vel = 0;
//			g_counter++;
//			if(g_counter>=100)
//			{
//				g_counter = 0;
//				motion.tracker.path_imu.rotationFinished = FALSE;
//			}
//		}
//		
//		
//		Motion_Process_Motor_Speed();
//		update_motor_control();
//		
//		update_map_nav();
//	}
//}
/*endof rotate angle test*/

		
/*deceleration and OA test*/
// extern T_bool g_obstacle_sensed;
// extern T_bool g_end_of_arc_reached;
// motion.zigzag.state = T_MOTION_ZIGZAG_STATE_IDLE;
// T_bool g_dec = TRUE;
// T_bool g_oa = FALSE;
// g_decision.params.bypass.pass_side = RIGHT_STEERING;
// while(1)
// {
// 	rt_event_recv(&sys_event, SYS_EVN_MOTION, RT_EVENT_FLAG_AND|RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved);

// 	if(g_att_valid == 2)
// 	{	
	
// 		Motion_Get_Position_2D(&motion.tracker.sense);
// 		Motion_Get_Sensor(&motion.tracker.sense);
		
// 		//deceleration test
// 		if(g_dec)
// 		{
// 			if(!g_obstacle_sensed)
// 			{
// 				motion.tracker.angular_vel = 0;
// 				printf("l_ori_v: %f ", motion.tracker.line_vel);
// 				motion.tracker.line_vel = tune_linear_vel();
// 				printf("l_cur_v: %f \n", motion.tracker.line_vel);
// 			}		
// 			else
// 			{
// //				rt_kprintf("oa dist reached.\n");
// 				set_velocity_hard(0, 0);
// 				g_obstacle_sensed = TRUE;
// 				g_dec = FALSE;
// 				g_oa = TRUE;
// 			}
// 		}
// //		OA test
// 		if(g_oa)
// 		{
// 			if(g_obstacle_sensed)
// 			{
// 				obstacle_avoidance(0.1, 0.25);
// 				//if(g_end_of_arc_reached)
// 				if(motion.zigzag.state == T_MOTION_ZIGZAG_STATE_LINE)    
// 				{
// 					set_velocity(0, 0);
// 					g_oa = FALSE;
// 				}
// 			}
// 			else
// 			{
// 				rt_kprintf("unassigned state[1]\n");
// 			}
// 		}
// //			
		
// 		Motion_Process_Motor_Speed();
// 		update_motor_control();
		
// 		update_map_nav();
// 	}
// }
/*endof deceleration and OA test*/
	while (1)
	{
		rt_event_recv(&sys_event, SYS_EVN_MOTION, RT_EVENT_FLAG_AND|RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved);
		
		
		if( RT_EOK == rt_mq_recv(&gp_app_mq, &g_decision, sizeof(g_decision), 0) )
			app_mtn_assigner();

		if(g_att_valid == 2)
		{
			Motion_Get_Position_2D(&motion.tracker.sense);
			Motion_Get_Sensor(&motion.tracker.sense);
			
			if(!g_pause_motion)
			{
				/*run state machine*/
				if(g_mtn_stage == RUN)
				{
					mtn_FSM();
				}
				
				/*processing bumps*/
				if(bump_check())
				{
					get_rt_decision();
					g_mtn_stage = BMPD;
				}
				
				if(g_mtn_stage == BMPD)
				{
					if(bk_after_bump()) // BACK FINISHED ?
					{
						rt_mtn_assigner();
					}
				}
				
				/*processing obstacles*/ //CAN'T MOVE TO THE OUTSIDE OF THIS IF-ELSE BLOCK!
				#ifdef ULTRASONIC_ON
					if(obst_check())
					{
						if(motion.motion_state != MOTION_STATE_WIRE) 
						{
							if( (motion.motion_state == MOTION_STATE_ZIGZAG) && ((motion.zigzag.state == T_MOTION_ZIGZAG_STATE_TURN) || (motion.zigzag.state == T_MOTION_ZIGZAG_STATE_BACK) ) )
							{
								g_os_stage = CLEAR;
							}
							else
							{
								get_os_decision();
								g_mtn_stage = OBST_SNSD;
							}
						}
					}
					
					if(g_mtn_stage == OBST_SNSD)
					{
						os_mtn_assigner();
					}
				#endif /*ULTRASONIC_ON*/
			}
			
			/*update motor control*/
			Motion_Process_Motor_Speed();
			update_motor_control();
			
			update_map_nav();			
		}
		
	#ifdef MOTION_CHECK_THREAD_TIME
		temp=(float)get_time_diff3(&MOTION_thread_time_check)*1e-3;
		rt_kprintf("\r\n..>>>:MOTION_thread_time_check_in = %d\r\n",(rt_uint32_t)temp);
	#endif /* end of MOTION_CHECK_THREAD_TIME*/		
	}
}

/*
static void mower_motion_circle(T_motion* motion,float line_speed, float angle_speed)
{
	float current_angle = 0;
	float tx = 1.0f;
	float ty = 0.0f;
	rt_uint32_t recved;
	uint8_t buf[100] = "hello!!!!!!!!!\n\r";
	while(1)
	{
		volatile float sin = sinf(current_angle);
		volatile float cos = cosf(current_angle);
		volatile float x = tx;
		volatile float y = ty;
		volatile float tx = x*cos-y*sin;
		volatile float ty = x*sin+y*cos;
		
		rt_event_recv(&sys_event, SYS_EVN_MOTION, RT_EVENT_FLAG_AND|RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved);
		
		//rt_debug(buf,16);
		Motion_Get_Position_2D(&motion->tracker.sense);
		Motion_Get_Sensor(&motion->tracker.sense);
		Motion_Update_2D_Angle(&motion->tracker,tx,ty,line_speed);
		Motion_Run_Tracker(&motion->tracker);
		Motion_Process_Motor_Speed();
		
		current_angle -= angle_speed;
		if(current_angle < 0)
		{
			current_angle += 2*3.1415926;
		}
		
		update_motor_control();
	}
}
*/