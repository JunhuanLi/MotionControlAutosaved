
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
//#include <string.h>
#include <stdio.h>
#include "mower_common.h"
#include "motion_control.h"
#include "global.h"
#include "motor_control.h"
#include "motion_mag_line.h"

#include "usart_driver.h"
#include "lcd12864_io_spi.h"
#include "bumper_front.h"

#include "global_planner.h"

/*********Temp Functions******************/
#define BACK_LINEAR_VEL -0.1
extern T_bool g_end_of_to_pose;
unsigned short g_counter = 0;


/*在分配堆栈空间时，必须要对其*/
ALIGN(RT_ALIGN_SIZE)
char thread_motion_stack[4096];
struct rt_thread thread_motion;
extern T_frontBumper g_Bumper;
extern T_gp_trigger g_trigger;
extern T_motion motion;
extern T_gp_action g_action;
extern T_gp_params g_action_params;
extern rt_uint8_t start_move;
/************************************/

void mower_motion_thread(void* parameter)
{
  rt_uint32_t recved;

	mag_sensor_initial();
	
	rt_thread_delay(1000); //need be removed later
	Motion_Init(&motion,1);
	
	motion.motion_state = MOTION_STATE_OUT_STATION;
	motion.zigzag.state = T_MOTION_ZIGZAG_STATE_LINE;
	
	g_trigger = INIT_PROC;
	
	rt_kprintf("\r\n Thread Motion Initializing...");
	
	rt_kprintf("\r\n g_att_valid == %d", g_att_valid);
	while(g_att_valid < 2)
	{
		
		hw_ms_delay(1);
	}
	rt_kprintf("\r\n g_att_valid ok...");hw_ms_delay(1);
	
	rt_kprintf("\r\n wait for button press...");
	while(1 == start_move)
	{
		
	}
	rt_kprintf("\r\n button pressed, motion control start...");
	
	make_decision(&g_trigger, &g_action, &g_action_params);
	action_params_print();
    /*onto wire test block*/
//    g_action = ONTO_WIRE;
//    g_action_params.onto_wire_.fin_vec[0] = 1.0;
//    g_action_params.onto_wire_.fin_vec[1] = 0.0;
    
//	while(1)
//	{
//		rt_event_recv(&sys_event, SYS_EVN_MOTION, RT_EVENT_FLAG_AND|RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved);
//		Motion_Get_Position_2D(&motion.tracker.sense);
//		Motion_Get_Sensor(&motion.tracker.sense);


//    if(motion.motion_state == MOTION_STATE_MAGLINE)
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
//		//set_velocity(&motion.tracker, 0.0, 0.0);
//		Motion_Process_Motor_Speed(&motion);
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
//						set_velocity(&motion.tracker, 0, 0);
//						bumper_flag = TRUE;
//					}
//				
//				}
//				Motion_Process_Motor_Speed(&motion);
//				update_motor_control();
//				
//				update_map_nav();
//			}
//		}
/*endof back to base test*/

/*mag sensor test*/
//	while (1)
//	{
//		rt_event_recv(&sys_event, SYS_EVN_MOTION, RT_EVENT_FLAG_AND|RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved);

//		if(g_att_valid == 2)
//		{	
//			mag_sensor_update();
//		}
//	}

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
//		
//			rt_kprintf("left_dist: %d, right_dist: %d \n", 
//								(int)(1000*motion.tracker.sense.sonar_l), (int)(1000*motion.tracker.sense.sonar_r) );
//		
//		Motion_Process_Motor_Speed(&motion);
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
//			motion.tracker.line_vel = 0.2;
//			motion.tracker.angular_vel = 0;
//			g_counter++;
//			if(g_counter>=200)
//			{
//				g_counter = 0;
//				motion.tracker.path_imu.rotationFinished = FALSE;
//			}
//		}
//		
//		
//		Motion_Process_Motor_Speed(&motion);
//		update_motor_control();
//		
//		update_map_nav();
//	}
//}
/*endof rotate angle test*/

		
/*deceleration and OA test*/
//extern T_bool g_obstacle_sensed;
//extern T_bool g_end_of_arc_reached;
//motion.zigzag.state = T_MOTION_ZIGZAG_STATE_IDLE;
//while(1)
//{
//	rt_event_recv(&sys_event, SYS_EVN_MOTION, RT_EVENT_FLAG_AND|RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved);

//	if(g_att_valid == 2)
//	{	
//	
//		Motion_Get_Position_2D(&motion.tracker.sense);
//		Motion_Get_Sensor(&motion.tracker.sense);
//		
////		deceleration test
////			if(!g_obstacle_sensed)
////			{
////				motion.tracker.angular_vel = 0;
////				motion.tracker.line_vel = tune_linear_vel(&motion);
////			}		
////			else
////				set_velocity(&motion.tracker,0, 0);
////		
//		
////		OA test
////			if(g_obstacle_sensed)
////			{
////				obstacle_avoidance(0.1, 0.25);
////				if(g_end_of_arc_reached)
////      //if(motion.zigzag.state == T_MOTION_ZIGZAG_STATE_LINE)    
////					set_velocity(&motion.tracker, 0, 0);
////			}
////			
//			
//		Motion_Process_Motor_Speed(&motion);
//		update_motor_control();
//		
//		update_map_nav();
//	}
//}
/*endof deceleration and OA test*/
	while (1)
	{
		rt_event_recv(&sys_event, SYS_EVN_MOTION, RT_EVENT_FLAG_AND|RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved);

		if(g_att_valid == 2)
		{	
			Motion_Get_Position_2D(&motion.tracker.sense);
			Motion_Get_Sensor(&motion.tracker.sense);

			if(motion.motion_state == MOTION_STATE_OUT_STATION)
			{
				motion.tracker.line_vel = BACK_LINEAR_VEL;
				g_counter ++;						//when outing station
				if(g_counter > 150)
				{
					if(!motion.tracker.path_imu.rotationFinished)
						rotate_vector(&motion.tracker, -1, 0, RIGHT_STEERING, 0);
					else
					{
						motion.motion_state = MOTION_STATE_MAGLINE;
						motion.tracker.path_imu.rotationFinished = FALSE;
						g_counter = 0;
					}
				}
			}
			else if( motion.motion_state == MOTION_STATE_ZIGZAG )
			{
				motion_cover();
			}
			else if(motion.motion_state == MOTION_STATE_MAGLINE)
			{
				Motion_Mag_Line_Test(&motion.tracker);
				
				record_track(); // global_planner_record 
				
				processing_bump();
				
				if(g_counter>100 && g_Bumper.bumper_state)
				{
					if(!motion.tracker.path_imu.rotationFinished)
					{
						rotate_vector(&motion.tracker, g_action_params.to_pose_.pos[0]-motion.tracker.sense.pos_x, 
																												 g_action_params.to_pose_.pos[1]-motion.tracker.sense.pos_y, 
																												 LEFT_STEERING, 0);
					}
					else
					{
						motion.tracker.path_imu.rotationFinished = FALSE;
						g_counter =0 ;
						g_Bumper.bumper_state = 0;
						motion.motion_state = MOTION_STATE_P2P;
					}
				}
//				get_front_bumper_info(&g_Bumper);
//				if(g_Bumper.left == 0)
//				{
//					g_trigger = RIGID_TOUCHED;
//					set_velocity(&motion.tracker, 0, 0);
//					Motion_Process_Motor_Speed(&motion);
//					update_motor_control();
//					make_decision(&g_trigger, &g_action, &g_action_params);
//					action_params_print();
//				}
//				
//				if(g_trigger == RIGID_TOUCHED)
//				{
//					motion.tracker.line_vel = -0.1;
//					g_counter++;
//					if(g_counter>100)
//					{
//						if(!motion.tracker.path_imu.rotationFinished)
//						{
//							rotate_vector(&motion.tracker, g_action_params.to_pose_.pos[0]-motion.tracker.sense.pos_x, 
//                                                           g_action_params.to_pose_.pos[1]-motion.tracker.sense.pos_y, 
//                                                           LEFT_STEERING, 0);
//						}
//						else
//						{
//							motion.tracker.path_imu.rotationFinished = FALSE;
//							g_counter =0 ;
//							motion.motion_state = MOTION_STATE_P2P;
//						}
//					}
//				}
					
			}
			else if(motion.motion_state == MOTION_STATE_P2P)
			{
				if(!motion.tracker.path_imu.pointReached)
				{
					track_point(&motion.tracker, g_action_params.to_pose_.pos[0], g_action_params.to_pose_.pos[1]);
				}
				else
				{
					motion.tracker.path_imu.pointReached = FALSE;

					g_trigger = POS_REACHED;
					make_decision(&g_trigger, &g_action, &g_action_params);
					action_params_print();

					if(g_action == DIR_DRIVE)
					{
						g_end_of_to_pose = TRUE;
						motion.motion_state = MOTION_STATE_ZIGZAG;
					}
					else if(g_action != TO_POSE)
					{
						
					}
				}
			}
			
			//Update Motor Command
			Motion_Process_Motor_Speed(&motion);
			update_motor_control();
			
			update_map_nav();
		}
	#ifdef MOTION_CHECK_THREAD_TIME
		temp=(float)get_time_diff3(&MOTION_thread_time_check)*1e-3;
		rt_kprintf("\r\n..>>>:MOTION_thread_time_check_in = %d\r\n",(rt_uint32_t)temp);
	#endif /* end of MOTION_CHECK_THREAD_TIME*/		
	}
}


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
		Motion_Process_Motor_Speed(motion);
		
		current_angle -= angle_speed;
		if(current_angle < 0)
		{
			current_angle += 2*3.1415926;
		}
		
		update_motor_control();
	}
}
