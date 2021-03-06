
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
static void mower_motion_square(T_motion* motion,float speed, uint32_t side_length)
{
	uint8_t state = 0;
	uint32_t count = 0;
	
	rt_uint32_t recved;
	
	Motion_Update_2D_Angle(&motion->tracker,1.0f,0.0f,speed);
	while(1)
	{
		rt_event_recv(&sys_event, SYS_EVN_MOTION, RT_EVENT_FLAG_AND|RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved);

		Motion_Get_Position_2D(&motion->tracker.sense);
		Motion_Get_Sensor(&motion->tracker.sense);
		
		if(state == 0 || state == 2 || state == 4 || state == 6)
		{
			if(count >= side_length)
			{
				state ++;
				count = 0;
			}
			count++;
			Motion_Run_Tracker(&motion->tracker);
		}
		else if(state == 1)
		{
			motion->tracker.line_vel = 0;
			motion->tracker.angular_vel = speed;
			
			if(motion->tracker.sense.dir_y < -0.96f)
			{
				Motion_Update_2D_Angle(&motion->tracker,0.0f,-1.0f,speed);
				state ++;
			}
		}
		else if(state == 3)
		{
			motion->tracker.line_vel = 0;
			motion->tracker.angular_vel = speed;
			
			if(motion->tracker.sense.dir_x < -0.96f)
			{
				Motion_Update_2D_Angle(&motion->tracker,-1.0f,0.0f,speed);
				state ++;
			}
		}
		else if(state == 5)
		{
			motion->tracker.line_vel = 0;
			motion->tracker.angular_vel = speed;
			
			if(motion->tracker.sense.dir_y > 0.96f)
			{
				Motion_Update_2D_Angle(&motion->tracker,0.0f,1.0f,speed);
				state ++;
			}
		}
		else if(state == 7)
		{
			motion->tracker.line_vel = 0;
			motion->tracker.angular_vel = speed;
			
			if(motion->tracker.sense.dir_x > 0.96f)
			{
				Motion_Update_2D_Angle(&motion->tracker,1.0f,0.0f,speed);
				state = 0;
			}
		}
		Motion_Process_Motor_Speed(motion);
		update_motor_control();
	}
}
const float DIR_POS_OFFSET = 0.96f;
//const float DIR_NEG_OFFSET = -DIR_POS_OFFSET;
const float DIR_NEG_OFFSET = 0.007f;
float vec0 = 0.0;
float vec1 = 0.0;

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



/*在分配堆栈空间时，必须要对其*/
ALIGN(RT_ALIGN_SIZE)
char thread_motion_stack[4096];
struct rt_thread thread_motion;
static void mower_motion_square_position(T_motion* motion,float speed, float side_length);

extern T_frontBumper g_Bumper;
extern T_trigger g_trigger;
T_bool bumper_flag = FALSE;

extern T_motion motion;
extern T_action g_action;
extern T_params_act g_action_params;
unsigned short OUT_STATION_FLAG = 0;
int state=0;

void mower_motion_thread(void* parameter)
{
  rt_uint32_t recved;

	mag_sensor_initial();
	
	rt_thread_delay(1000); //need be removed later
	Motion_Init(&motion,1);
	
	//motion.motion_state = MOTION_STATE_ZIGZAG;
	motion.motion_state = MOTION_STATE_OUT_STATION;
	motion.zigzag.state = T_MOTION_ZIGZAG_STATE_LINE;
	
	g_trigger = INIT_PROC;
	
	rt_kprintf("\r\n Thread Motion Initial...");
	
	while(is_att_valid < 2)
	{
	
	}
	rt_kprintf("\r\n is_att_valid ok...");
	make_decision(&g_trigger, &g_action, &g_action_params);
	action_params_print(g_action, g_action_params);
    /*test block*/
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
//       ontoWire(&motion, LEFT_STEERING);
//       rt_kprintf("INSIDE ONTO WIRE\n");
//    }
//		//setVelocity(&motion.tracker, 0.0, 0.0);
//		Motion_Process_Motor_Speed(&motion);
//		update_motor_control();
//		
//	}

    /*endof test block*/ 

		/*back to base test*/
//		while(1)
//		{
//		rt_event_recv(&sys_event, SYS_EVN_MOTION, RT_EVENT_FLAG_AND|RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved);

//			if(is_att_valid == 2)
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
//						motion.tracker.line_vel = 0;
//						motion.tracker.angular_vel = 0;
//						bumper_flag = TRUE;
//					}
//				
//				}
//				Motion_Process_Motor_Speed(&motion);
//				update_motor_control();
//				
//				//global planner record
//				//record_track(); //move to mag_test
//				update_map_nav();
//				//Debug
//				//rt_kprintf("angle = %d                x = %d                 y = %d \n\r",(int)(eul_rad[0]*10*57.3), (int)(motion.tracker.sense.pos_x*100), (int)(motion.tracker.sense.pos_y*100));
//				//rt_kprintf("left = %d, right =%d \n\r",(int)(motion.tracker.line_vel),(int)(motion.tracker.angular_vel));
//				//rt_kprintf("left = %d, right = %d \r\n\r", leftsensor_data,rightsensor_data);
//				
//				//rt_event_send(&sys_event,SYS_EVN_MOTOR);
//				//	rt_exit_critical();
//			}
//		}
/*endof back to base test*/

/*mag sensor test*/
//	while (1)
//	{
//		rt_event_recv(&sys_event, SYS_EVN_MOTION, RT_EVENT_FLAG_AND|RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved);

//		if(is_att_valid == 2)
//		{	
//			mag_sensor_update();
//		}
//	}

/*endof mag sensor test*/
	while (1)
	{
		rt_event_recv(&sys_event, SYS_EVN_MOTION, RT_EVENT_FLAG_AND|RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved);

		if(is_att_valid == 2)
		{	
		
			Motion_Get_Position_2D(&motion.tracker.sense);
			Motion_Get_Sensor(&motion.tracker.sense);
			
			
			
			//Motion_Run_Tracker(&motion.tracker);
			if( motion.motion_state == MOTION_STATE_ZIGZAG )
			{
				//make_decision(&g_trigger, &g_action, &g_action_params);
				//params trans
				Motion_Run(&motion);
			}
			else if(motion.motion_state == MOTION_STATE_OUT_STATION)
			{
				motion.tracker.line_vel = -0.10;
				OUT_STATION_FLAG ++;						//when outing station
				if(OUT_STATION_FLAG > 150)
				{
					/*lijunhjuan 20171012 changed cuz ontowire*/
					if(!motion.tracker.path_imu.rotationFinished)
						rotateVector(&motion.tracker, -1, 0, RIGHT_STEERING);
					else
					{
						motion.motion_state = MOTION_STATE_MAGLINE;
						motion.tracker.path_imu.rotationFinished = FALSE;
						OUT_STATION_FLAG = 0;
					}
				}
			}
			else if(motion.motion_state == MOTION_STATE_MAGLINE)
			{
				motion.tracker.target_vel = 0.3;
				motion.tracker.line_vel = 0.3;
				
				Motion_Mag_Line_Test(&motion.tracker);
				
				record_track(); // global_planner_record 
				//Motion_Run_Mag_Line(&motion.tracker);
				//bumper issue
				get_front_bumper_info(&g_Bumper);
				if(g_Bumper.left == 0)
				{
					
					g_trigger = RIGID_TOUCHED;
					OUT_STATION_FLAG = 0;
					setVelocity(&motion.tracker, 0.0, 0.0);
					Motion_Process_Motor_Speed(&motion);
					update_motor_control();
					make_decision(&g_trigger, &g_action, &g_action_params);
					action_params_print(g_action, g_action_params);
					vec0 = g_action_params.to_pose_.pos[0]-motion.tracker.sense.pos_x;
					vec1 = g_action_params.to_pose_.pos[1]-motion.tracker.sense.pos_y;
					
					//rt_kprintf("RT\n");
					
					//motion.motion_state = MOTION_STATE_P2P;
				}
				
				if(g_trigger == RIGID_TOUCHED)
				{
					motion.tracker.line_vel = -0.10;
					OUT_STATION_FLAG++;
					if(OUT_STATION_FLAG>100)
					{
						if(!motion.tracker.path_imu.rotationFinished)
						{
							rotateVector(&motion.tracker, vec0, vec1, LEFT_STEERING);
						}
						else
						{
							motion.tracker.path_imu.rotationFinished = FALSE;
							OUT_STATION_FLAG =0 ;
							motion.motion_state = MOTION_STATE_P2P;
						}
					}
				}
					
			}
			else if(motion.motion_state == MOTION_STATE_P2P)
			{
				if(!motion.tracker.path_imu.pointReached)
				{
					trackPoint(&motion.tracker, g_action_params.to_pose_.pos[0], g_action_params.to_pose_.pos[1]);
				}
				else
				{
					motion.tracker.path_imu.pointReached = FALSE;
					
					//rt_kprintf("c_p(%d,%d)\n", (int)(motion.tracker.sense.pos_x*1000), (int)(motion.tracker.sense.pos_y*1000));
					
					g_trigger = POS_REACHED;
					make_decision(&g_trigger, &g_action, &g_action_params);
					action_params_print(g_action, g_action_params);
					/*action params print*/
					
					//rt_kprintf("g_action = %d\n", g_action);
				//rt_kprintf("n_p(%d,%d)", (int)(g_action_params.to_pose_.pos[0]*1000), (int)(g_action_params.to_pose_.pos[1]*1000)); 
					
					if(g_action == DIR_DRIVE)
					{
//						rt_kprintf("DIR_DRIVE ON\n");
//						rt_kprintf("n_dir(%d,%d)", (int)(g_action_params.dir_drive_.fin_vec[0]*1000), (int)(g_action_params.dir_drive_.fin_vec[1]*1000) ); hw_us_delay(600);
//						rt_kprintf("c_dir(%d,%d)\n", (int)(motion.tracker.sense.dir_x*1000), (int)(motion.tracker.sense.dir_y*1000)); hw_us_delay(600);

						motion.motion_state = MOTION_STATE_ZIGZAG;
//						g_action_params.u_turn_.fin_vec[0] = g_action_params.dir_drive_.fin_vec[0];
//						g_action_params.u_turn_.fin_vec[1] = g_action_params.dir_drive_.fin_vec[1];
//						g_action_params.u_turn_.turn_side = g_action_params.dir_drive_.rot_side;
					}
					else if(g_action != TO_POSE)
					{
						rt_kprintf("\nerrp2p w/ a=%d\n", g_action);
					}
				}
			}
			
			//Update Motor Command
			Motion_Process_Motor_Speed(&motion);
			update_motor_control();
			
			
			//global planner record
			//record_track(); //move to mag_test
			update_map_nav();
			//Debug
			//rt_kprintf("angle = %d                x = %d                 y = %d \n\r",(int)(eul_rad[0]*10*57.3), (int)(motion.tracker.sense.pos_x*100), (int)(motion.tracker.sense.pos_y*100));
			//rt_kprintf("left = %d, right =%d \n\r",(int)(motion.tracker.line_vel),(int)(motion.tracker.angular_vel));
			//rt_kprintf("left = %d, right = %d \r\n\r", leftsensor_data,rightsensor_data);
			
			//rt_event_send(&sys_event,SYS_EVN_MOTOR);
			//	rt_exit_critical();
		}
	}
}

/*Turn to mag*/
static void mower_motion_square_position(T_motion* motion,float speed, float side_length)
{
	uint8_t state = 0;
	uint32_t count = 0;
	float point_x = 0;
	float point_y = 0;
	
	rt_uint32_t recved;
	
	point_x = side_length;
	point_y = 0.0f;
	Motion_Update_2D_Line(&motion->tracker,point_x,point_y,1.0f,0.0f,speed);
	Motion_Process_Motor_Speed(motion);
	update_motor_control();
	motion->zigzag.state = T_MOTION_ZIGZAG_STATE_LINE;
	rt_thread_delay(500);
	while(1)
	{
		rt_event_recv(&sys_event, SYS_EVN_MOTION, RT_EVENT_FLAG_AND|RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved);

		/*
		mag_sensor_update();
		if((mag_state.right_sensor_change == 1)&&(mag_state.left_sensor_change == 1))
		{
			//g_trigger = WIRE_SENSED;
			motion->zigzag.state = T_MOTION_ZIGZAG_STATE_TURN;
			
			mag_state.right_sensor_change = 0;
			mag_state.left_sensor_change = 0;
		}
		*/
		
		if(motion->zigzag.state == T_MOTION_ZIGZAG_STATE_TURN)
		{
			motion->tracker.line_vel = 0;
			motion->tracker.angular_vel = speed;
			//motion->zigzag.state = T_MOTION_ZIGZAG_STATE_LINE;
			
			
			Motion_Get_Position_2D(&motion->tracker.sense);
			Motion_Get_Sensor(&motion->tracker.sense);
			Motion_Update_2D_Line(&motion->tracker,point_x,point_y,0.0f,0.0f,speed);
			
			if(motion->tracker.sense.dir_x < -0.96)
			{
				motion->zigzag.state = T_MOTION_ZIGZAG_STATE_LINE;
			}
		}
		
		if(motion->zigzag.state == T_MOTION_ZIGZAG_STATE_LINE)
		{
			motion->tracker.line_vel = speed;
			motion->tracker.angular_vel = 0;
			Motion_Update_2D_Line(&motion->tracker,point_x,point_y,0.0f,0.0f,speed);	
		}
		
		
		/*Motor Speed update*/
		Motion_Process_Motor_Speed(motion);
		update_motor_control();
	}
}








