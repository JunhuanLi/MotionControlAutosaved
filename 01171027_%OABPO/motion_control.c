/*******************************************************************************

  Copyright (C), 2017-2027, TOPBAND Co., Ltd.
  File name 	: pi.h
  Author		: George 	
  Version		: V1.0.0	  
  Date			: 2017/07/21
  Description	: pi 
  
  History:		  
				  
	1. Date 		:
	   Author		:
	   Modification :
	2. ...
	
*******************************************************************************/

/* includes *******************************************************************/
#include "motion_control.h"
#include "motor_control.h"
#include "motion_math.h"
#include "bumper_front.h"

#include "usart_driver.h"
#include "motion_imu.h"
#include "global_planner.h"
/* macros *********************************************************************/
//motor parameters
float g_radius_left = 0.0915; //meter
float g_radius_right = 0.0915; //meter
float g_vehicle_width = 0.35; //meter
float g_wheel_omega_max = 6.98; //maximum angluar velocity of the wheels 3000/45(rpm) == 6.98(rad/s)
float g_vehicle_omega_max = 3.65; //maximum angular velocity of the vehicle 0.63837/0.35/2 ~= 3.65 rad/s 
	
T_frontBumper g_Bumper;
T_trigger g_trigger;
int g_left_wheel_sign;
int g_right_wheel_sign;

T_motion motion;
T_action g_action;
T_params_act g_action_params;

#define ONTO_WIRE_ROT_VEL 0.5
#define ZIGZAG_UTURN_LINEAR_VEL 0.2
#define OA_DECELERATION_DIST 1 // in meter
#define HEADING_CTRL_LINEAR_VEL 0.35
#define ARC_LINEAR_VEL 0.2
#define ARC_ANGULAR_VEL 0.5
#define OA_RADIUS 0.5;
T_bool g_obstacle_sensed = FALSE;
T_bool g_obstacle_avoidance_finished = FALSE;
T_bool g_end_of_to_pose;
//T_bool g_pose_memed = FALSE;
uint16_t g_stage = 0;
float g_angle_cnt = 0;
T_bool g_end_of_arc_reached = FALSE;
extern unsigned short BACK_TIME_COUNT;

/*************************************************
  Function:       // auto_arc
  Description:    // algorithm of running a arc
  Input:          // arc radius,linear velocity and T_motion
  Output:         // none
  Others:         // none
*************************************************/
//void auto_arc(T_motion* motion, float radius, float linear_vel)
//{
//	float dist = sqrtf( (motion->tracker.sense.pos_x - motion->tracker.path_imu.oa_arc_center_x)
//							 * (motion->tracker.sense.pos_x - motion->tracker.path_imu.oa_arc_center_x)
//							 + (motion->tracker.sense.pos_y - motion->tracker.path_imu.oa_arc_center_y)
//							 * (motion->tracker.sense.pos_y - motion->tracker.path_imu.oa_arc_center_y) );
//	float dist_err = radius - dist;
//	set_velocity(&motion->tracker, linear_vel, -dist_err);
//}
void auto_arc(T_motion* motion, float linear_vel, float angular_vel)
{
	g_angle_cnt++;
	if(g_angle_cnt*0.02*57.3 > 90)
		g_end_of_arc_reached = TRUE;
	else
	{
		if(g_action_params.bypass_.pass_side == LEFT_STEERING)
			set_velocity(&motion->tracker, linear_vel, angular_vel);
		else
			set_velocity(&motion->tracker, linear_vel, -angular_vel);
	}
}

/*************************************************
  Function:       // obstacle_avoidance
  Description:    // algorithm of avoiding obstacles
  Input:          // arc radius and T_motion
  Output:         // none
  Others:         // none
*************************************************/
void obstacle_avoidance(T_motion* motion, float linear_vel, float angluar_vel)
{
	if(g_stage == 0)
	{
		if(!motion->tracker.path_imu.rotationFinished)
			rotate_angle(&motion->tracker, 90, g_action_params.bypass_.pass_side);
		else
		{
			g_stage++;
			motion->tracker.path_imu.rotationFinished = FALSE;
		}
	}
	
	if(g_stage == 1)
	{
		if(!g_end_of_arc_reached)
			auto_arc(motion, ARC_LINEAR_VEL, ARC_ANGULAR_VEL);
		else
		{
			g_angle_cnt = 0;
			g_stage++;
		}
	}
	
	if(g_stage == 2)
	{
		if(!motion->tracker.path_imu.rotationFinished)
			rotate_angle(&motion->tracker, 90, g_action_params.bypass_.pass_side);
		else
		{
			g_stage++;
			motion->tracker.path_imu.rotationFinished = FALSE;
		}
	}
	
	if(g_stage == 3)
	{
		motion->zigzag.state = T_MOTION_ZIGZAG_STATE_LINE;
		g_stage = 0;
		g_end_of_arc_reached = FALSE;
	}
}
//void obstacle_avoidance(T_motion* motion, float radius)
//{
//	/*OBS*/
//	//记录当前位置，方向
//	if(!g_pose_memed)
//	{
//		motion->tracker.path_imu.oa_pre_pos_x = motion->tracker.sense.pos_x;
//		motion->tracker.path_imu.oa_pre_pos_y = motion->tracker.sense.pos_y;
//		motion->tracker.path_imu.oa_pre_dir_x = motion->tracker.sense.dir_x;
//		motion->tracker.path_imu.oa_pre_dir_y = motion->tracker.sense.dir_y;
//	
//		float tmp_theta = atan2f(motion->tracker.sense.dir_x, motion->tracker.sense.dir_y);
//		motion->tracker.path_imu.oa_arc_center_x = cos(tmp_theta)*radius - sin(tmp_theta)*0;
//		motion->tracker.path_imu.oa_arc_center_y = sin(tmp_theta)*radius + cos(tmp_theta)*0;
//		
//		g_pose_memed = TRUE;
//	}
//	//绕行完毕，恢复绕行前状态
//	if(g_obstacle_avoidance_finished)
//	{
//		if(!motion->tracker.path_imu.rotationFinished)
//			rotate_vector(&motion->tracker, 
//												motion->tracker.path_imu.oa_pre_dir_x, 
//												motion->tracker.path_imu.oa_pre_dir_y, 
//												g_action_params.bypass_.pass_side, 
//												0.05);
//		else
//		{
//			motion->zigzag.state = T_MOTION_ZIGZAG_STATE_LINE;
//			g_obstacle_avoidance_finished = FALSE;
//			g_pose_memed = FALSE;
//		}
//	}
//	//绕行
//	else
//	{
//		//转90度
//		if(!motion->tracker.path_imu.rotationFinished)
//		{
//			if(g_action_params.bypass_.pass_side == LEFT_STEERING)
//				rotate_vector(&motion->tracker, 
//												-motion->tracker.path_imu.oa_pre_pos_y, 
//												motion->tracker.path_imu.oa_pre_pos_x, 
//												g_action_params.bypass_.pass_side, 
//												0);
//			else 
//				rotate_vector(&motion->tracker, 
//												motion->tracker.path_imu.oa_pre_pos_y, 
//												-motion->tracker.path_imu.oa_pre_pos_x, 
//												g_action_params.bypass_.pass_side, 
//												0);
//		}
//		else
//		{	
//			float dist = sqrtf( (motion->tracker.sense.pos_x - 2*motion->tracker.path_imu.oa_arc_center_x)
//							 * (motion->tracker.sense.pos_x - 2*motion->tracker.path_imu.oa_arc_center_x)
//							 + (motion->tracker.sense.pos_y - 2*motion->tracker.path_imu.oa_arc_center_y)
//							 * (motion->tracker.sense.pos_y - 2*motion->tracker.path_imu.oa_arc_center_y) );
//			//绕行完毕条件满足？
//			if(dist < 0.2)
//			{
//				g_obstacle_avoidance_finished = TRUE;
//				motion->tracker.path_imu.rotationFinished = FALSE;
//			}
//			else 
//				auto_arc(motion, ARC_LINEAR_VEL, ACR_ANGULAR_VEL);
//		}
//	}
//	
//}

/*************************************************
  Function:       // get_dist_from_sensors
  Description:    // get distance information from electromagnetic sensor and proximity sensor
  Input:          // void
  Output:         // distance from vehicle to obstacle or wire
  Others:         // none
*************************************************/
float get_dist_from_sensors()
{
	/*
	//mag part
	
	//ultrosonic part
	
	
	return dist;
	*/
	
	return OA_DECELERATION_DIST;
}

/*************************************************
  Function:       // tune_linear_vel
  Description:    // tune linear velocity according to the distance
  Input:          // void
  Output:         // tuned linear velocity
  Others:         // none
*************************************************/
float tune_linear_vel(T_motion* motion)
{
	float cur_vel = 0;
	float tuned_vel = 0;
	float dist = 0;
	float prop = 0;
	
	cur_vel = motion->tracker.line_vel;
	dist = get_dist_from_sensors();
	prop = dist/OA_DECELERATION_DIST;
	
	if(prop >= 1)
		prop = 1;
	
	if(prop <= 0.25)
		g_obstacle_sensed = TRUE;
	
	tuned_vel = prop * cur_vel;
	
	return tuned_vel;
}

void action_params_print(T_action g_action, T_params_act g_action_params)
{
	switch(g_action)
	{
		case 0:
			rt_kprintf("\naction: TO_POSE, params: pos(%d mm, %d mm), vec(%d, %d)\n", 
									(int)(1000*g_action_params.to_pose_.pos[0]), (int)(1000*g_action_params.to_pose_.pos[1]), 
									(int)(1000*g_action_params.to_pose_.vec[0]), (int)(1000*g_action_params.to_pose_.vec[1]));
			break;
		
		case 1:
			rt_kprintf("\naction: DIR_DRIVE, params: rot_side(%d), fin_vec(%d, %d)\n", 
									g_action_params.dir_drive_.rot_side, 
									(int)(1000*g_action_params.dir_drive_.fin_vec[0]), (int)(1000*g_action_params.dir_drive_.fin_vec[1]));
			break;
		
		case 2:
			rt_kprintf("\naction: U_TURN, params: turn_side(%d), fin_vec(%d, %d)\n", 
									g_action_params.u_turn_.turn_side, 
									(int)(1000*g_action_params.u_turn_.fin_vec[0]), (int)(1000*g_action_params.u_turn_.fin_vec[1]));
			break;
		
		case 3:
			rt_kprintf("\naction: ONTO_WIRE, params: enter_side(%d), laps(%d), fin_vec(%d, %d)\n", 
									g_action_params.onto_wire_.enter_side, g_action_params.onto_wire_.laps, 
									(int)(1000*g_action_params.onto_wire_.fin_vec[0]), (int)(1000*g_action_params.onto_wire_.fin_vec[1]));
			break;
		
		default:
			rt_kprintf("invalid action!");
			break;
	}
	
	return;
}

void change_zigzag_state(T_motion* motion,T_action g_action)
{
	switch(g_action)
	{
		case U_TURN:
			motion->zigzag.state = T_MOTION_ZIGZAG_STATE_TURN;
		break;
		
    case ONTO_WIRE:
			motion->zigzag.state = T_MOTION_ZIGZAG_STATE_ONTOWIRE;
		break;
		
		case TO_POSE:
			motion->motion_state = MOTION_STATE_P2P;
		break;
		
		default:
			rt_kprintf("Invalid action. Can't complete change zigzag state!\n");
		break;
	}
}

void heading_control(T_motion* motion, T_action g_action, T_params_act g_action_params)
{
	switch(g_action)
	{
		case ONTO_WIRE:
			track_vector(&motion->tracker, g_action_params.onto_wire_.fin_vec[0], g_action_params.onto_wire_.fin_vec[1], HEADING_CTRL_LINEAR_VEL );
		break;
		
		case U_TURN:
			track_vector(&motion->tracker,g_action_params.u_turn_.fin_vec[0], g_action_params.u_turn_.fin_vec[1], HEADING_CTRL_LINEAR_VEL );	
		break;
		
		case DIR_DRIVE:
			if(g_end_of_to_pose)
			{
				if(!motion->tracker.path_imu.rotationFinished)
					rotate_vector(&motion->tracker, g_action_params.dir_drive_.fin_vec[0], g_action_params.dir_drive_.fin_vec[1],
												g_action_params.dir_drive_.rot_side, 0);
				else
				{
					motion->tracker.path_imu.rotationFinished = FALSE;
					g_end_of_to_pose = FALSE;
				}
			}
			else 
				track_vector(&motion->tracker, g_action_params.dir_drive_.fin_vec[0], g_action_params.dir_drive_.fin_vec[1], HEADING_CTRL_LINEAR_VEL);
		break;
			
		default:
			rt_kprintf("Invalid action. Can't complete heading control!\n");
		break;
	}
	/*OBS*/
//	if(g_obstacle_sensed)
//	{
//		g_trigger = OBS_SENSED;
//		make_decision(&g_trigger, &g_action, &g_action_params);
//		action_params_print(g_action, g_action_params);
//	}
//	else
		motion->tracker.line_vel = tune_linear_vel(motion);
}

void onto_wire(T_motion* motion, T_side enterSide, float rot_vel)
{
	if(motion->tracker.sense.side_l == motion->tracker.sense.side_r)
  {
    if(enterSide == LEFT_STEERING)
    {
      motion->tracker.line_vel = 0;
      motion->tracker.angular_vel = rot_vel;
    }
    else if(enterSide == RIGHT_STEERING)
    {
      motion->tracker.line_vel = 0;
      motion->tracker.angular_vel = -rot_vel;
    }
  }
  else
		motion->motion_state = MOTION_STATE_MAGLINE;
}

MAG_STATUE mag_state;
const unsigned char MAG_CNT = 3;
void mag_sensor_initial(void)
{
	unsigned char i = 0;
	
	mag_state.left_sensor_now = 0;
	mag_state.right_sensor_now = 0;
	
	mag_state.left_sensor_change = 0;
	mag_state.right_sensor_change = 0;
	
	for(i = 0; i < MAG_CNT; i++)
	{
		mag_state.left_sensor_old[i] = 0;
		mag_state.right_sensor_old[i] = 0;
	}
	
}

void mag_sensor_update(void)
{
	unsigned char i  = 0;
	
	if((leftsensor_data > 0)&&(UART_RX_BUFFER[0] == 0xAA)&&((UART_RX_BUFFER[1] == 0xBB)))
	{
		mag_state.left_sensor_now = MOTION_MAG_LINE_INSIDE;
		g_left_wheel_sign = 1; 
	}
	else if((leftsensor_data < 0)&&(UART_RX_BUFFER[0] == 0xAA)&&((UART_RX_BUFFER[1] == 0xBB)))
	{
		mag_state.left_sensor_now = MOTION_MAG_LINE_OUTSIDE;
		g_left_wheel_sign = -1;
	}
	else
	{
		if(g_left_wheel_sign == -1)
		{
			mag_state.left_sensor_now = MOTION_MAG_LINE_OUTSIDE;			
		}
		g_left_wheel_sign = 0;
	}
	
	if((rightsensor_data > 0)&&(UART_RX_BUFFER[0] == 0xAA)&&((UART_RX_BUFFER[1] == 0xBB)))
	{
		mag_state.right_sensor_now = MOTION_MAG_LINE_INSIDE;
		g_right_wheel_sign = 1;
	}
	else if((rightsensor_data < 0)&&(UART_RX_BUFFER[0] == 0xAA)&&((UART_RX_BUFFER[1] == 0xBB)))
	{
		mag_state.right_sensor_now = MOTION_MAG_LINE_OUTSIDE;
		g_right_wheel_sign = -1;
	}
	else
	{
		if(g_right_wheel_sign==-1)
		{
			mag_state.right_sensor_now = MOTION_MAG_LINE_OUTSIDE;
		}
		g_right_wheel_sign = 0;
	}

	if(mag_state.right_sensor_now == MOTION_MAG_LINE_OUTSIDE)
	{
		mag_state.right_sensor_change = 1;
	}
	if(mag_state.left_sensor_now == MOTION_MAG_LINE_OUTSIDE)
	{
		mag_state.left_sensor_change = 1;
	}
	
	
	for(i = 0; i < MAG_CNT; i++)
	{
		mag_state.right_sensor_old[i] = mag_state.right_sensor_old[i+1];
		mag_state.left_sensor_old[i] = mag_state.left_sensor_old[i+1];
	}
	mag_state.right_sensor_old[MAG_CNT-1] = mag_state.right_sensor_now;
	mag_state.left_sensor_old[MAG_CNT-1] = mag_state.left_sensor_now;
}

static void Motion_Norm_2D(float* x, float* y)
{
	float length = sqrtf((*x)*(*x) + (*y)*(*y));
	if(length != 0)
	{
		*x = *x / length;
		*y = *y / length;
	}
	else
		while(1);
}

unsigned short zigzag_flag = 0;
static void motion_cover(T_motion* motion)
{
	if(motion->zigzag.state == T_MOTION_ZIGZAG_STATE_LINE)
	{
		mag_sensor_update();
		zigzag_flag++;
		
		if((mag_state.left_sensor_change == 1)&&(mag_state.right_sensor_change == 1)&&(zigzag_flag > 150))
		{		
			g_trigger = WIRE_SENSED;
			make_decision(&g_trigger, &g_action, &g_action_params);
      action_params_print(g_action, g_action_params);
			
			motion->zigzag.state = T_MOTION_ZIGZAG_STATE_BACK;
		}
		else 
		{
			heading_control(motion, g_action, g_action_params);
			/*OBS*/
//			if(g_action == OBS_SENSED)
//			{
//				motion->zigzag.state = T_MOTION_ZIGZAG_STATE_BYPASS;
//			}
			
		}
			//clear the mag_sensor state
		mag_state.right_sensor_change = 0;
		mag_state.left_sensor_change = 0;
	}
	else if(motion->zigzag.state == T_MOTION_ZIGZAG_STATE_BACK)
	{
		BACK_TIME_COUNT++;
		if(BACK_TIME_COUNT < 25)
			set_velocity(&motion->tracker, -0.1, 0);
		else
		{
			change_zigzag_state(motion, g_action);
			zigzag_flag = 0;
			BACK_TIME_COUNT = 0;
		}
	}
	else if(motion->zigzag.state == T_MOTION_ZIGZAG_STATE_TURN)
	{
		zigzag_flag=0;
		if(!motion->tracker.path_imu.rotationFinished)
		{
			rotate_vector(&motion->tracker, g_action_params.u_turn_.fin_vec[0], 
									g_action_params.u_turn_.fin_vec[1] , g_action_params.u_turn_.turn_side, ZIGZAG_UTURN_LINEAR_VEL);
		}
		else
		{
			motion->tracker.path_imu.rotationFinished = FALSE;
			motion->zigzag.state = T_MOTION_ZIGZAG_STATE_LINE;
		}
			
	}
	else if(motion->zigzag.state == T_MOTION_ZIGZAG_STATE_ONTOWIRE)
	{
		onto_wire(motion, g_action_params.onto_wire_.enter_side, ONTO_WIRE_ROT_VEL);
	}
		/*OBS*/
//	else if(motion->zigzag.state == T_MOTION_ZIGZAG_STATE_BYPASS)
//	{
//		obstacle_avoidance(motion, OA_RADIUS);
//	}
}

static void Motion_Run_Zigzag(T_motion* motion)
{
	if(motion->zigzag.state == T_MOTION_ZIGZAG_STATE_IDLE)
	{
		rt_kprintf("Waiting orders! \n");
		return;
	}
	else if(motion->zigzag.state == T_MOTION_ZIGZAG_STATE_LINE)
	{
		float dot_product = (motion->tracker.sense.dir_x* motion->zigzag.heading_x)+(motion->tracker.sense.dir_y* motion->zigzag.heading_y);
		//运行位姿控制器
		//Motion_Update_2D_Line(&motion->tracker, TEST_LEN, 0.0f, direction[0].x, direction[0].y, 0.1);
		Motion_Run_Tracker(&motion->tracker);
		
		mag_sensor_update();
		zigzag_flag++;

		

		if((mag_state.left_sensor_change == 1)&&(mag_state.right_sensor_change == 1)&&(zigzag_flag > 150)&&(dot_product > 0.96))
		{
			motion->zigzag.state = T_MOTION_ZIGZAG_STATE_TURN;
			
			mag_state.right_sensor_change = 0;
			mag_state.left_sensor_change = 0;
			
			zigzag_flag = 0;
			
			g_trigger = WIRE_SENSED;
			make_decision(&g_trigger, &g_action, &g_action_params);
			action_params_print(g_action, g_action_params);
		}
		if( ( mag_state.left_sensor_change == 0) && (mag_state.right_sensor_change == 0) )
		{
			//motion->zigzag.heading_x = motion->tracker.sense.dir_x;
			//motion->zigzag.heading_y = motion->tracker.sense.dir_y;
		}
	}
	//掉头部分
	else if(motion->zigzag.state == T_MOTION_ZIGZAG_STATE_TURN)
	{
		
		//float dot_product = (motion->tracker.sense.dir_x*motion->zigzag.heading_x)+(motion->tracker.sense.dir_y*motion->zigzag.heading_y);
		float dot_product = (motion->tracker.sense.dir_x* motion->zigzag.heading_x)+(motion->tracker.sense.dir_y* motion->zigzag.heading_y);
		float k = (1- motion->zigzag.blade_bodywidth_ratio * motion->zigzag.blade_overlaping_ratio) / 2;
		
		zigzag_flag = 0;
		//计算线速度和角速度
				
		if(g_action_params.u_turn_.turn_side == RIGHT_STEERING) 
				{
					motion->zigzag.turn_dir = T_MOTION_ZIGZAG_TURN_CLOCKWISE; //turn right
				}
				else if(g_action_params.u_turn_.turn_side == LEFT_STEERING) 
				{
					motion->zigzag.turn_dir = T_MOTION_ZIGZAG_TURN_COUNTERCLOCKWISE; // turn left
				}
				else
				{
					motion->zigzag.turn_dir = motion->zigzag.turn_dir;
				}		
				
		if(motion->zigzag.turn_dir == T_MOTION_ZIGZAG_TURN_COUNTERCLOCKWISE)
		{
			motion->tracker.line_vel = motion->zigzag.target_vel;
			motion->tracker.angular_vel = motion->zigzag.target_vel/(1-k);
			//motion->tracker.line_vel = 0;
		}
		else
		{
			motion->tracker.line_vel = motion->zigzag.target_vel;
			motion->tracker.angular_vel = -motion->zigzag.target_vel/(1-k);
			//motion->tracker.line_vel = 0;
		}
		
		//利用点乘判断是否完成掉头 之后计算下一阶段参数


		if( (dot_product) < -0.985f)  //~170degree
			{
				//turn finished, need to toggle the next turn direction
				/*
				motion->zigzag.turn_dir += 1;
				if(motion->zigzag.turn_dir>2)
					motion->zigzag.turn_dir=1;
				*/

				motion->zigzag.state = T_MOTION_ZIGZAG_STATE_LINE;
				motion->tracker.angular_vel = 0;
				motion->tracker.line_vel = 0;
				
				
				
				//if(motion->zigzag.turn_dir == T_MOTION_ZIGZAG_TURN_COUNTERCLOCKWISE) motion->zigzag.turn_dir = T_MOTION_ZIGZAG_TURN_CLOCKWISE;
				//if(motion->zigzag.turn_dir == T_MOTION_ZIGZAG_TURN_CLOCKWISE) motion->zigzag.turn_dir = T_MOTION_ZIGZAG_TURN_COUNTERCLOCKWISE;

				motion->zigzag.heading_x = -motion->zigzag.heading_x;
				//motion->tracker.sense.dir_x = (- motion->tracker.sense.dir_x);
				Motion_Update_2D_Line(&motion->tracker, (motion->zigzag.heading_x) * 200, 0, (motion->zigzag.heading_x),0, 500.0f);

			}
			
			//Tracking_Start_2D_Angle(&motion->tracker,motion->zigzag.heading_x,motion->zigzag.heading_y,motion->zigzag.target_vel);
		
		
	}
	else if(motion->zigzag.state == T_MOTION_ZIGZAG_STATE_EXCEPTION)
	{
		
	}

}
/* funcitons ******************************************************************/
void Motion_Start_2D_Angle(T_motion_tracker* obj,float dir_x,float dir_y,float vel)
{
	Motion_Norm_2D(&dir_x,&dir_y);
	obj->tracking														= MOTION_TRACKING_2D_ANGLE;
	obj->path_imu.dir_x 										= dir_x;
	obj->path_imu.dir_y 										= dir_y;	
	obj->path_imu.dir_x_adj									= dir_x;
	obj->path_imu.dir_y_adj									= dir_y;
	obj->command_vel 												= vel;
	obj->target_vel													= vel;
//	obj->path_imu.direction_pi.integral			= 0;
}
void Motion_Init(T_motion* motion,uint8_t en)
{
	set_motor_control_type(MOTOR_CONTROL_TYPE_SPEED);
	
	motion->motion_state = 											MOTION_STATE_IDLE;
	motion->exception = 												MOTION_EXCEPTION_NONE;
	motion->enable = 														en;
	
	motion->tracker.tracking = 									MOTION_TRACKING_2D_LINE;
	motion->tracker.acc = 											MOTION_ACC;
	motion->tracker.omega_leftWheel = 					0;
	motion->tracker.omega_rightWheel	= 				0;
	
	//obj->error = TRACKING_ERROR_NONE;
	//obj->sensor.mag_polarity = 1;
	
	Motion_Set_Path_Param(&motion->tracker,0.4);
	Motion_Set_cornerAngle_Param(&motion->tracker,0.01,0.001f,0.2);
	Motion_Set_lineAngle_Param(&motion->tracker,0.005, 0.001f,0.1);
	Motion_Set_Mag_Tracking_Param(&motion->tracker,  0.0001f, 0, 0);
	Motion_Set_Mag_Gotoline_Param(&motion->tracker,0,0,0);
	
	Motion_Update_2D_Line(&motion->tracker,1,0,1,0,0.1);
	
//	Motion_Set_Mag_Tracking_Param(&motion->tracker,0,0,0);
//	Motion_Set_Mag_Gotoline_Param(&motion->tracker,0,0,0);
//	Motion_Update_2D_Angle(&motion->tracker,1,0,0);
	Motion_Zigzag_Init(motion,0.8,1.0);
}

void Motion_Run(T_motion* motion)
{
	motion_cover(motion);
}

/*
void Motion_Process_Obstacle(T_motion* motion)
{
	//Run Obsticle
	float vel_sonar = motion->tracker.command_vel;
	float vel_mag_line = motion->tracker.command_vel;
	uint16_t sonar = motion->tracker.sense.sonar_l > motion->tracker.sense.sonar_r ? motion->tracker.sense.sonar_l : motion->tracker.sense.sonar_r;
	
	if(sonar > DECELERATION_SONAR_MIN)
	{
		vel_sonar = (float)(1 - (sonar - DECELERATION_SONAR_MIN))/(float)(DECELERATION_SONAR_MAX - DECELERATION_SONAR_MIN) * (float)(1 -DECELERATION_MIN_SPEED)
		           + (float)DECELERATION_MIN_SPEED;
		vel_sonar = vel_sonar * motion->tracker.command_vel;
	}
	
	motion->tracker.target_vel = vel_sonar < vel_mag_line ? vel_sonar : vel_mag_line;
}
*/

void Motion_Process_Motor_Speed(T_motion* motion)
{
    
	float omega_leftWheel =  1/g_radius_left * motion->tracker.line_vel 
														+ g_vehicle_width/(2*g_radius_left) * motion->tracker.angular_vel;
	float omega_rightWheel = 1/g_radius_right * motion->tracker.line_vel
														- g_vehicle_width/(2*g_radius_right) * motion->tracker.angular_vel;
	float omega_dr_leftWheel=0.0;
	float omega_dr_rightWheel=0.0;
	
	//Run Accellation
//	if(motion->tracker.omega_leftWheel < omega_leftWheel)
//	{
//		motion->tracker.omega_leftWheel += motion->tracker.acc;
//		if(motion->tracker.omega_leftWheel > omega_leftWheel)
//		{
//			motion->tracker.omega_leftWheel = omega_leftWheel;
//		}
//	}
//	else if(motion->tracker.omega_leftWheel > omega_leftWheel)
//	{
//		motion->tracker.omega_leftWheel -= motion->tracker.acc;
//		if(motion->tracker.omega_leftWheel < omega_leftWheel)
//		{
//			motion->tracker.omega_leftWheel = omega_leftWheel;
//		}
//	}
//	
//	if(motion->tracker.omega_rightWheel < omega_rightWheel)
//	{
//		motion->tracker.omega_rightWheel += motion->tracker.acc;
//		if(motion->tracker.omega_rightWheel > omega_rightWheel)
//		{
//			motion->tracker.omega_rightWheel = omega_rightWheel;
//		}
//	}
//	else if(motion->tracker.omega_rightWheel > omega_rightWheel)
//	{
//		motion->tracker.omega_rightWheel -= motion->tracker.acc;
//		if(motion->tracker.omega_rightWheel < omega_rightWheel)
//		{
//			motion->tracker.omega_rightWheel = omega_rightWheel;
//		}
//	}
//	
		
	omega_dr_leftWheel = 3600 * omega_leftWheel/g_wheel_omega_max; // dr == dutyratio
	omega_dr_rightWheel = 3600 * omega_rightWheel/g_wheel_omega_max;
	//Set Speed
	set_motor_control_speed_s32(omega_dr_leftWheel,omega_dr_rightWheel);
}


void Motion_Zigzag_Init(T_motion* motion,float blade_bodywidth_ratio,float blade_overlaping_ratio)
{
	motion->zigzag.blade_bodywidth_ratio =   		blade_bodywidth_ratio;
	motion->zigzag.blade_overlaping_ratio =  		blade_overlaping_ratio;
	motion->zigzag.state = 									    T_MOTION_ZIGZAG_STATE_IDLE;
	motion->zigzag.f_r = 												T_MOTION_ZIGZAG_GO_FOWARD;
	motion->zigzag.state = 											T_MOTION_ZIGZAG_STATE_LINE;
}

void Motion_Zigzag_Update(T_motion* motion,float speed,float heading_x,float heading_y,T_motion_zigzag_turn_dir_type turn_dir)
{
	float x = 																	heading_x;
	float y =                             			heading_y;

	Motion_Norm_2D(&x,&y);
	
	motion->zigzag.heading_x = 									x;
	motion->zigzag.heading_y = 									y;
	motion->zigzag.turn_dir = 									turn_dir;
	motion->zigzag.f_r = 												T_MOTION_ZIGZAG_GO_FOWARD;
	motion->zigzag.target_vel = 								speed;
	motion->motion_state = 											MOTION_STATE_ZIGZAG;
	
	//Motion_Update_2D_Angle(&motion->tracker, motion->zigzag.heading_x, motion->zigzag.heading_y, motion->zigzag.target_vel);
}


void Motion_Zigzag_Start(T_motion* motion,float speed,float heading_x,float heading_y,T_motion_zigzag_turn_dir_type turn_dir)
{
	float x = heading_x,y = heading_y;

	Motion_Norm_2D(&x,&y);
	
	motion->zigzag.heading_x = 							x;
	motion->zigzag.heading_y = 							y;
	motion->zigzag.turn_dir = 							turn_dir;
	motion->zigzag.f_r = 										T_MOTION_ZIGZAG_GO_FOWARD;
	motion->zigzag.target_vel = 						speed;
	motion->zigzag.state = 									T_MOTION_ZIGZAG_STATE_LINE;
	motion->motion_state = 									MOTION_STATE_ZIGZAG;
	
	Motion_Start_2D_Angle(&motion->tracker,motion->zigzag.heading_x,motion->zigzag.heading_y,motion->zigzag.target_vel);
}
