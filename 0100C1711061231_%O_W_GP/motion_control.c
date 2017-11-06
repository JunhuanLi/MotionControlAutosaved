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

/* macros *********************************************************************/
//vehicle parameters
float g_radius_left = 0.0915; //meter
float g_radius_right = 0.0915; //meter
float g_vehicle_width = 0.35; //meter
float g_wheel_omega_max = 6.98; //maximum angluar velocity of the wheels 3000/45(rpm) == 6.98(rad/s)
float g_vehicle_omega_max = 3.65; //maximum angular velocity of the vehicle 0.63837/0.35/2 ~= 3.65 rad/s 
	
T_frontBumper g_Bumper;
T_sonar g_mtn_sonar;

T_gp_trigger g_trigger;
T_gp_decision g_decision;
//T_gp_params g_decision.params;

int g_left_wheel_sign;
int g_right_wheel_sign;
T_motion motion;

#define BACK_TIME_CNT_B4_UTURN 100
#define BACK_LINEAR_VELOCITY_B4_UTURN -0.2
#define ONTO_WIRE_ROT_VEL 0.5
#define ONTO_WIRE_LINEAR_VEL 0.05
#define ZIGZAG_UTURN_LINEAR_VEL 0.2
#define OA_DECELERATION_DIST 1 // in meter
#define OA_AVOIDANCE_DIST 0.5
#define HEADING_CTRL_LINEAR_VEL 0.35
#define ARC_LINEAR_VEL 0.2
#define ARC_ANGULAR_VEL 0.5
#define OA_RADIUS 0.5

T_bool g_bumped = FALSE;
T_bool g_obstacle_sensed = FALSE;
T_bool g_obstacle_avoidance_finished = FALSE;
T_bool g_end_of_to_pose;
//T_bool g_pose_memed = FALSE;
uint16_t g_stage = 0;
float g_angle_cnt = 0;
T_bool g_end_of_arc_reached = FALSE;
extern unsigned short g_counter;

extern struct rt_messagequeue mt_mq; 
extern struct rt_messagequeue gp_mq; 

/*************************************************
  Function:       // out_station
  Description:    // linear velocity
  Input:          // None
  Output:         // none
  Others:         // none
*************************************************/
void out_station(float v)
{
	motion.tracker.line_vel = v;
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
/*************************************************
  Function:       // get_decision
  Description:    // get decision from global planner
  Input:          // current trigger container and decision container
  Output:         // none
  Others:         // none
*************************************************/
void get_decision(T_gp_trigger *trigger_ptr, T_gp_decision *decision_ptr)
{
	//(1)send the trigger to planner thread
	if(-RT_EFULL == rt_mq_send(&mt_mq, trigger_ptr, sizeof(*trigger_ptr)))
	{
		rt_kprintf("[ljh]mq is FULL, delay 1s.\n");
		rt_thread_delay(1000);
	}

	//(2)receive decision from planner thread
	if(RT_EOK == rt_mq_recv(&gp_mq, decision_ptr, sizeof(*decision_ptr), RT_WAITING_FOREVER))
	{
		//rt_kprintf("[ljh]recv action: %d.\n", decision_ptr->action);
	}
	//rt_thread_delay(10);

}
					
/*************************************************
  Function:       // processing_bump
  Description:    // process bumps
  Input:          // None
  Output:         // none
  Others:         // none
*************************************************/
void processing_bump()
{
	get_front_bumper_info(&g_Bumper);

	if(g_Bumper.left == 0 && !g_bumped)
	{
		g_bumped = TRUE;
		set_velocity_hard(&motion.tracker, 0, 0);
//		Motion_Process_Motor_Speed();
//		update_motor_control();
		g_trigger = RIGID_TOUCHED;
		get_decision(&g_trigger, &g_decision);
		action_params_print();

	}
	
	if(g_bumped)
	{
		set_velocity(&motion.tracker, -0.1, 0);
		g_counter++;
	}
}
/*************************************************
  Function:       // auto_arc
  Description:    // algorithm of running a arc
  Input:          // linear velocity angular velocity
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
void auto_arc(float linear_vel, float angular_vel)
{
	g_angle_cnt++;
	if(angular_vel*g_angle_cnt*0.015*57.3 >= 90)
		g_end_of_arc_reached = TRUE;
	else
	{
		if(g_decision.params.bypass.pass_side == LEFT_STEERING)
			set_velocity(&motion.tracker, linear_vel, angular_vel);
		else
			set_velocity(&motion.tracker, linear_vel, -angular_vel);
	}
}

/*************************************************
  Function:       // obstacle_avoidance
  Description:    // algorithm of avoiding obstacles
  Input:          // T_motion, linear velocity, angular velocity
  Output:         // none
  Others:         // none
*************************************************/
void obstacle_avoidance(float linear_vel, float angular_vel)
{
	if(g_stage == 0)
	{
		if(!motion.tracker.path_imu.rotationFinished) //rotate_angle(&motion.tracker, 90, LEFT_STEERING);
			rotate_angle(&motion.tracker, 90, g_decision.params.bypass.pass_side);
		else
		{
			g_stage++;
			motion.tracker.path_imu.rotationFinished = FALSE;
		}
	}
	
	if(g_stage == 1)
	{
		if(!g_end_of_arc_reached)
			auto_arc(linear_vel, angular_vel);
		else
		{
			g_angle_cnt = 0;
			g_stage++;
		}
	}
	
	if(g_stage == 2)
	{
		if(!motion.tracker.path_imu.rotationFinished) //rotate_angle(&motion.tracker, 90, LEFT_STEERING);
			rotate_angle(&motion.tracker, 90, g_decision.params.bypass.pass_side);
		else
		{
			g_stage++;
			motion.tracker.path_imu.rotationFinished = FALSE;
		}
	}
	
	if(g_stage == 3)
	{
		motion.zigzag.state = T_MOTION_ZIGZAG_STATE_LINE;
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
//												g_decision.params.bypass.pass_side, 
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
//			if(g_decision.params.bypass.pass_side == LEFT_STEERING)
//				rotate_vector(&motion->tracker, 
//												-motion->tracker.path_imu.oa_pre_pos_y, 
//												motion->tracker.path_imu.oa_pre_pos_x, 
//												g_decision.params.bypass.pass_side, 
//												0);
//			else 
//				rotate_vector(&motion->tracker, 
//												motion->tracker.path_imu.oa_pre_pos_y, 
//												-motion->tracker.path_imu.oa_pre_pos_x, 
//												g_decision.params.bypass.pass_side, 
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
	//return motion.tracker.sense.sonar_l < motion.tracker.sense.sonar_r ? motion.tracker.sense.sonar_l : motion.tracker.sense.sonar_r;
	return OA_DECELERATION_DIST;
}

/*************************************************
  Function:       // tune_linear_vel
  Description:    // tune linear velocity according to the distance
  Input:          // void
  Output:         // tuned linear velocity
  Others:         // none
*************************************************/
float tune_linear_vel()
{
	float cur_vel = 0;
	float tuned_vel = 0;
	float dist = 0;
	float prop = 0;
	
	cur_vel = motion.tracker.line_vel;
	dist = get_dist_from_sensors();
	prop = dist/OA_DECELERATION_DIST;
	
	if(dist <= OA_AVOIDANCE_DIST)
		g_obstacle_sensed = TRUE;
		
	if(prop >= 1)
		prop = 1;
	

	
	tuned_vel = prop * cur_vel;
	
	return tuned_vel;
}

void action_params_print()
{
	switch(g_decision.action)
	{
		case 0:
			rt_kprintf("[ljh]recv action: ACT_NULL\n");
			break;
		
		case 1:
			rt_kprintf("[ljh]recv action: TO_POSE, params: pos(%d mm, %d mm), vec(%d, %d)\n", 
									(int)(1000*g_decision.params.to_pose.pos[0]), (int)(1000*g_decision.params.to_pose.pos[1]), 
									(int)(1000*g_decision.params.to_pose.vec[0]), (int)(1000*g_decision.params.to_pose.vec[1]));
			break;
		
		case 2:
			rt_kprintf("[ljh]recv action: DIR_DRIVE, params: rot_side(%d), fin_vec(%d, %d)\n", 
									g_decision.params.dir_drive.rot_side, 
									(int)(1000*g_decision.params.dir_drive.fin_vec[0]), 
									(int)(1000*g_decision.params.dir_drive.fin_vec[1]));
			break;
		
		case 3:
			rt_kprintf("[ljh]recv action: U_TURN, params: turn_side(%d), fin_vec(%d, %d)\n", 
									g_decision.params.u_turn.turn_side, 
									(int)(1000*g_decision.params.u_turn.fin_vec[0]), (int)(1000*g_decision.params.u_turn.fin_vec[1]));
			break;
		
		case 4:
			rt_kprintf("[ljh]recv action: ONTO_WIRE, params: enter_side(%d), laps(%d), fin_vec(%d, %d)\n", 
									g_decision.params.onto_wire.enter_side, g_decision.params.onto_wire.laps, 
									(int)(1000*g_decision.params.onto_wire.fin_vec[0]), 
									(int)(1000*g_decision.params.onto_wire.fin_vec[1]));
			break;
									
		case 5:
			rt_kprintf("[ljh]recv action: BYPASS, params: pass_side(%d)\n", 
									g_decision.params.bypass.pass_side);
			break;
		
		default:
			rt_kprintf("[ljh]recv invalid action!\n");
			break;
	}
	rt_kprintf("\n");
	return;
}


//void change_zigzag_state()
//{
//	switch(g_decision.action)
//	{
//		case U_TURN:
//			motion.zigzag.state = T_MOTION_ZIGZAG_STATE_TURN;
//		break;
//		
//    case ONTO_WIRE:
//			motion.zigzag.state = T_MOTION_ZIGZAG_STATE_ONTOWIRE;
//		break;
//		
//		case TO_POSE:
//			motion.motion_state = MOTION_STATE_P2P;
//		break;
//		
//		default:
//			rt_kprintf("Invalid action. Can't complete change zigzag state!\n");
//		break;
//	}
//}

void heading_control()
{
	switch(g_decision.action)
	{
		case ONTO_WIRE:
			track_vector(&motion.tracker, g_decision.params.onto_wire.fin_vec[0], g_decision.params.onto_wire.fin_vec[1], HEADING_CTRL_LINEAR_VEL);
		break;
		
		case U_TURN:
			track_vector(&motion.tracker,g_decision.params.u_turn.fin_vec[0], g_decision.params.u_turn.fin_vec[1], HEADING_CTRL_LINEAR_VEL);	
		break;
		
		case DIR_DRIVE:
			if(g_end_of_to_pose)
			{
				if(!motion.tracker.path_imu.rotationFinished)
					rotate_vector(&motion.tracker, g_decision.params.dir_drive.fin_vec[0], g_decision.params.dir_drive.fin_vec[1],
												g_decision.params.dir_drive.rot_side, 0);
				else
				{
					motion.tracker.path_imu.rotationFinished = FALSE;
					g_end_of_to_pose = FALSE;
				}
			}
			else 
				track_vector(&motion.tracker, g_decision.params.dir_drive.fin_vec[0], g_decision.params.dir_drive.fin_vec[1], HEADING_CTRL_LINEAR_VEL);
		break;
			
		default:
			rt_kprintf("Invalid action. Can't complete heading control!\n");
		break;
	}
	/*OBS*/
//	if(g_obstacle_sensed)
//	{
//		g_trigger = OBST_SENSED;
//		get_decision(&g_trigger, &g_decision);
//		action_params_print(g_decision.action, g_decision.params);
//	}
//	else
		motion.tracker.line_vel = tune_linear_vel();
}

void mtn_onto_wire(T_gp_side enterSide, float rot_vel, float linear_vel)
{
	if(motion.tracker.sense.side_l == motion.tracker.sense.side_r)
  {
    if(enterSide == LEFT_STEERING)
    {
      motion.tracker.line_vel = linear_vel;
      motion.tracker.angular_vel = rot_vel;
      motion.tracker.path_mag_line.dir = MOTION_MAG_LINE_DIRECT;
    }
    else if(enterSide == RIGHT_STEERING)
    {
      motion.tracker.line_vel = linear_vel;
      motion.tracker.angular_vel = -rot_vel;
      motion.tracker.path_mag_line.dir = MOTION_MAG_LINE_REVERSE;
    }
  }
  else
		motion.motion_state = MOTION_STATE_MAGLINE;
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
void motion_cover()
{
	if(motion.zigzag.state == T_MOTION_ZIGZAG_STATE_LINE)
	{
		mag_sensor_update();
		zigzag_flag++;
		
		if((mag_state.left_sensor_change == 1)&&(mag_state.right_sensor_change == 1)&&(zigzag_flag > 30))
		{	
			rt_kprintf("L\n");
			mag_state.right_sensor_change = 0;
			mag_state.left_sensor_change = 0;
			
			g_trigger = WIRE_SENSED;
			set_velocity_hard(&motion.tracker, 0 , 0);
			get_decision(&g_trigger, &g_decision);
      action_params_print();
			
			if(g_decision.action == ONTO_WIRE)
			{
				//change_zigzag_state();
				motion.zigzag.state = T_MOTION_ZIGZAG_STATE_ONTOWIRE;
				zigzag_flag = 0;
			}
			else if((g_decision.action == U_TURN) || (g_decision.action == TO_POSE)) 
				motion.zigzag.state = T_MOTION_ZIGZAG_STATE_BACK;
			else
				rt_kprintf("Action without assigned reactions[2].");
		}
		else 
		{
			heading_control();
			/*OBS*/
//			if(g_decision.action == BYPASS)
//			{
//				motion->zigzag.state = T_MOTION_ZIGZAG_STATE_BYPASS;
//			}
			
		}
			//clear the mag_sensor state
		mag_state.right_sensor_change = 0;
		mag_state.left_sensor_change = 0;
	}
	else if(motion.zigzag.state == T_MOTION_ZIGZAG_STATE_BACK)
	{
		mag_state.right_sensor_change = 0;
		mag_state.left_sensor_change = 0;
		
		g_counter++;
		if(g_counter < BACK_TIME_CNT_B4_UTURN)
			set_velocity(&motion.tracker, BACK_LINEAR_VELOCITY_B4_UTURN, 0);
		else if(g_decision.action == U_TURN)
		{
			//change_zigzag_state();
			motion.zigzag.state = T_MOTION_ZIGZAG_STATE_TURN;
			zigzag_flag = 0;
			g_counter = 0;
		}
		else if(g_decision.action == TO_POSE)
		{
			motion.motion_state = MOTION_STATE_P2P;
			zigzag_flag = 0;
			g_counter = 0;
		}
		else
			rt_kprintf("Action without assigned reactions[3].");
	}
	else if(motion.zigzag.state == T_MOTION_ZIGZAG_STATE_TURN)
	{
		zigzag_flag=0;
		if(!motion.tracker.path_imu.rotationFinished)
		{
			rotate_vector(&motion.tracker, g_decision.params.u_turn.fin_vec[0], 
									g_decision.params.u_turn.fin_vec[1] , g_decision.params.u_turn.turn_side, ZIGZAG_UTURN_LINEAR_VEL);
		}
		else
		{
			motion.tracker.path_imu.rotationFinished = FALSE;
			motion.zigzag.state = T_MOTION_ZIGZAG_STATE_LINE;
		}
			
	}
	else if(motion.zigzag.state == T_MOTION_ZIGZAG_STATE_ONTOWIRE)
	{
		mtn_onto_wire(g_decision.params.onto_wire.enter_side, ONTO_WIRE_ROT_VEL, ONTO_WIRE_LINEAR_VEL);
	}
		/*OBS*/
//	else if(motion->zigzag.state == T_MOTION_ZIGZAG_STATE_BYPASS)
//	{
//		obstacle_avoidance(ARC_LINEAR_VEL, ARC_ANGULAR_VEL );
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
			get_decision(&g_trigger, &g_decision);
			action_params_print();
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
				
		if(g_decision.params.u_turn.turn_side == RIGHT_STEERING) 
				{
					motion->zigzag.turn_dir = T_MOTION_ZIGZAG_TURN_CLOCKWISE; //turn right
				}
				else if(g_decision.params.u_turn.turn_side == LEFT_STEERING) 
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

void Motion_Process_Motor_Speed()
{
    
	float omega_leftWheel =  1/g_radius_left * motion.tracker.line_vel 
														+ g_vehicle_width/(2*g_radius_left) * motion.tracker.angular_vel;
	float omega_rightWheel = 1/g_radius_right * motion.tracker.line_vel
														- g_vehicle_width/(2*g_radius_right) * motion.tracker.angular_vel;
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
	if(2 == MOTOR_DRIVER_SELECT)
	{
		set_motor_control_speed_s32(omega_dr_leftWheel,omega_dr_rightWheel);
	}
	else if(2 == MOTOR_DRIVER_SELECT)
	{
		set_motor_speed_spi(omega_dr_leftWheel,omega_dr_rightWheel);
	}
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
