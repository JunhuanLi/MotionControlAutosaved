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
#define TEST_LEN 10.0f
T_bool DABUG_TEMP_FLAG = FALSE;
//DABUG
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

float Rl = 0.0915; //meter
float Rr = 0.0915; //meter
float l = 0.35; //meter
float omega_max = 6.98; //maximum angluar velocity 3000/45(rpm) == 6.98(rad/s)

/*Temp Mag**********************************************************************/
/*Mag Sensor*/
T_frontBumper g_Bumper;
T_trigger g_trigger;


T_motion motion;
T_action g_action;
T_params_act g_action_params;
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
int left_wheel_sign;
int right_wheel_sign;
void mag_sensor_update(void)
{
//	if((leftsensor_data<=0) && (rightsensor_data<=0))
//	{
//		mag_state.right_sensor_change = 1;
//  	mag_state.left_sensor_change = 1;
//	}
	
	unsigned char i  = 0;
	
	if((leftsensor_data > 0)&&(UART_RX_BUFFER[0] == 0xAA)&&((UART_RX_BUFFER[1] == 0xBB)))
	{
		mag_state.left_sensor_now = MOTION_MAG_LINE_INSIDE;
		left_wheel_sign = 1; 
	}
	else if((leftsensor_data < 0)&&(UART_RX_BUFFER[0] == 0xAA)&&((UART_RX_BUFFER[1] == 0xBB)))
	{
		mag_state.left_sensor_now = MOTION_MAG_LINE_OUTSIDE;
		left_wheel_sign = -1;
	}
	else
	{
		if(left_wheel_sign == -1)
		{
			mag_state.left_sensor_now = MOTION_MAG_LINE_OUTSIDE;
			
			//rt_kprintf("mgl=0\n");
			
		}
		l ./eft_wheel_sign = 0;
		//mag_state.left_sensor_now = MOTION_MAG_LINE_OUTSIDE;
		//mag_state.left_sensor_now = MOTION_MAG_LINE_MISSING;
	}
	
	if((rightsensor_data > 0)&&(UART_RX_BUFFER[0] == 0xAA)&&((UART_RX_BUFFER[1] == 0xBB)))
	{
		mag_state.right_sensor_now = MOTION_MAG_LINE_INSIDE;
		right_wheel_sign = 1;
	}
	else if((rightsensor_data < 0)&&(UART_RX_BUFFER[0] == 0xAA)&&((UART_RX_BUFFER[1] == 0xBB)))
	{
		mag_state.right_sensor_now = MOTION_MAG_LINE_OUTSIDE;
		right_wheel_sign = -1;
	}
	else
	{
		if(right_wheel_sign==-1)
		{
			mag_state.right_sensor_now = MOTION_MAG_LINE_OUTSIDE;
			
			//rt_kprintf("mgr=0\n");
			
		}
		right_wheel_sign = 0;
		//mag_state.right_sensor_now = MOTION_MAG_LINE_OUTSIDE;
		//mag_state.right_sensor_now = MOTION_MAG_LINE_MISSING;
		
		//rt_kprintf("mag_sensor_data:(%d,%d)\n", leftsensor_data, rightsensor_data);
	}
	
	/*
	if((mag_state.right_sensor_now == MOTION_MAG_LINE_OUTSIDE)&&(mag_state.left_sensor_now == MOTION_MAG_LINE_OUTSIDE))
	{
		if((mag_state.right_sensor_old[0] == MOTION_MAG_LINE_OUTSIDE)&&(mag_state.right_sensor_old[1] == MOTION_MAG_LINE_OUTSIDE)&& (mag_state.right_sensor_old[2] == MOTION_MAG_LINE_OUTSIDE) )//&&(mag_state.right_sensor_old[3] == MOTION_MAG_LINE_OUTSIDE))//&&(mag_state.right_sensor_old[4] == MOTION_MAG_LINE_OUTSIDE))
		{
			mag_state.right_sensor_change = 1;
		}
		if((mag_state.left_sensor_old[0] == MOTION_MAG_LINE_OUTSIDE)&&(mag_state.left_sensor_old[1] == MOTION_MAG_LINE_OUTSIDE)&&(mag_state.left_sensor_old[2] == MOTION_MAG_LINE_OUTSIDE))//&&(mag_state.left_sensor_old[3] == MOTION_MAG_LINE_OUTSIDE))//&&(mag_state.left_sensor_old[4] == MOTION_MAG_LINE_OUTSIDE))
		{
			mag_state.left_sensor_change = 1;
		}
	}
	*/
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
/*Temp end*********************************************************************/


/* static variables ***********************************************************/

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

void ontoWire(T_motion* motion, T_side enterSide)
{
    if(motion->tracker.sense.side_l == motion->tracker.sense.side_r)
    {
        if(enterSide == LEFT_STEERING)
        {
            motion->tracker.line_vel = 0.0;
            motion->tracker.angular_vel = 0.4;
        }
        else if(enterSide == RIGHT_STEERING)
        {
            motion->tracker.line_vel = 0.0;
            motion->tracker.angular_vel = -0.4;
        }
    }
    else
    {
        motion->motion_state = MOTION_STATE_MAGLINE;
    }
}

unsigned short zigzag_flag = 0;

static void Cover(T_motion* motion)
{
	if(motion->zigzag.state == T_MOTION_ZIGZAG_STATE_LINE)
	{
		mag_sensor_update();
		
		zigzag_flag++;
		
//		if(zigzag_flag >150)
//			rt_kprintf("\n zzflag150  ");
//		if(mag_state.left_sensor_change == 1)
//		{
//			rt_kprintf("left sensor changed   ");
//			if(mag_state.right_sensor_change != 1)
//				rt_kprintf("right value=%d\n", (int16_t)motion->tracker.sense.value_r);
//		}
//		if(mag_state.right_sensor_change == 1)
//		{
//			rt_kprintf("right sensor changed   ");
//			if(mag_state.left_sensor_change != 1)
//				rt_kprintf("left value=%d\n", (int16_t)motion->tracker.sense.value_l);
//		}
		
		if((mag_state.left_sensor_change == 1)&&(mag_state.right_sensor_change == 1)&&(zigzag_flag > 150))
		{
			//motion->zigzag.state = T_MOTION_ZIGZAG_STATE_TURN;
			zigzag_flag = 0;
			setVelocity(&motion->tracker, 0.0, 0.0);
			g_trigger = WIRE_SENSED;
			
			//rt_kprintf("WS\n");
			
			make_decision(&g_trigger, &g_action, &g_action_params);
      action_params_print(g_action, g_action_params);
			
//			rt_kprintf("T w/ a= %d \n",g_action);      
//			rt_kprintf("w / t_dir(%d,%d) ", (int)(g_action_params.u_turn_.fin_vec[0]*1000),(int)(g_action_params.u_turn_.fin_vec[1]*1000) );
//      rt_kprintf("and c_dir(%d,%d)\n", (int)(motion->tracker.sense.dir_x*1000), (int)(motion->tracker.sense.dir_y*1000));
//			
			if(g_action == U_TURN)
      {
          motion->zigzag.state = T_MOTION_ZIGZAG_STATE_TURN;
      }
      else if(g_action == ONTO_WIRE)
      {
				motion->zigzag.state = T_MOTION_ZIGZAG_STATE_ONTOWIRE;
      }
			else if(g_action == TO_POSE)
			{
				//DA BUG
				motion->motion_state = MOTION_STATE_P2P;
				
//				rt_kprintf("p2p\n");
//				rt_kprintf("c_p(%d,%d)\n", (int)(motion->tracker.sense.pos_x*1000), (int)(motion->tracker.sense.pos_y*1000));
//			
			}
			else
				rt_kprintf("erroor DD/BP");
		
		}	
		else if(g_action == ONTO_WIRE)
		{
			trackVector(&motion->tracker, g_action_params.onto_wire_.fin_vec[0], g_action_params.onto_wire_.fin_vec[1] );
			//rt_kprintf("DOWN WIRE FROM OW\n");
		}
		else if(g_action == U_TURN)
		{
			trackVector(&motion->tracker,g_action_params.u_turn_.fin_vec[0], g_action_params.u_turn_.fin_vec[1] );	
		}
		else if( (g_action == DIR_DRIVE) && (!DABUG_TEMP_FLAG )) //****** + DABUG_KEYIXIA_FLAG
		{
			if(!motion->tracker.path_imu.rotationFinished)
				rotateVector(&motion->tracker, g_action_params.dir_drive_.fin_vec[0], g_action_params.dir_drive_.fin_vec[1], g_action_params.dir_drive_.rot_side);
			else
			{
				motion->tracker.path_imu.rotationFinished = FALSE;
				DABUG_TEMP_FLAG = TRUE;
				motion->tracker.line_vel = 0.2;
				
				//rt_kprintf("a=DD,dirreached\n");
			
			}
			
		}
		else if(g_action == DIR_DRIVE)
		{
			//rt_kprintf("DDzigzag finished\n");
			trackVector(&motion->tracker, g_action_params.dir_drive_.fin_vec[0], g_action_params.dir_drive_.fin_vec[1]);
		}
		else
		{
			rt_kprintf("errzigzag w/ a=%d\n", g_action);
		}
			//clear the mag_sensor state
		mag_state.right_sensor_change = 0;
		mag_state.left_sensor_change = 0;
	}
	else if(motion->zigzag.state == T_MOTION_ZIGZAG_STATE_TURN)
	{
		zigzag_flag=0;
		if(!motion->tracker.path_imu.rotationFinished)
		{
			rotateVector(&motion->tracker, g_action_params.u_turn_.fin_vec[0], 
									g_action_params.u_turn_.fin_vec[1] , g_action_params.u_turn_.turn_side);
			
			motion->tracker.line_vel = 0.2;//横向位移一个车身宽度时，线速度为0.08m/s
			
		}
		else
		{
			motion->tracker.path_imu.rotationFinished = FALSE;
			motion->zigzag.state = T_MOTION_ZIGZAG_STATE_LINE;
			
			//rt_kprintf("L\n");
			
			//stop(&motion->tracker);
		}
			
	}
	else if(motion->zigzag.state == T_MOTION_ZIGZAG_STATE_ONTOWIRE)
	{
		ontoWire(motion, g_action_params.onto_wire_.enter_side);
	}
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
		//Motion_Run_Zigzag(motion);
		Cover(motion);
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
    
	float omega_leftWheel =  1/Rl * motion->tracker.line_vel + l/(2*Rl) * motion->tracker.angular_vel;
	float omega_rightWheel = 1/Rr * motion->tracker.line_vel - l/(2*Rr) * motion->tracker.angular_vel;
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
		
	omega_dr_leftWheel = 3600 * omega_leftWheel/omega_max; // dr == dutyratio
	omega_dr_rightWheel = 3600 * omega_rightWheel/omega_max;
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
