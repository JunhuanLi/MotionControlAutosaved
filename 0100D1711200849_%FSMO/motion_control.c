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
	
T_frontBumper g_Bumper;
T_sonar g_mtn_sonar;
T_gp_trigger g_trigger;
T_gp_decision g_decision;
T_motion motion;

T_motion_stage g_mtn_stage;
T_oa_stage g_oa_stage = FIRST_ROTATION;
T_bump_stage g_bump_stage = NOT_BMPD;
T_os_stage g_os_stage = CLEAR;

T_bool g_bumped = FALSE;
T_bool g_obstacle_sensed = FALSE;
T_bool g_obstacle_avoidance_finished = FALSE;
T_bool g_end_of_arc_reached = FALSE;
T_bool g_end_of_to_pose = FALSE;
T_bool g_dir_memd = FALSE;
T_bool g_pause_motion = FALSE;
T_bool g_back_to_work = FALSE;
T_bool g_d2w_abt_ut = FALSE;

int g_left_wheel_sign = 0;
int g_right_wheel_sign = 0;
int g_debouncing_counter = 0;
MAG_STATUE mag_state;
const unsigned char MAG_CNT = 3;
unsigned short g_lock_cnt_ut = 0;

/*************************************************
  Function:       // mtn_FSM
  Description:    // none
  Input:          // None
  Output:         // none
  Others:         // none
*************************************************/
void mtn_FSM()
{
	switch(motion.motion_state)
	{
		case MOTION_STATE_OUT_STATION:
			out_station(BACK_LNR_VEL_AFT_DPT);
		break;
		
		case MOTION_STATE_WIRE:
			mtn_wire_turn_n_track(&motion.tracker);
			record_track(); // global_planner_record 
		break;
			
		case MOTION_STATE_P2P:
			mtn_path_tracker();
		break;
			
		case MOTION_STATE_ZIGZAG:
			mtn_cover_FSM();
		break;
		
		default:
			#ifdef JHL_SERIAL_DEBUG
			rt_kprintf("Motion state becomes IDLE.\n");
			#endif
		break;	
	}
	
	get_tuned_vel();
}

/*************************************************
  Function:       // app_mtn_assigner
  Description:    // none
  Input:          // None
  Output:         // none
  Others:         // none
*************************************************/
void app_mtn_assigner()
{
	switch(g_decision.action)
	{
		case ACT_NULL:
			g_pause_motion = TRUE;
			set_velocity_hard(0, 0);
		break;
		
		case TO_POSE:
			motion.motion_state = MOTION_STATE_P2P;
			motion.zigzag.state = T_MOTION_ZIGZAG_STATE_NONE; // may not need jhli
			g_pause_motion = FALSE;
		break;
		
		case DEPART: // charging state jump out
			g_pause_motion = FALSE;
			g_back_to_work = 1;	
		break;
		
		default:
			g_pause_motion = FALSE;
		break;
	}
}

/*************************************************
  Function:       // or_mtn_assigner
  Description:    // none
  Input:          // None
  Output:         // none
  Others:         // none
*************************************************/
void or_mtn_assigner()
{
	switch(g_decision.action)
	{
		case TO_POSE:
			if(to_pose_r2tdir())
			{
				g_os_stage = CLEAR;
				g_mtn_stage = RUN;
				motion.motion_state = MOTION_STATE_P2P;
				motion.zigzag.state = T_MOTION_ZIGZAG_STATE_NONE; //may not need jhli
			}
		break;
		
		case ONTO_WIRE:
			g_os_stage = CLEAR;
			g_mtn_stage = RUN;
			mtn_onto_wire(g_decision.params.onto_wire.ntr_side, ROT_VEL_OW, LNR_VEL_OW);
		break;
		
		case DEPART:
			g_os_stage = CLEAR;
			g_mtn_stage = RUN;
			motion.motion_state = MOTION_STATE_OUT_STATION;
			motion.zigzag.state = T_MOTION_ZIGZAG_STATE_NONE; //may not need jhli
		break;
				
		case DIR_DRIVE:
			g_os_stage = CLEAR;
			g_mtn_stage = RUN;
			motion.motion_state = MOTION_STATE_ZIGZAG;
			motion.zigzag.state = T_MOTION_ZIGZAG_STATE_LINE;
		break;
		
		default:
			#ifdef JHL_SERIAL_DEBUG
			rt_kprintf("Invalid action rcvd in an_mtn_assnr.\n");
			#endif
		break;
	}
}
		
/*************************************************
  Function:       // os_mtn_assigner
  Description:    // none
  Input:          // None
  Output:         // none
  Others:         // none
*************************************************/
void os_mtn_assigner()
{
	if(g_os_stage == W8_RM)
	{
		switch(g_decision.action)
		{
			case ACT_NULL:// STOP FOR OBSTACLE, NEED TO RESUME
				if(get_dist_from_sensors() <= OA_RESUME_DIST)
					set_velocity_hard(0, 0);
				else
					get_or_decision();
			break;

			case DEPART:
				g_os_stage = CLEAR;
				g_mtn_stage = RUN;
				motion.motion_state = MOTION_STATE_OUT_STATION;
				motion.zigzag.state = T_MOTION_ZIGZAG_STATE_NONE; //may not need jhli
			break;
			
			case DIR_DRIVE:
				set_velocity(OS_BOT_VEL, 0); // CHECK!!!
			break;
			
			case ONTO_WIRE:
				set_velocity(0.2, 0);
			break;
			
			case U_TURN:
				g_os_stage = CLEAR;
				g_mtn_stage = RUN;
				g_counter = 0;
				motion.zigzag.state = T_MOTION_ZIGZAG_STATE_BACK;
			break;
			
			case BYPASS:
				mtn_obstacle_avoidance();
			break;
				
			default:
				#ifdef JHL_SERIAL_DEBUG
				rt_kprintf("Invalid action rcvd in os_mtn_assnr.\n");
				#endif
			break;
		}
	}
	else if(g_os_stage == OBS_RMD)
		or_mtn_assigner();
}
/*************************************************
  Function:       // pr_mtn_assigner
  Description:    // none
  Input:          // None
  Output:         // none
  Others:         // none
*************************************************/
void pr_mtn_assigner()
{
	switch(g_decision.action)
	{
		case DIR_DRIVE:
		g_end_of_to_pose = TRUE;
		motion.motion_state = MOTION_STATE_ZIGZAG;
		motion.zigzag.state = T_MOTION_ZIGZAG_STATE_BACK; 
		break;
	
		case TO_POSE:
		break;
		
		default:
			#ifdef JHL_SERIAL_DEBUG
			rt_kprintf("Invalid action rcvd in pr_mtn_assnr.\n");
			#endif
		break;
	}
}
/*************************************************
  Function:       // dc_mtn_assigner
  Description:    // none
  Input:          // None
  Output:         // none
  Others:         // none
*************************************************/
void dc_mtn_assigner()
{
	switch(g_decision.action)
	{
		case ONTO_WIRE:
			if(g_decision.params.onto_wire.laps == 0)
			{
				mtn_onto_wire(g_decision.params.onto_wire.ntr_side, 
													ROT_VEL_OW, LNR_VEL_OW);
				#ifdef JHL_SERIAL_DEBUG
					//rt_kprintf("Received o2w after out_station.\n");
				#endif
			}
			else 
			{
				#ifdef JHL_SERIAL_DEBUG
					rt_kprintf("Error! Received o2w after out_station but with laps != 0.\n");
				#endif
			}
		break;
			
		case TO_POSE:
			if( to_pose_r2tdir() )
			{
				motion.tracker.path_imu.rotationFinished = FALSE;
				motion.motion_state = MOTION_STATE_P2P;
				motion.zigzag.state = T_MOTION_ZIGZAG_STATE_NONE; //may not need jhli
				#ifdef JHL_SERIAL_DEBUG
					rt_kprintf("Received p2p after out_station.\n");
				#endif
			}
		break;
			
		default:
			#ifdef JHL_SERIAL_DEBUG
			rt_kprintf("Wrong action or params received in out_station.\n");
			#endif
		break;
	}
}
/*************************************************
  Function:       // ws_mtn_assigner
  Description:    // none
  Input:          // None
  Output:         // none
  Others:         // none
*************************************************/
void ws_mtn_assigner()
{
	switch(g_decision.action)
	{
		case ONTO_WIRE:
			motion.zigzag.state = T_MOTION_ZIGZAG_STATE_ONTOWIRE;
			#ifdef JHL_SERIAL_DEBUG
			rt_kprintf("Entering ontowire mode.\n");
			#endif
			g_lock_cnt_ut = 0;
			g_counter = 0;
		break;
		
		case U_TURN:
			g_counter++;
			if(g_counter <= BACK_TCNT_B4_UT)
				set_velocity(BACK_LNR_VEL_B4_UT, 0);
			else
			{
				motion.zigzag.state = T_MOTION_ZIGZAG_STATE_TURN;
				g_lock_cnt_ut = 0;
				g_counter = 0;
			}
		break;
			
		case TO_POSE:
			if(to_pose_r2tdir())
			{
				motion.motion_state = MOTION_STATE_P2P;
				motion.zigzag.state = T_MOTION_ZIGZAG_STATE_NONE;
				g_lock_cnt_ut = 0;
				g_counter = 0;
			}
		break;
		
		case DIR_DRIVE:
			g_lock_cnt_ut = 0;
			if(dir_drive_r2tdir())
			{
				motion.motion_state = MOTION_STATE_ZIGZAG;
				motion.zigzag.state = T_MOTION_ZIGZAG_STATE_LINE; 
			}
		break;
		
		default:
			#ifdef JHL_SERIAL_DEBUG
			rt_kprintf("Invalid action rcvd in ws_mtn_assnr.\n");
			#endif
		break;
	}
}
/*************************************************
  Function:       // rt_mtn_assigner
  Description:    // none
  Input:          // None
  Output:         // none
  Others:         // none
*************************************************/
void rt_mtn_assigner()
{
	switch(g_decision.action)
	{
		case TO_POSE:
			if( to_pose_r2tdir() )
			{
					//motion.tracker.path_imu.rotationFinished = FALSE;
					g_counter = 0;
					g_bump_stage = NOT_BMPD;
					g_os_stage = CLEAR;
					g_mtn_stage = RUN;
					//rt_kprintf("going to p2p. g_bump_stage = %d \n", g_bump_stage);
					motion.motion_state = MOTION_STATE_P2P;
					motion.zigzag.state = T_MOTION_ZIGZAG_STATE_NONE; //may not need jhli
			}
		break;
		
		case DEPART:
			motion.motion_state = MOTION_STATE_OUT_STATION;
			motion.zigzag.state = T_MOTION_ZIGZAG_STATE_NONE; //may not need jhli
			g_counter = 0;
			g_bump_stage = NOT_BMPD;
			g_os_stage = CLEAR;
			g_mtn_stage = RUN;
		break;
		
		case U_TURN:
			g_counter = 0;
			g_mtn_stage = RUN;
			g_bump_stage = NOT_BMPD;
			g_os_stage = CLEAR;
			motion.zigzag.state = T_MOTION_ZIGZAG_STATE_TURN;
		break;
		
		default:
			#ifdef JHL_SERIAL_DEBUG
			rt_kprintf("Invalid action after bump.\n");
			#endif 
		break;
	}
}

/*************************************************
  Function:       // get_or_decision
  Description:    // get obstacle removed decisions
  Input:          // None
  Output:         // none
  Others:         // none
*************************************************/
void get_or_decision()
{
	if(g_os_stage == W8_RM)
	{
		g_trigger = OBST_REMOVED;
		get_decision(&g_trigger, &g_decision);
		action_params_print();
		
		g_os_stage = OBS_RMD;
	}
}

/*************************************************
  Function:       // mtn_path_tracker
  Description:    // path tracker
  Input:          // None
  Output:         // none
  Others:         // none
*************************************************/
void mtn_path_tracker()
{
	if(!motion.tracker.path_imu.pointReached)
	{
		track_point(&motion.tracker, g_decision.params.to_pose.pos[0], g_decision.params.to_pose.pos[1]);
	}
	else
	{
		motion.tracker.path_imu.pointReached = FALSE;
		
		g_trigger = POS_REACHED;
		get_decision(&g_trigger, &g_decision);
		action_params_print();
		
		pr_mtn_assigner();
	}
}

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
	int tmp_cnt = 0;
	tmp_cnt = g_decision.params.depart.dep_dist/fabs(v)/MTN_THREAD_TIME*OT_DIST_INFLU_RATIO;
	//tmp_cnt = g_decision.params.depart.dep_dist/fabs(v)/MTN_THREAD_TIME;
	g_counter++;						//when outing station
	
	if(g_counter == tmp_cnt) //rough back distance
	{
		g_trigger = DEP_COMPL;
		get_decision(&g_trigger, &g_decision);
		action_params_print();
	}
	else if(g_counter > tmp_cnt)
	{
		dc_mtn_assigner();
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
		#ifdef JHL_SERIAL_DEBUG
		rt_kprintf("[ljh]mq is FULL, delay 1s.\n");
		#endif
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
  Function:       // get_bump_state
  Description:    // get bump state information
  Input:          // None
  Output:         // T_bool bumped or not
  Others:         // none
*************************************************/
T_bool bump_check()
{
	get_front_bumper_info(&g_Bumper);
	if( (g_Bumper.left == 0) && (g_bump_stage == NOT_BMPD) )
	{
		g_bump_stage = BUMPED;
		g_counter = 0;

		set_velocity_hard(0, 0);
		return TRUE;
	}
	else
		return FALSE;
}

/*************************************************
  Function:       // get_rt_decision
  Description:    // get decision after rigid touched
  Input:          // None
  Output:         // none
  Others:         // none
*************************************************/
void get_rt_decision()
{
	if(g_bump_stage == BUMPED)
	{
		g_trigger = RIGID_TOUCHED;
		get_decision(&g_trigger, &g_decision);
		action_params_print();
		
		g_counter = 0;
		g_bump_stage = BACK;
	}
}

/*************************************************
  Function:       // bk_after_bump
  Description:    // back certain distance after bump
  Input:          // None
  Output:         // T_bool back finished or not
  Others:         // none
*************************************************/
T_bool bk_after_bump()
{
	if(g_bump_stage == BACK)
	{
		set_velocity(BACK_LNR_VEL_AFT_BMP, 0);
		g_counter++;
		if(g_counter>=BACK_TCNT_AFT_BMP)
		{
			return TRUE;
		}
		else 
			return FALSE;
	}
	else
		return FALSE;
}

/*************************************************
  Function:       // obst_check
  Description:    // return if obstacle sensed or not
  Input:          // NONE
Output:         // TRUE: obstacle sensed, vice versa
  Others:         // none
*************************************************/
T_bool obst_check()
{
	float dist=0;
	dist = get_dist_from_sensors();

	if(g_os_stage == CLEAR)
	{
		if(dist <= OA_AVOIDANCE_DIST)
		{
			g_os_stage = DECS;
			#ifdef JHL_SERIAL_DEBUG
			rt_kprintf("obstacle sensed!\n");
			#endif
			return TRUE;//g_obstacle_sensed = TRUE;
		}
		else 
			return FALSE; //g_obstacle_sensed = FALSE;
	}
	else 
		return FALSE;
}

/*************************************************
  Function:       // get_os_decision
  Description:    // corresponding processing when obstacle sensed
  Input:          // NONE
  Output:         // none
  Others:         // none
*************************************************/
void get_os_decision()
{
	if(g_os_stage == DECS)
	{
		g_trigger = OBST_SENSED;
		get_decision(&g_trigger, &g_decision);
		action_params_print();
		
		g_os_stage=W8_RM;
	}	
}
/*************************************************
  Function:       // auto_arc
  Description:    // algorithm of running a arc
  Input:          // NONE
  Output:         // none
  Others:         // none
*************************************************/
void auto_arc(float radius, float finish_time)
{
	if(!g_dir_memd)
	{
		motion.tracker.path_imu.oa_pre_dir_x = motion.tracker.sense.dir_x;
		motion.tracker.path_imu.oa_pre_dir_y = motion.tracker.sense.dir_y;
		g_dir_memd = TRUE;
	}
	
	float v = ( radius * 2) / finish_time;
	float omega = 90 / (57.3 * finish_time);
	float dp = motion.tracker.sense.dir_x*motion.tracker.path_imu.oa_pre_dir_x + motion.tracker.sense.dir_y*motion.tracker.path_imu.oa_pre_dir_y;
	float angle = acosf(dp)*57.3;

	if(angle >= 175)
	{
		g_end_of_arc_reached = TRUE;
		g_dir_memd = FALSE;
	}
	else
	{
		if(g_decision.params.bypass.pass_side == LEFT_STEERING)
			set_velocity(v, -omega);
		else
			set_velocity(v, omega);
	}
}

/*************************************************
  Function:       // mtn_obstacle_avoidance
  Description:    // algorithm of avoiding obstacles
  Input:          // none
  Output:         // none
  Others:         // none
*************************************************/
void mtn_obstacle_avoidance()
{
	if(g_oa_stage == FIRST_ROTATION)
	{
		if(!motion.tracker.path_imu.rotationFinished) //rotate_angle(&motion.tracker, 90, LEFT_STEERING);
			rotate_angle(&motion.tracker, 90, g_decision.params.bypass.pass_side);
		else
		{
			g_oa_stage = AUTO_ARC;
			#ifdef JHL_SERIAL_DEBUG
			rt_kprintf("first rotation finished.\n");
			#endif
			motion.tracker.path_imu.rotationFinished = FALSE;			
		}
	}
	
	if(g_oa_stage == AUTO_ARC)
	{
		if(!g_end_of_arc_reached)
			auto_arc(ARC_RADIUS, ARC_FINISH_TIME);
		else
		{
			g_oa_stage = SECOND_ROTATION;
			#ifdef JHL_SERIAL_DEBUG
			rt_kprintf("auto_arc finished.\n");
			#endif
		}
	}
	
	if(g_oa_stage == SECOND_ROTATION)
	{
		if(!motion.tracker.path_imu.rotationFinished) //rotate_angle(&motion.tracker, 90, LEFT_STEERING);
			rotate_angle(&motion.tracker, 90, g_decision.params.bypass.pass_side);
		else
		{
			g_oa_stage = OA_FINISHED;
			motion.tracker.path_imu.rotationFinished = FALSE;
			#ifdef JHL_SERIAL_DEBUG
			rt_kprintf("seconde rotation finished.\n");
			#endif
		}
	}
	
	if(g_oa_stage == OA_FINISHED)
	{
		//only for zigzag state
		motion.motion_state = MOTION_STATE_ZIGZAG;
		motion.zigzag.state = T_MOTION_ZIGZAG_STATE_LINE;
		
		g_mtn_stage = RUN;
		g_os_stage = CLEAR;
		g_oa_stage = FIRST_ROTATION;
		
		g_end_of_arc_reached = FALSE;
		g_obstacle_sensed = FALSE;
		
		#ifdef JHL_SERIAL_DEBUG
			rt_kprintf("oa finished, going back to zigzag_line.\n");
		#endif
	}
}

/*************************************************
  Function:       // get_dist_from_sensors
  Description:    // get distance information from electromagnetic sensor and proximity sensor
  Input:          // void
  Output:         // distance from vehicle to obstacle or wire
  Others:         // none
*************************************************/
float get_dist_from_sensors()
{
	float dist=0;
	
	dist = motion.tracker.sense.sonar_l < motion.tracker.sense.sonar_r ? motion.tracker.sense.sonar_l : motion.tracker.sense.sonar_r;
	//printf("sonar_left: %.3f, sonar_right: %.3f", motion.tracker.sense.sonar_l, motion.tracker.sense.sonar_r);
	return dist;
	//return OA_DECELERATION_DIST;
 }

 /*************************************************
  Function:       // get_tuned_vel
  Description:    // get the tuned linear velocity
  Input:          // void
  Output:         // none
  Others:         // none
*************************************************/
void get_tuned_vel()
{
	#ifdef ULTRASONIC_ON
		if(motion.motion_state != MOTION_STATE_WIRE) 
		{
			if( (motion.motion_state == MOTION_STATE_ZIGZAG) && ((motion.zigzag.state == T_MOTION_ZIGZAG_STATE_TURN) || (motion.zigzag.state == T_MOTION_ZIGZAG_STATE_BACK) ) )
			{}
			else
				motion.tracker.line_vel = tune_linear_vel();//jhl_bug
		}
	#endif
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
	static float cur_vel = 0;
	float tuned_vel = 0;
	float dist = 0;
	float prop = 0;
	
	cur_vel = motion.tracker.line_vel;
	
	dist = get_dist_from_sensors();
	prop = dist/OA_DECELERATION_DIST;
	
//	if(dist <= OA_AVOIDANCE_DIST)
//		g_obstacle_sensed = TRUE;
//	else 
//		g_obstacle_sensed = FALSE;
		
	if(prop >= 1)
		prop = 1;
	
	tuned_vel = prop * cur_vel;
	
	return tuned_vel;
}

/*************************************************
  Function:       // action_params_print
  Description:    // print out actions and action params
  Input:          // NONE
  Output:         // none
  Others:         // none
*************************************************/
void action_params_print()
{
	switch(g_decision.action)
	{
		case 0:
			#ifdef JHL_SERIAL_DEBUG
			rt_kprintf("[ljh]recv action: ACT_NULL\n");
			#endif
		break;
		
		case 1:
			#ifdef JHL_SERIAL_DEBUG
			rt_kprintf("[ljh]recv action: DEPART, params: dep_dist(%d)\n", (int)(1000*g_decision.params.depart.dep_dist) );
			#endif
		break;
				
		case 2:
			#ifdef JHL_SERIAL_DEBUG
			rt_kprintf("[ljh]recv action: TO_POSE, params: pos(%d mm, %d mm), vec(%d, %d)\n", 
									(int)(1000*g_decision.params.to_pose.pos[0]), (int)(1000*g_decision.params.to_pose.pos[1]), 
									(int)(1000*g_decision.params.to_pose.vec[0]), (int)(1000*g_decision.params.to_pose.vec[1]));
			#endif
		break;
		
		case 3:
			#ifdef JHL_SERIAL_DEBUG
			rt_kprintf("[ljh]recv action: DIR_DRIVE, params: rot_side(%d), fin_vec(%d, %d)\n", 
									g_decision.params.dir_drive.rot_side, 
									(int)(1000*g_decision.params.dir_drive.fin_vec[0]), 
									(int)(1000*g_decision.params.dir_drive.fin_vec[1]));
			#endif
		break;
		
		case 4:
			#ifdef JHL_SERIAL_DEBUG
			rt_kprintf("[ljh]recv action: U_TURN, params: turn_side(%d), fin_vec(%d, %d)\n", 
									g_decision.params.u_turn.turn_side, 
									(int)(1000*g_decision.params.u_turn.fin_vec[0]), (int)(1000*g_decision.params.u_turn.fin_vec[1]));
			#endif
		break;
		
		case 5:
			#ifdef JHL_SERIAL_DEBUG
			rt_kprintf("[ljh]recv action: ONTO_WIRE, params: enter_side(%d), laps(%d), fin_vec(%d, %d)\n", 
									g_decision.params.onto_wire.ntr_side, g_decision.params.onto_wire.laps, 
									(int)(1000*g_decision.params.onto_wire.fin_vec[0]), 
									(int)(1000*g_decision.params.onto_wire.fin_vec[1]));
			#endif
		break;
									
		case 6:
			#ifdef JHL_SERIAL_DEBUG
			rt_kprintf("[ljh]recv action: BYPASS, params: pass_side(%d)\n", 
									g_decision.params.bypass.pass_side);
			#endif
		break;
		
		default:
			#ifdef JHL_SERIAL_DEBUG
			rt_kprintf("[ljh]recv invalid action!\n");
			#endif
		break;
	}
	#ifdef JHL_SERIAL_DEBUG
	rt_kprintf("\n");
	#endif
}

/*************************************************
  Function:       // heading_control
  Description:    // heading control algorithm in state zigzag
  Input:          // NONE
  Output:         // none
  Others:         // none
*************************************************/
void heading_control()
{
	switch(g_decision.action)
	{
		case ONTO_WIRE:
			track_vector(&motion.tracker, g_decision.params.onto_wire.fin_vec[0], g_decision.params.onto_wire.fin_vec[1], HEADING_CTRL_LINEAR_VEL);
		break;
		
		case U_TURN:
		case BYPASS:
			track_vector(&motion.tracker,g_decision.params.u_turn.fin_vec[0], g_decision.params.u_turn.fin_vec[1], HEADING_CTRL_LINEAR_VEL);	
		break;
		
		case DIR_DRIVE:
//			if(g_end_of_to_pose)
//			{
//				if(!motion.tracker.path_imu.rotationFinished)
//					rotate_vector(&motion.tracker, g_decision.params.dir_drive.fin_vec[0], g_decision.params.dir_drive.fin_vec[1],
//												g_decision.params.dir_drive.rot_side, 0);
//				else
//				{
//					motion.tracker.path_imu.rotationFinished = FALSE;
//					g_end_of_to_pose = FALSE;
//				}
//			}
//			else 
				track_vector(&motion.tracker, g_decision.params.dir_drive.fin_vec[0], g_decision.params.dir_drive.fin_vec[1], HEADING_CTRL_LINEAR_VEL);
		break;
			
		default:
		{
			#ifdef JHL_SERIAL_DEBUG
			rt_kprintf("Invalid action. Can't complete heading control!\n");
			#endif
		}
		break;
	}
}

/*************************************************
  Function:       // mtn_onto_wire
  Description:    // onto wire enter side assign and get on to wire
  Input:          // enter side, rotation velocity and linear velocity
  Output:         // none
  Others:         // none
*************************************************/
void mtn_onto_wire(T_gp_side enterSide, float rot_vel, float linear_vel)
{
	if(enterSide == LEFT_STEERING)
	{
		if(	(motion.tracker.sense.side_l == MOTION_MAG_LINE_OUTSIDE) 
			&& (motion.tracker.sense.side_r == MOTION_MAG_LINE_INSIDE) )
		{
			motion.tracker.path_mag_line.track_dir = MOTION_WIRE_CCW;
			motion.motion_state = MOTION_STATE_WIRE;
			motion.zigzag.state = T_MOTION_ZIGZAG_STATE_NONE;
			g_counter = 0;  //reset the counter in out_station
			#ifdef JHL_SERIAL_DEBUG
			rt_kprintf("onto_wire direct.\n");
			#endif
		}
		else
			set_velocity(0, ROT_VEL_OW);
	}
	else if(enterSide == RIGHT_STEERING)
	{
		if(	(motion.tracker.sense.side_l == MOTION_MAG_LINE_INSIDE) 
			&& (motion.tracker.sense.side_r == MOTION_MAG_LINE_OUTSIDE) )
		{
			motion.tracker.path_mag_line.track_dir = MOTION_WIRE_CW;
			motion.motion_state = MOTION_STATE_WIRE;
			motion.zigzag.state = T_MOTION_ZIGZAG_STATE_NONE;
			g_counter = 0;  //reset the counter in out_station
			#ifdef JHL_SERIAL_DEBUG
			rt_kprintf("onto_wire reverse.\n");
			#endif
		}
		else
			set_velocity(0, -ROT_VEL_OW);
	}
}

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
	else if(mag_state.right_sensor_now == MOTION_MAG_LINE_INSIDE)
		mag_state.right_sensor_change = 0;
	
	if(mag_state.left_sensor_now == MOTION_MAG_LINE_OUTSIDE)
	{
		mag_state.left_sensor_change = 1;
	}
	else if(mag_state.left_sensor_now == MOTION_MAG_LINE_INSIDE)
		mag_state.left_sensor_change = 0;
	
	
	for(i = 0; i < MAG_CNT; i++)
	{
		mag_state.right_sensor_old[i] = mag_state.right_sensor_old[i+1];
		mag_state.left_sensor_old[i] = mag_state.left_sensor_old[i+1];
	}
	mag_state.right_sensor_old[MAG_CNT-1] = mag_state.right_sensor_now;
	mag_state.left_sensor_old[MAG_CNT-1] = mag_state.left_sensor_now;
}

void Motion_Norm_2D(float* x, float* y)
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

/*************************************************
  Function:       // mtn_cover_FSM
  Description:    // zigzag state machine
  Input:          // NONE
  Output:         // none
  Others:         // none
*************************************************/
int g_d2w_lock_cntr = 0;
void mtn_cover_FSM()
{
	switch(motion.zigzag.state)
	{
		case T_MOTION_ZIGZAG_STATE_LINE:
			mag_sensor_update();
			g_lock_cnt_ut++;
		
			if((mag_state.left_sensor_change==0)&&(mag_state.right_sensor_change==0)&&g_decision.action==ONTO_WIRE )
			{
				g_d2w_lock_cntr++;
				
				if(!g_d2w_abt_ut && (g_d2w_lock_cntr>=50))
				{
					g_d2w_abt_ut = TRUE;
					#ifdef JHL_SERIAL_DEBUG
						rt_kprintf("Inside-flag set after ONTO_WIRE. Able to U_TURN now.\n");
					#endif
				}
			}

			T_bool ut_cdt = ((mag_state.left_sensor_change == 1) || (mag_state.right_sensor_change == 1) ) && (g_lock_cnt_ut > 30);
			if(ut_cdt && (g_d2w_abt_ut || (g_decision.action != ONTO_WIRE) ) )	
			{
				g_trigger = WIRE_SENSED;
				set_velocity_hard(0, 0);
				get_decision(&g_trigger, &g_decision);
				action_params_print();
				
				g_counter =0 ;
				g_d2w_abt_ut = FALSE; 
				g_d2w_lock_cntr = 0;
				motion.zigzag.state = T_MOTION_ZIGZAG_STATE_BACK;
			}
			else 
				heading_control();
		
		break;
		
		case T_MOTION_ZIGZAG_STATE_BACK:
			ws_mtn_assigner();
		break;
		
		case T_MOTION_ZIGZAG_STATE_TURN:
			g_lock_cnt_ut=0;
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
		break;
			
		case T_MOTION_ZIGZAG_STATE_ONTOWIRE:
			mtn_onto_wire(g_decision.params.onto_wire.ntr_side, ROT_VEL_OW, LNR_VEL_OW);
		break;
	}
}

/*
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
		g_lock_cnt_ut++;

		

		if((mag_state.left_sensor_change == 1)&&(mag_state.right_sensor_change == 1)&&(g_lock_cnt_ut > 150)&&(dot_product > 0.96))
		{
			motion->zigzag.state = T_MOTION_ZIGZAG_STATE_TURN;
			
			mag_state.right_sensor_change = 0;
			mag_state.left_sensor_change = 0;
			
			g_lock_cnt_ut = 0;
			
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
		
		g_lock_cnt_ut = 0;
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
//				
//				motion->zigzag.turn_dir += 1;
//				if(motion->zigzag.turn_dir>2)
//					motion->zigzag.turn_dir=1;
//				

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
//funcitons *****************************************************************
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
*/
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
	Motion_Set_Mag_Tracking_Param(&motion->tracker,  0.02f, 0, 0);//pre is 0.000001
	Motion_Set_Mag_Gotoline_Param(&motion->tracker,0,0,0);
	
//	Motion_Update_2D_Line(&motion->tracker,1,0,1,0,0.1);
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
    
	float omega_leftWheel =  1/LEFT_WHEEL_RADIUS * motion.tracker.line_vel \
														+ VEHICLE_BODY_WIDTH/(2*LEFT_WHEEL_RADIUS) * motion.tracker.angular_vel;
	float omega_rightWheel = 1/RIGHT_WHEEL_RADIUS * motion.tracker.line_vel
														- VEHICLE_BODY_WIDTH/(2*RIGHT_WHEEL_RADIUS) * motion.tracker.angular_vel;
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
		
	omega_dr_leftWheel = 3600 * omega_leftWheel/WHEEL_OMEGA_MAX; // dr == dutyratio
	omega_dr_rightWheel = 3600 * omega_rightWheel/WHEEL_OMEGA_MAX;
	//Set Speed
	if(1 == MOTOR_DRIVER_SELECT)
	{
		set_motor_control_speed_s32(omega_dr_leftWheel,omega_dr_rightWheel);
	}
	else if(0 == MOTOR_DRIVER_SELECT)
	{
		set_motor_speed_spi(omega_dr_leftWheel,omega_dr_rightWheel);
	}
}

void set_motor_zero(void)
{
	set_motor_speed_spi(0, 0);
	set_motor_control_speed_s32(0, 0);
	
	motion.tracker.line_vel = 0;
	motion.tracker.angular_vel = 0;
	Motion_Process_Motor_Speed();
	update_motor_control();
}

void Motion_Zigzag_Init(T_motion* motion,float blade_bodywidth_ratio,float blade_overlaping_ratio)
{
	motion->zigzag.blade_bodywidth_ratio =   		blade_bodywidth_ratio;
	motion->zigzag.blade_overlaping_ratio =  		blade_overlaping_ratio;
	motion->zigzag.state = 									    T_MOTION_ZIGZAG_STATE_NONE;
	motion->zigzag.f_r = 												T_MOTION_ZIGZAG_GO_FOWARD;
	motion->zigzag.state = 											T_MOTION_ZIGZAG_STATE_LINE;
}

/*
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
*/
