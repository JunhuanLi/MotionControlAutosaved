#define ARC_LINEAR_VEL 0.2
#define OA_RADIUS 0.5;
T_bool g_obstacle_sensed = FALSE;
T_bool g_obstacle_avoidance_finished = FALSE;
T_bool g_end_of_to_pose;
T_bool g_pose_memed = FALSE;
extern unsigned short BACK_TIME_COUNT;

void auto_arc(T_motion* motion, float radius, float linear_vel)
{
	float dist = sqrtf( (motion->tracker.sense.pos_x - motion->tracker.path_imu.oa_arc_center_x)
							 * (motion->tracker.sense.pos_x - motion->tracker.path_imu.oa_arc_center_x)
							 + (motion->tracker.sense.pos_y - motion->tracker.path_imu.oa_arc_center_y)
							 * (motion->tracker.sense.pos_y - motion->tracker.path_imu.oa_arc_center_y) );
	float dist_err = radius - dist;
	set_velocity(&motion->tracker, linear_vel, -dist_err);
}
/*************************************************
  Function:       // obstacle_avoidance
  Description:    // algorithm of avoiding obstacles
  Input:          // arc radius and T_motion
  Output:         // none
  Others:         // none
*************************************************/
void obstacle_avoidance(T_motion* motion, float radius)
{
	/*OBS*/
	//store the current pose
	if(!g_pose_memed)
	{
		motion->tracker.path_imu.oa_pre_pos_x = motion->tracker.sense.pos_x;
		motion->tracker.path_imu.oa_pre_pos_y = motion->tracker.sense.pos_y;
		motion->tracker.path_imu.oa_pre_dir_x = motion->tracker.sense.dir_x;
		motion->tracker.path_imu.oa_pre_dir_y = motion->tracker.sense.dir_y;
	
		float tmp_theta = atan2f(motion->tracker.sense.dir_x, motion->tracker.sense.dir_y);
		motion->tracker.path_imu.oa_arc_center_x = cos(tmp_theta)*radius - sin(tmp_theta)*0;
		motion->tracker.path_imu.oa_arc_center_y = sin(tmp_theta)*radius + cos(tmp_theta)*0;
		
		g_pose_memed = TRUE;
	}
	//obstacle avoidance finished
	if(g_obstacle_avoidance_finished)
	{
		if(!motion->tracker.path_imu.rotationFinished)
			rotate_vector(&motion->tracker, 
												motion->tracker.path_imu.oa_pre_dir_x, 
												motion->tracker.path_imu.oa_pre_dir_y, 
												g_action_params.bypass_.pass_side, 
												0.05);
		else
		{
			motion->zigzag.state = T_MOTION_ZIGZAG_STATE_LINE;
			g_obstacle_avoidance_finished = FALSE;
			g_pose_memed = FALSE;
		}
	}
	//avoidance
	else
	{
		//turn 90 degree first
		if(!motion->tracker.path_imu.rotationFinished)
		{
			if(g_action_params.bypass_.pass_side == LEFT_STEERING)
				rotate_vector(&motion->tracker, 
												-motion->tracker.path_imu.oa_pre_pos_y, 
												motion->tracker.path_imu.oa_pre_pos_x, 
												g_action_params.bypass_.pass_side, 
												0);
			else 
				rotate_vector(&motion->tracker, 
												motion->tracker.path_imu.oa_pre_pos_y, 
												-motion->tracker.path_imu.oa_pre_pos_x, 
												g_action_params.bypass_.pass_side, 
												0);
		}
		else
		{	
			float dist = sqrtf( (motion->tracker.sense.pos_x - 2*motion->tracker.path_imu.oa_arc_center_x)
							 * (motion->tracker.sense.pos_x - 2*motion->tracker.path_imu.oa_arc_center_x)
							 + (motion->tracker.sense.pos_y - 2*motion->tracker.path_imu.oa_arc_center_y)
							 * (motion->tracker.sense.pos_y - 2*motion->tracker.path_imu.oa_arc_center_y) );
			//is the terminal condition satisfied?
			if(dist < 0.2)
			{
				g_obstacle_avoidance_finished = TRUE;
				motion->tracker.path_imu.rotationFinished = FALSE;
			}
			else 
				auto_arc(motion, radius, ARC_LINEAR_VEL);
		}
	}
	
}
