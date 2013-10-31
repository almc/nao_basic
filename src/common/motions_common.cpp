#include <nao_basic/motions_common.h>
#include <nao_basic/ball_tracker_common.h>

#define ALVA AL::ALValue::array

void CloseHand(AL::ALMotionProxy* motion_proxy_ptr)
{
	// motion_proxy_ptr->closeHand("RHand");
	AL::ALValue joint_name = "RHand";
	AL::ALValue joint_angle = 0.3f;
	AL::ALValue joint_time = 1.0f;
	bool is_absolute = true;

	motion_proxy_ptr->angleInterpolation(joint_name, joint_angle, joint_time,
	                                     is_absolute);
}

void OpenHand(AL::ALMotionProxy* motion_proxy_ptr)
{
	// motion_proxy_ptr->openHand("RHand");
	AL::ALValue joint_name = "RHand";
	AL::ALValue joint_angle = 1.0f;
	AL::ALValue joint_time = 1.0f;
	bool is_absolute = true;

	motion_proxy_ptr->angleInterpolation(joint_name, joint_angle, joint_time,
	                                     is_absolute);
}


float MoveHand(AL::ALMotionProxy* motion_proxy_ptr,
               float x_hand, float y_hand, float z_hand)
{
	std::vector<float> goal_pos;
	goal_pos.push_back(x_hand);
	goal_pos.push_back(y_hand);
	goal_pos.push_back(z_hand);
	goal_pos.push_back(0.0);
	goal_pos.push_back(0.0);
	goal_pos.push_back(0.0);

	std::string chain_name = "RArm";
	int coordinate_space = 2;       // FRAME_ROBOT
	bool use_sensor_values = true;  // Why was this false in python code?

	std::vector<float> current_pos =
		motion_proxy_ptr->getPosition(chain_name, coordinate_space, use_sensor_values);

	float error_total = 0.0;
	for (int i = 0; i < 3; ++i)
	{
		error_total += fabs(current_pos.at(i)-goal_pos.at(i));
	}

	if (error_total > 0.02)
	{
		std::cout << "Position cannot be reached with enough accuracy, error: "
		          << error_total << std::endl;
	} else {
		std::cout << "Position reached with enough accuracy, error: "
		          << error_total << std::endl;
	}

	float fraction_max_speed = 0.2;
	int axis_mask = 1+2+4+8; // 7 == Only control position,
	                         // 56 == Only rotation,
	                         // 63 == Both position and orientation
	motion_proxy_ptr->setPosition(chain_name, coordinate_space, goal_pos,
	                              fraction_max_speed, axis_mask);

	return error_total;
}

void TrackBallWithHead(AL::ALMotionProxy* motion_proxy_ptr,
						int u_img, int v_img,
						int img_width, int img_height)
{
	AL::ALValue joint_names = "Head"; //Implies ["HeadYaw", "HeadPitch"]
	bool use_sensor_values = true;

	std::vector<float> current_head_angles =
		motion_proxy_ptr->getAngles(joint_names, use_sensor_values);

	float img_HFOV = 47.64*M_PI/180.0;
	float img_WFOV = 60.97*M_PI/180.0;

	float ball_yaw   = -1*((float)u_img - (float)(img_width )/2.0) / (float)img_width  * img_WFOV;
	float ball_pitch =    ((float)v_img - (float)(img_height)/2.0) / (float)img_height * img_HFOV;

	float target_yaw = ball_yaw+current_head_angles.at(0);
	float target_pitch = ball_pitch+current_head_angles.at(1);

	if (fabs(target_yaw) > 45.0*M_PI/180.0)
	{
		target_yaw = current_head_angles.at(0);
	}
	if (fabs(target_pitch) > 45.0*M_PI/180.0)
	{
		target_pitch = current_head_angles.at(1);
	}

	// Example showing how to set angles, using a fraction of max speed
	AL::ALValue names         = ALVA("HeadYaw", "HeadPitch");
	AL::ALValue target_angles = ALVA(target_yaw, target_pitch);
	float fraction_max_speed  = 0.1f;
	motion_proxy_ptr->setAngles(names, target_angles, fraction_max_speed);
}

void CheckHand(AL::ALMotionProxy* motion_proxy_ptr)
{
	std::vector<std::string> joint_names;
	AL::ALValue joint_times;
	AL::ALValue control_points;

	joint_names.push_back("RShoulderPitch");
	joint_names.push_back("RShoulderRoll");
	joint_names.push_back("RElbowYaw");
	joint_names.push_back("RElbowRoll");
	joint_names.push_back("RWristYaw");
	// joint_names.push_back("RHand");
	joint_names.push_back("HeadYaw");
	joint_names.push_back("HeadPitch");


	joint_times.arrayPush(ALVA(6.00000f));
	joint_times.arrayPush(ALVA(6.00000f));
	joint_times.arrayPush(ALVA(6.00000f));
	joint_times.arrayPush(ALVA(6.00000f));
	joint_times.arrayPush(ALVA(6.00000f));
	// joint_times.arrayPush(ALVA(6.00000f));
	joint_times.arrayPush(ALVA(6.00000f));
	joint_times.arrayPush(ALVA(6.00000f));


	control_points.arrayPush(ALVA(ALVA(0.21173f, ALVA(3, -0.33333f, 0.00000f), ALVA(3, 0.00000f, 0.00000f))));
	control_points.arrayPush(ALVA(ALVA(0.09046f, ALVA(3, -0.33333f, 0.00000f), ALVA(3, 0.00000f, 0.00000f))));
	control_points.arrayPush(ALVA(ALVA(0.50004f, ALVA(3, -0.33333f, 0.00000f), ALVA(3, 0.00000f, 0.00000f))));
	control_points.arrayPush(ALVA(ALVA(1.36070f, ALVA(3, -0.33333f, 0.00000f), ALVA(3, 0.00000f, 0.00000f))));
	control_points.arrayPush(ALVA(ALVA(1.35908f, ALVA(3, -0.33333f, 0.00000f), ALVA(3, 0.00000f, 0.00000f))));
	// control_points.arrayPush(ALVA(ALVA(0.00047f, ALVA(3, -0.33333f, 0.00000f), ALVA(3, 0.00000f, 0.00000f))));
	control_points.arrayPush(ALVA(ALVA(0.01837f, ALVA(3, -0.33333f, 0.00000f), ALVA(3, 0.00000f, 0.00000f))));
	control_points.arrayPush(ALVA(ALVA(0.03524f, ALVA(3, -0.33333f, 0.00000f), ALVA(3, 0.00000f, 0.00000f))));

	motion_proxy_ptr->angleInterpolationBezier(joint_names, joint_times, control_points);
}


void Bend(AL::ALMotionProxy* motion_proxy_ptr)
{

	std::vector<std::string> joint_names;
	AL::ALValue joint_times;
	AL::ALValue control_points;

// joint names
	joint_names.push_back("LHipYawPitch");
	joint_names.push_back("LHipRoll");
	joint_names.push_back("LHipPitch");
	joint_names.push_back("LKneePitch");
	joint_names.push_back("LAnklePitch");
	joint_names.push_back("LAnkleRoll");
	joint_names.push_back("RHipRoll");
	joint_names.push_back("RHipPitch");
	joint_names.push_back("RKneePitch");
	joint_names.push_back("RAnklePitch");
	joint_names.push_back("RAnkleRoll");
	joint_names.push_back("LShoulderPitch");
	joint_names.push_back("LShoulderRoll");
	joint_names.push_back("LElbowYaw");
	joint_names.push_back("LElbowRoll");
	joint_names.push_back("LWristYaw");
	joint_names.push_back("LHand");
	joint_names.push_back("RShoulderPitch");
	joint_names.push_back("RShoulderRoll");
	joint_names.push_back("RElbowYaw");
	joint_names.push_back("RElbowRoll");
	joint_names.push_back("RWristYaw");
	joint_names.push_back("RHand");

// joint_times
	joint_times.arrayPush(ALVA(4.1f));
	joint_times.arrayPush(ALVA(4.1f));
	joint_times.arrayPush(ALVA(4.1f));
	joint_times.arrayPush(ALVA(4.1f));
	joint_times.arrayPush(ALVA(4.1f));
	joint_times.arrayPush(ALVA(4.1f));
	joint_times.arrayPush(ALVA(4.1f));
	joint_times.arrayPush(ALVA(4.1f));
	joint_times.arrayPush(ALVA(4.1f));
	joint_times.arrayPush(ALVA(4.1f));
	joint_times.arrayPush(ALVA(4.1f));
	joint_times.arrayPush(ALVA(4.1f));
	joint_times.arrayPush(ALVA(4.1f));
	joint_times.arrayPush(ALVA(4.1f));
	joint_times.arrayPush(ALVA(4.1f));
	joint_times.arrayPush(ALVA(4.1f));
	joint_times.arrayPush(ALVA(4.1f));
	joint_times.arrayPush(ALVA(2.0f));
	joint_times.arrayPush(ALVA(2.0f));
	joint_times.arrayPush(ALVA(2.0f));
	joint_times.arrayPush(ALVA(2.0f));
	joint_times.arrayPush(ALVA(2.0f));
	joint_times.arrayPush(ALVA(2.0f));

// control points
	control_points.arrayPush(ALVA(ALVA(-0.20551f, ALVA(3, -0.36667f, 0.00000f), ALVA(3, 0.00000f, 0.00000f))));
	control_points.arrayPush(ALVA(ALVA(-0.16256f, ALVA(3, -0.36667f, 0.00000f), ALVA(3, 0.00000f, 0.00000f))));
	control_points.arrayPush(ALVA(ALVA(-0.64270f, ALVA(3, -0.36667f, 0.00000f), ALVA(3, 0.00000f, 0.00000f))));
	control_points.arrayPush(ALVA(ALVA(+1.59225f, ALVA(3, -0.36667f, 0.00000f), ALVA(3, 0.00000f, 0.00000f))));
	control_points.arrayPush(ALVA(ALVA(-1.17969f, ALVA(3, -0.36667f, 0.00000f), ALVA(3, 0.00000f, 0.00000f))));
	control_points.arrayPush(ALVA(ALVA(-0.05211f, ALVA(3, -0.36667f, 0.00000f), ALVA(3, 0.00000f, 0.00000f))));
	control_points.arrayPush(ALVA(ALVA(-0.49544f, ALVA(3, -0.36667f, 0.00000f), ALVA(3, 0.00000f, 0.00000f))));
	control_points.arrayPush(ALVA(ALVA(-1.58773f, ALVA(3, -0.36667f, 0.00000f), ALVA(3, 0.00000f, 0.00000f))));
	control_points.arrayPush(ALVA(ALVA(+2.11255f, ALVA(3, -0.36667f, 0.00000f), ALVA(3, 0.00000f, 0.00000f))));
	control_points.arrayPush(ALVA(ALVA(-0.95104f, ALVA(3, -0.36667f, 0.00000f), ALVA(3, 0.00000f, 0.00000f))));
	control_points.arrayPush(ALVA(ALVA(+0.07828f, ALVA(3, -0.36667f, 0.00000f), ALVA(3, 0.00000f, 0.00000f))));
	control_points.arrayPush(ALVA(ALVA(+1.72878f, ALVA(3, -0.36667f, 0.00000f), ALVA(3, 0.00000f, 0.00000f))));
	control_points.arrayPush(ALVA(ALVA(+0.30676f, ALVA(3, -0.36667f, 0.00000f), ALVA(3, 0.00000f, 0.00000f))));
	control_points.arrayPush(ALVA(ALVA(-1.37451f, ALVA(3, -0.36667f, 0.00000f), ALVA(3, 0.00000f, 0.00000f))));
	control_points.arrayPush(ALVA(ALVA(-0.06285f, ALVA(3, -0.36667f, 0.00000f), ALVA(3, 0.00000f, 0.00000f))));
	control_points.arrayPush(ALVA(ALVA(+0.16870f, ALVA(3, -0.36667f, 0.00000f), ALVA(3, 0.00000f, 0.00000f))));
	control_points.arrayPush(ALVA(ALVA(+0.00020f, ALVA(3, -0.36667f, 0.00000f), ALVA(3, 0.00000f, 0.00000f))));
	control_points.arrayPush(ALVA(ALVA(+0.68421f, ALVA(3, -0.33333f, 0.00000f), ALVA(3, 0.00000f, 0.00000f))));
	control_points.arrayPush(ALVA(ALVA(-0.87442f, ALVA(3, -0.33333f, 0.00000f), ALVA(3, 0.00000f, 0.00000f))));
	control_points.arrayPush(ALVA(ALVA(+0.87741f, ALVA(3, -0.33333f, 0.00000f), ALVA(3, 0.00000f, 0.00000f))));
	control_points.arrayPush(ALVA(ALVA(+1.45734f, ALVA(3, -0.33333f, 0.00000f), ALVA(3, 0.00000f, 0.00000f))));
	control_points.arrayPush(ALVA(ALVA(+0.00000f, ALVA(3, -0.33333f, 0.00000f), ALVA(3, 0.00000f, 0.00000f))));
	control_points.arrayPush(ALVA(ALVA(+1.00000f, ALVA(3, -0.33333f, 0.06000f), ALVA(3, 0.00000f, 0.00000f))));

	motion_proxy_ptr->angleInterpolationBezier(joint_names, joint_times, control_points);
}
