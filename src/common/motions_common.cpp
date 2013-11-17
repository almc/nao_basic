#include <nao_basic/motions_common.h>
#include <nao_basic/ball_tracker_common.h>

#define ALVA AL::ALValue::array

void CloseHand(AL::ALMotionProxy* motion_proxy_ptr)
{
	// motion_proxy_ptr->closeHand("RHand");
	AL::ALValue joint_name = "LHand";
	AL::ALValue joint_angle = 0.3f;
	AL::ALValue joint_time = 1.0f;
	bool is_absolute = true;

	motion_proxy_ptr->angleInterpolation(joint_name, joint_angle, joint_time,
	                                     is_absolute);
}

void OpenHand(AL::ALMotionProxy* motion_proxy_ptr)
{
	// motion_proxy_ptr->openHand("RHand");
	AL::ALValue joint_name = "LHand";
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


void MoveHand2(AL::ALMotionProxy* motion_proxy_ptr)
{
// Choregraphe simplified export in c++.
// Add #include <alproxies/almotionproxy.h> at the beginning of this file.
std::vector<std::string> names;
AL::ALValue times, keys;
names.reserve(25);
times.arraySetSize(25);
keys.arraySetSize(25);

names.push_back("HeadPitch");
times[0].arraySetSize(2);
keys[0].arraySetSize(2);

times[0][0] = 0.840000;
keys[0][0] = 0.514872;
times[0][1] = 2.52000;
keys[0][1] = 0.509245;

names.push_back("HeadYaw");
times[1].arraySetSize(2);
keys[1].arraySetSize(2);

times[1][0] = 0.960000;
keys[1][0] = 0.323633;
times[1][1] = 2.52000;
keys[1][1] = 0.311360;

names.push_back("LAnklePitch");
times[2].arraySetSize(2);
keys[2].arraySetSize(2);

times[2][0] = 0.840000;
keys[2][0] = -0.647517;
times[2][1] = 2.40000;
keys[2][1] = -0.642787;

names.push_back("LAnkleRoll");
times[3].arraySetSize(2);
keys[3].arraySetSize(2);

times[3][0] = 0.840000;
keys[3][0] = -0.282743;
times[3][1] = 2.40000;
keys[3][1] = -0.397761;

names.push_back("LElbowRoll");
times[4].arraySetSize(2);
keys[4].arraySetSize(2);

times[4][0] = 0.920000;
keys[4][0] = -0.191709;
times[4][1] = 2.48000;
keys[4][1] = -0.186452;

names.push_back("LElbowYaw");
times[5].arraySetSize(2);
keys[5].arraySetSize(2);

times[5][0] = 0.920000;
keys[5][0] = -1.25140;
times[5][1] = 2.40000;
keys[5][1] = -0.986111;

names.push_back("LHand");
times[6].arraySetSize(2);
keys[6].arraySetSize(2);

times[6][0] = 0.920000;
//keys[6][0] = 0.0174533;
keys[6][0] = 1.0;
times[6][1] = 2.48000;
//keys[6][1] = 0.00539073;
keys[6][1] = 1.0;

names.push_back("LHipPitch");
times[7].arraySetSize(2);
keys[7].arraySetSize(2);

times[7][0] = 0.840000;
keys[7][0] = -1.59072;
times[7][1] = 2.40000;
keys[7][1] = -1.59072;

names.push_back("LHipRoll");
times[8].arraySetSize(2);
keys[8].arraySetSize(2);

times[8][0] = 0.840000;
keys[8][0] = 0.447970;
times[8][1] = 2.40000;
keys[8][1] = 0.475581;

names.push_back("LHipYawPitch");
times[9].arraySetSize(2);
keys[9].arraySetSize(2);

times[9][0] = 0.840000;
keys[9][0] = -0.604353;
times[9][1] = 2.40000;
keys[9][1] = -0.625830;

names.push_back("LKneePitch");
times[10].arraySetSize(2);
keys[10].arraySetSize(2);

times[10][0] = 0.840000;
keys[10][0] = 2.11227;
times[10][1] = 2.40000;
keys[10][1] = 2.11227;

names.push_back("LShoulderPitch");
times[11].arraySetSize(2);
keys[11].arraySetSize(2);

times[11][0] = 0.920000;
keys[11][0] = 0.709511;
times[11][1] = 2.48000;
keys[11][1] = 0.750492;

names.push_back("LShoulderRoll");
times[12].arraySetSize(2);
keys[12].arraySetSize(2);

times[12][0] = 0.920000;
keys[12][0] = 0.159494;
times[12][1] = 2.48000;
keys[12][1] = 0.185572;

names.push_back("LWristYaw");
times[13].arraySetSize(2);
keys[13].arraySetSize(2);

times[13][0] = 0.920000;
keys[13][0] = 0.663225;
times[13][1] = 2.48000;
keys[13][1] = 0.553104;

names.push_back("RAnklePitch");
times[14].arraySetSize(2);
keys[14].arraySetSize(2);

times[14][0] = 0.840000;
keys[14][0] = -1.13446;
times[14][1] = 2.40000;
keys[14][1] = -1.18630;

names.push_back("RAnkleRoll");
times[15].arraySetSize(2);
keys[15].arraySetSize(2);

times[15][0] = 0.840000;
keys[15][0] = -0.0735901;
times[15][1] = 2.40000;
keys[15][1] = -0.0720561;

names.push_back("RElbowRoll");
times[16].arraySetSize(2);
keys[16].arraySetSize(2);

times[16][0] = 0.880000;
keys[16][0] = 1.40365;
times[16][1] = 2.44000;
keys[16][1] = 1.37757;

names.push_back("RElbowYaw");
times[17].arraySetSize(2);
keys[17].arraySetSize(2);

times[17][0] = 0.880000;
keys[17][0] = 0.348176;
times[17][1] = 2.44000;
keys[17][1] = 0.342041;

names.push_back("RHand");
times[18].arraySetSize(2);
keys[18].arraySetSize(2);

times[18][0] = 0.880000;
keys[18][0] = 0.000572468;
times[18][1] = 2.44000;
keys[18][1] = 0.000565487;

names.push_back("RHipPitch");
times[19].arraySetSize(2);
keys[19].arraySetSize(2);

times[19][0] = 0.840000;
keys[19][0] = -0.980268;
times[19][1] = 2.40000;
keys[19][1] = -0.908170;

names.push_back("RHipRoll");
times[20].arraySetSize(2);
keys[20].arraySetSize(2);

times[20][0] = 0.840000;
keys[20][0] = 0.360533;
times[20][1] = 2.40000;
keys[20][1] = 0.348260;

names.push_back("RKneePitch");
times[21].arraySetSize(2);
keys[21].arraySetSize(2);

times[21][0] = 0.840000;
keys[21][0] = 2.11255;
times[21][1] = 2.40000;
keys[21][1] = 2.11255;

names.push_back("RShoulderPitch");
times[22].arraySetSize(2);
keys[22].arraySetSize(2);

times[22][0] = 0.880000;
keys[22][0] = 1.38064;
times[22][1] = 2.44000;
keys[22][1] = 1.37604;

names.push_back("RShoulderRoll");
times[23].arraySetSize(2);
keys[23].arraySetSize(2);

times[23][0] = 0.880000;
keys[23][0] = -0.771643;
times[23][1] = 2.44000;
keys[23][1] = -0.751701;

names.push_back("RWristYaw");
times[24].arraySetSize(2);
keys[24].arraySetSize(2);

times[24][0] = 0.880000;
keys[24][0] = 0.510779;
times[24][1] = 2.44000;
keys[24][1] = 0.423342;

try
{
  motion_proxy_ptr->angleInterpolation(names, keys, times, true);
}
catch(const std::exception&)
{

}

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

void Bend2(AL::ALMotionProxy* motion_proxy_ptr)
{

// Choregraphe simplified export in c++.
// Add #include <alproxies/almotionproxy.h> at the beginning of this file.
std::vector<std::string> names;
AL::ALValue times, keys;
names.reserve(25);
times.arraySetSize(25);
keys.arraySetSize(25);

names.push_back("HeadPitch");
times[0].arraySetSize(3);
keys[0].arraySetSize(3);

times[0][0] = 0.760000;
keys[0][0] = 0.487771;
times[0][1] = 2.16000;
keys[0][1] = 0.470897;
times[0][2] = 2.84000;
keys[0][2] = 0.486237;

names.push_back("HeadYaw");
times[1].arraySetSize(3);
keys[1].arraySetSize(3);

times[1][0] = 0.760000;
keys[1][0] = 0.269941;
times[1][1] = 2.16000;
keys[1][1] = 0.276078;
times[1][2] = 2.84000;
keys[1][2] = 0.262272;

names.push_back("LAnklePitch");
times[2].arraySetSize(3);
keys[2].arraySetSize(3);

times[2][0] = 0.640000;
keys[2][0] = 0.176367;
times[2][1] = 2.04000;
keys[2][1] = -0.753235;
times[2][2] = 2.72000;
keys[2][2] = -0.767043;

names.push_back("LAnkleRoll");
times[3].arraySetSize(3);
keys[3].arraySetSize(3);

times[3][0] = 0.640000;
keys[3][0] = -0.230059;
times[3][1] = 2.04000;
keys[3][1] = -0.197844;
times[3][2] = 2.72000;
keys[3][2] = -0.185572;

names.push_back("LElbowRoll");
times[4].arraySetSize(4);
keys[4].arraySetSize(4);

times[4][0] = 0.720000;
keys[4][0] = -1.49101;
times[4][1] = 1.56000;
keys[4][1] = -1.54462;
times[4][2] = 2.12000;
keys[4][2] = -0.797638;
times[4][3] = 2.80000;
keys[4][3] = -0.530721;

names.push_back("LElbowYaw");
times[5].arraySetSize(3);
keys[5].arraySetSize(3);

times[5][0] = 0.720000;
keys[5][0] = -1.26866;
times[5][1] = 2.12000;
keys[5][1] = -1.55552;
times[5][2] = 2.80000;
keys[5][2] = -1.48035;

names.push_back("LHand");
times[6].arraySetSize(4);
keys[6].arraySetSize(4);

times[6][0] = 0.720000;
keys[6][0] = 1.0;
//keys[6][0] = 0.0113377;
times[6][1] = 1.56000;
keys[6][1] = 1.0;
//keys[6][1] = 0.0174533;
times[6][2] = 2.12000;
keys[6][2] = 1.0;
//keys[6][2] = 0.0174533;
times[6][3] = 2.80000;
keys[6][3] = 1.0;
//keys[6][3] = 0.0174533;

names.push_back("LHipPitch");
times[7].arraySetSize(3);
keys[7].arraySetSize(3);

times[7][0] = 0.640000;
keys[7][0] = -0.0858620;
times[7][1] = 2.04000;
keys[7][1] = -1.25017;
times[7][2] = 2.72000;
keys[7][2] = -1.29159;

names.push_back("LHipRoll");
times[8].arraySetSize(3);
keys[8].arraySetSize(3);

times[8][0] = 0.640000;
keys[8][0] = 0.242414;
times[8][1] = 2.04000;
keys[8][1] = 0.352862;
times[8][2] = 2.72000;
keys[8][2] = 0.357464;

names.push_back("LHipYawPitch");
times[9].arraySetSize(3);
keys[9].arraySetSize(3);

times[9][0] = 0.640000;
keys[9][0] = -0.383458;
times[9][1] = 2.04000;
keys[9][1] = -0.469363;
times[9][2] = 2.72000;
keys[9][2] = -0.464760;

names.push_back("LKneePitch");
times[10].arraySetSize(3);
keys[10].arraySetSize(3);

times[10][0] = 0.640000;
keys[10][0] = -0.0123140;
times[10][1] = 2.04000;
keys[10][1] = 2.09080;
times[10][2] = 2.72000;
keys[10][2] = 2.11255;

names.push_back("LShoulderPitch");
times[11].arraySetSize(4);
keys[11].arraySetSize(4);

times[11][0] = 0.720000;
keys[11][0] = 1.55390;
times[11][1] = 1.56000;
keys[11][1] = 0.877901;
times[11][2] = 2.12000;
keys[11][2] = 1.13205;
times[11][3] = 2.80000;
keys[11][3] = 1.06609;

names.push_back("LShoulderRoll");
times[12].arraySetSize(3);
keys[12].arraySetSize(3);

times[12][0] = 0.720000;
keys[12][0] = 0.124212;
times[12][1] = 2.12000;
keys[12][1] = 0.154892;
times[12][2] = 2.80000;
keys[12][2] = 0.157960;

names.push_back("LWristYaw");
times[13].arraySetSize(4);
keys[13].arraySetSize(4);

times[13][0] = 0.720000;
keys[13][0] = 0.855930;
times[13][1] = 1.56000;
keys[13][1] = 1.22697;
times[13][2] = 2.12000;
keys[13][2] = 0.604353;
times[13][3] = 2.80000;
keys[13][3] = 0.641169;

names.push_back("RAnklePitch");
times[14].arraySetSize(3);
keys[14].arraySetSize(3);

times[14][0] = 0.640000;
keys[14][0] = 4.19617e-05;
times[14][1] = 2.04000;
keys[14][1] = -0.926494;
times[14][2] = 2.72000;
keys[14][2] = -0.993989;

names.push_back("RAnkleRoll");
times[15].arraySetSize(3);
keys[15].arraySetSize(3);

times[15][0] = 0.640000;
keys[15][0] = -0.0275701;
times[15][1] = 2.04000;
keys[15][1] = 0.0353239;
times[15][2] = 2.72000;
keys[15][2] = 0.0445279;

names.push_back("RElbowRoll");
times[16].arraySetSize(5);
keys[16].arraySetSize(5);

times[16][0] = 0.720000;
keys[16][0] = 1.54462;
times[16][1] = 1.24000;
keys[16][1] = 1.54462;
times[16][2] = 1.60000;
keys[16][2] = 1.13272;
times[16][3] = 2.08000;
keys[16][3] = 1.12753;
times[16][4] = 2.76000;
keys[16][4] = 1.12446;

names.push_back("RElbowYaw");
times[17].arraySetSize(5);
keys[17].arraySetSize(5);

times[17][0] = 0.720000;
keys[17][0] = 1.38056;
times[17][1] = 1.24000;
keys[17][1] = 0.455531;
times[17][2] = 1.60000;
keys[17][2] = 0.226893;
times[17][3] = 2.08000;
keys[17][3] = 0.325165;
times[17][4] = 2.76000;
keys[17][4] = 0.322099;

names.push_back("RHand");
times[18].arraySetSize(4);
keys[18].arraySetSize(4);

times[18][0] = 0.720000;
keys[18][0] = 0.00869174;
times[18][1] = 1.24000;
keys[18][1] = 0.00000;
times[18][2] = 2.08000;
keys[18][2] = 0.000111701;
times[18][3] = 2.76000;
keys[18][3] = 0.000181514;

names.push_back("RHipPitch");
times[19].arraySetSize(3);
keys[19].arraySetSize(3);

times[19][0] = 0.640000;
keys[19][0] = -0.108956;
times[19][1] = 2.04000;
keys[19][1] = -0.882091;
times[19][2] = 2.72000;
keys[19][2] = -0.877490;

names.push_back("RHipRoll");
times[20].arraySetSize(3);
keys[20].arraySetSize(3);

times[20][0] = 0.640000;
keys[20][0] = 0.104354;
times[20][1] = 2.04000;
keys[20][1] = 0.145772;
times[20][2] = 2.72000;
keys[20][2] = 0.145772;

names.push_back("RKneePitch");
times[21].arraySetSize(3);
keys[21].arraySetSize(3);

times[21][0] = 0.640000;
keys[21][0] = 0.213269;
times[21][1] = 2.04000;
keys[21][1] = 1.98350;
times[21][2] = 2.72000;
keys[21][2] = 2.02032;

names.push_back("RShoulderPitch");
times[22].arraySetSize(3);
keys[22].arraySetSize(3);

times[22][0] = 0.720000;
keys[22][0] = 1.45121;
times[22][1] = 2.08000;
keys[22][1] = 1.36837;
times[22][2] = 2.76000;
keys[22][2] = 1.37757;

names.push_back("RShoulderRoll");
times[23].arraySetSize(5);
keys[23].arraySetSize(5);

times[23][0] = 0.720000;
keys[23][0] = -0.0230520;
times[23][1] = 1.24000;
keys[23][1] = -0.834267;
times[23][2] = 1.60000;
keys[23][2] = -0.551524;
times[23][3] = 2.08000;
keys[23][3] = -0.632050;
times[23][4] = 2.76000;
keys[23][4] = -0.625914;

names.push_back("RWristYaw");
times[24].arraySetSize(4);
keys[24].arraySetSize(4);

times[24][0] = 0.720000;
keys[24][0] = -0.831470;
times[24][1] = 1.24000;
keys[24][1] = 0.431096;
times[24][2] = 2.08000;
keys[24][2] = 0.808375;
times[24][3] = 2.76000;
keys[24][3] = 0.785367;

try
{

	motion_proxy_ptr->angleInterpolation(names, keys, times, true);
}
catch(const std::exception&)
{

}
	}

void UnBend2(AL::ALMotionProxy* motion_proxy_ptr)
{

// Choregraphe simplified export in c++.
// Add #include <alproxies/almotionproxy.h> at the beginning of this file.
std::vector<std::string> names;
AL::ALValue times, keys;
names.reserve(25);
times.arraySetSize(25);
keys.arraySetSize(25);

names.push_back("HeadPitch");
times[0].arraySetSize(2);
keys[0].arraySetSize(2);

times[0][0] = 0.840000;
keys[0][0] = -0.613927;
times[0][1] = 2.80000;
keys[0][1] = 0.00000;

names.push_back("HeadYaw");
times[1].arraySetSize(2);
keys[1].arraySetSize(2);

times[1][0] = 1.64000;
keys[1][0] = 0.311360;
times[1][1] = 2.80000;
keys[1][1] = 0.00000;

names.push_back("LAnklePitch");
times[2].arraySetSize(2);
keys[2].arraySetSize(2);

times[2][0] = 1.64000;
keys[2][0] = -0.671934;
times[2][1] = 2.80000;
keys[2][1] = -0.349066;

names.push_back("LAnkleRoll");
times[3].arraySetSize(2);
keys[3].arraySetSize(2);

times[3][0] = 1.64000;
keys[3][0] = -0.360449;
times[3][1] = 2.80000;
keys[3][1] = 0.00000;

names.push_back("LElbowRoll");
times[4].arraySetSize(2);
keys[4].arraySetSize(2);

times[4][0] = 1.64000;
keys[4][0] = -0.188640;
times[4][1] = 2.80000;
keys[4][1] = -1.04720;

names.push_back("LElbowYaw");
times[5].arraySetSize(2);
keys[5].arraySetSize(2);

times[5][0] = 1.64000;
keys[5][0] = -1.00481;
times[5][1] = 2.80000;
keys[5][1] = -1.39626;

names.push_back("LHand");
times[6].arraySetSize(2);
keys[6].arraySetSize(2);

times[6][0] = 1.64000;
keys[6][0] = 0.00545939;
times[6][1] = 2.80000;
keys[6][1] = 0.00000;

names.push_back("LHipPitch");
times[7].arraySetSize(2);
keys[7].arraySetSize(2);

times[7][0] = 1.64000;
keys[7][0] = -1.43885;
times[7][1] = 2.80000;
keys[7][1] = -0.436332;

names.push_back("LHipRoll");
times[8].arraySetSize(2);
keys[8].arraySetSize(2);

times[8][0] = 1.64000;
keys[8][0] = 0.493989;
times[8][1] = 2.80000;
keys[8][1] = 0.00000;

names.push_back("LHipYawPitch");
times[9].arraySetSize(3);
keys[9].arraySetSize(3);

times[9][0] = 0.840000;
keys[9][0] = -0.606479;
times[9][1] = 1.64000;
keys[9][1] = -0.507713;
times[9][2] = 2.80000;
keys[9][2] = 0.00000;

names.push_back("LKneePitch");
times[10].arraySetSize(2);
keys[10].arraySetSize(2);

times[10][0] = 1.64000;
keys[10][0] = 2.09387;
times[10][1] = 2.80000;
keys[10][1] = 0.698132;

names.push_back("LShoulderPitch");
times[11].arraySetSize(3);
keys[11].arraySetSize(3);

times[11][0] = 0.840000;
keys[11][0] = 0.676933;
times[11][1] = 1.64000;
keys[11][1] = 0.720938;
times[11][2] = 2.80000;
keys[11][2] = 1.39626;

names.push_back("LShoulderRoll");
times[12].arraySetSize(2);
keys[12].arraySetSize(2);

times[12][0] = 1.64000;
keys[12][0] = 0.426831;
times[12][1] = 2.80000;
keys[12][1] = 0.349066;

names.push_back("LWristYaw");
times[13].arraySetSize(2);
keys[13].arraySetSize(2);

times[13][0] = 1.64000;
keys[13][0] = 0.547595;
times[13][1] = 2.80000;
keys[13][1] = 0.00000;

names.push_back("RAnklePitch");
times[14].arraySetSize(2);
keys[14].arraySetSize(2);

times[14][0] = 1.64000;
keys[14][0] = -0.487771;
times[14][1] = 2.80000;
keys[14][1] = -0.349066;

names.push_back("RAnkleRoll");
times[15].arraySetSize(2);
keys[15].arraySetSize(2);

times[15][0] = 1.64000;
keys[15][0] = -0.0582500;
times[15][1] = 2.80000;
keys[15][1] = 0.00000;

names.push_back("RElbowRoll");
times[16].arraySetSize(2);
keys[16].arraySetSize(2);

times[16][0] = 1.64000;
keys[16][0] = 1.53414;
times[16][1] = 2.80000;
keys[16][1] = 1.04720;

names.push_back("RElbowYaw");
times[17].arraySetSize(2);
keys[17].arraySetSize(2);

times[17][0] = 1.64000;
keys[17][0] = 0.403400;
times[17][1] = 2.80000;
keys[17][1] = 1.39626;

names.push_back("RHand");
times[18].arraySetSize(2);
keys[18].arraySetSize(2);

times[18][0] = 1.64000;
keys[18][0] = 0.00918741;
times[18][1] = 2.80000;
keys[18][1] = 0.00000;

names.push_back("RHipPitch");
times[19].arraySetSize(2);
keys[19].arraySetSize(2);

times[19][0] = 1.64000;
keys[19][0] = -1.20577;
times[19][1] = 2.80000;
keys[19][1] = -0.436332;

names.push_back("RHipRoll");
times[20].arraySetSize(2);
keys[20].arraySetSize(2);

times[20][0] = 1.64000;
keys[20][0] = 0.343659;
times[20][1] = 2.80000;
keys[20][1] = 0.00000;

names.push_back("RKneePitch");
times[21].arraySetSize(2);
keys[21].arraySetSize(2);

times[21][0] = 1.64000;
keys[21][0] = 1.85312;
times[21][1] = 2.80000;
keys[21][1] = 0.698132;

names.push_back("RShoulderPitch");
times[22].arraySetSize(3);
keys[22].arraySetSize(3);

times[22][0] = 0.840000;
keys[22][0] = 0.877901;
times[22][1] = 1.64000;
keys[22][1] = 1.22348;
times[22][2] = 2.80000;
keys[22][2] = 1.39626;

names.push_back("RShoulderRoll");
times[23].arraySetSize(2);
keys[23].arraySetSize(2);

times[23][0] = 1.64000;
keys[23][0] = -1.26187;
times[23][1] = 2.80000;
keys[23][1] = -0.349066;

names.push_back("RWristYaw");
times[24].arraySetSize(2);
keys[24].arraySetSize(2);

times[24][0] = 1.64000;
keys[24][0] = 0.440216;
times[24][1] = 2.80000;
keys[24][1] = 0.00000;

try
{
  motion_proxy_ptr->angleInterpolation(names, keys, times, true);
}
catch(const std::exception&)
{

}

}

