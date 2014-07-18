#include <nao_basic/motions_common.h>
#include <nao_basic/ball_tracker_common.h>
#include <alproxies/almotionproxy.h>

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




void MoveBack(AL::ALMotionProxy* motion_proxy_ptr)
{

  // Example showing the moveTo command
  // as length of path is less than 0.4m
  // the path will be an SE3 interpolation
  // The units for this command are meters and radians
 		 float x  = 0.0f;
 		 float y  = 0.0f;
  // pi/2 anti-clockwise (90 degrees)
 		 float theta = 0.0f;
  // Will block until walk Task is finished

  // Example showing the moveTo command
  // as length of path is more than 0.4m
  // the path will be follow a dubins curve
  // The units for this command are meters and radians
 		x  = -0.2f;
  		y  = 0.0f;
 		theta  = 0.0f;
  		motion_proxy_ptr->moveTo(x, y, theta);

		AL::ALValue names         = ALVA("HeadYaw", "HeadPitch");
		AL::ALValue target_angles = ALVA(0.5, 0.0);
		float fraction_max_speed  = 0.1f;
		motion_proxy_ptr->setAngles(names, target_angles, fraction_max_speed);
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

bool CheckLHand(AL::ALMotionProxy* motion_proxy_ptr)
{
	const std::string name = "LHand";
	float confidence = 0.95;
	std::vector<float> commandAngles;
	std::vector<float> sensorAngles;
	bool useSensors;
	useSensors = false;
	commandAngles = motion_proxy_ptr->getAngles(name, useSensors);
	useSensors = true;
	sensorAngles  = motion_proxy_ptr->getAngles(name, useSensors);
	if (std::abs(commandAngles[0] / sensorAngles[0]) < confidence)
	{
		return true;
	}
	return false;
}


void Kickold(AL::ALMotionProxy* motion_proxy_ptr, bool hand)
{
// Choregraphe simplified export in c++.
// Add #include <alproxies/almotionproxy.h> at the beginning of this file.
std::vector<std::string> names;
AL::ALValue times, keys;
names.reserve(25);
times.arraySetSize(25);
keys.arraySetSize(25);

names.push_back("HeadPitch");
times[0].arraySetSize(6);
keys[0].arraySetSize(6);

times[0][0] = 1.16000;
keys[0][0] = 0.0436332;
times[0][1] = 2.68000;
keys[0][1] = 0.261799;
times[0][2] = 3.20000;
keys[0][2] = 0.174533;
times[0][3] = 4.24000;
keys[0][3] = -0.279253;
times[0][4] = 5.12000;
keys[0][4] = -0.261799;
times[0][5] = 6.12000;
keys[0][5] = -0.242414;

names.push_back("HeadYaw");
times[1].arraySetSize(6);
keys[1].arraySetSize(6);

times[1][0] = 1.16000;
keys[1][0] = -0.00464395;
times[1][1] = 2.68000;
keys[1][1] = 0.00149204;
times[1][2] = 3.20000;
keys[1][2] = -0.00310997;
times[1][3] = 4.24000;
keys[1][3] = 0.0490460;
times[1][4] = 5.12000;
keys[1][4] = 0.0337060;
times[1][5] = 6.12000;
keys[1][5] = 0.0245859;

names.push_back("LAnklePitch");
times[2].arraySetSize(12);
keys[2].arraySetSize(12);

times[2][0] = 1.04000;
keys[2][0] = 0.0872665;
times[2][1] = 1.32000;
keys[2][1] = -0.0872665;
times[2][2] = 1.76000;
keys[2][2] = -0.593412;
times[2][3] = 2.24000;
keys[2][3] = -0.401426;
times[2][4] = 2.56000;
keys[2][4] = 0.122173;
times[2][5] = 2.84000;
keys[2][5] = -0.0523599;
times[2][6] = 3.08000;
keys[2][6] = -0.122173;
times[2][7] = 3.36000;
keys[2][7] = 0.244346;
times[2][8] = 3.68000;
keys[2][8] = -0.122173;
times[2][9] = 4.12000;
keys[2][9] = -0.644027;
times[2][10] = 5.00000;
keys[2][10] = -0.219911;
times[2][11] = 6.00000;
keys[2][11] = 0.113558;

names.push_back("LAnkleRoll");
times[3].arraySetSize(7);
keys[3].arraySetSize(7);

times[3][0] = 1.04000;
keys[3][0] = -0.401426;
times[3][1] = 2.56000;
keys[3][1] = -0.108872;
times[3][2] = 3.08000;
keys[3][2] = -0.138018;
times[3][3] = 3.36000;
keys[3][3] = 0.00000;
times[3][4] = 4.12000;
keys[3][4] = -0.180970;
times[3][5] = 5.00000;
keys[3][5] = -0.345575;
times[3][6] = 6.00000;
keys[3][6] = -0.0506639;

names.push_back("LElbowRoll");
times[4].arraySetSize(2);
keys[4].arraySetSize(2);

times[4][0] = 1.00000;
keys[4][0] = -0.641169;
times[4][1] = 2.24000;
keys[4][1] = -0.607422;

names.push_back("LElbowYaw");
times[5].arraySetSize(2);
keys[5].arraySetSize(2);

times[5][0] = 1.00000;
keys[5][0] = -0.997141;
times[5][1] = 2.24000;
keys[5][1] = -0.909704;

names.push_back("LHand");
times[6].arraySetSize(5);
keys[6].arraySetSize(5);

times[6][0] = 1.00000;
keys[6][0] = 0.00128883;
times[6][1] = 2.24000;
keys[6][1] = 0.00136136;
times[6][2] = 2.92000;
keys[6][2] = 0.00279253;
times[6][3] = 3.04000;
keys[6][3] = 0.0118560;
times[6][4] = 6.04000;
keys[6][4] = 0.9154317;

names.push_back("LHipPitch");
times[7].arraySetSize(7);
keys[7].arraySetSize(7);

times[7][0] = 1.04000;
keys[7][0] = 0.162646;
times[7][1] = 2.56000;
keys[7][1] = -0.397265;
times[7][2] = 3.08000;
keys[7][2] = -1.11876;
times[7][3] = 3.36000;
keys[7][3] = -1.11978;
times[7][4] = 4.12000;
keys[7][4] = -0.785398;
times[7][5] = 5.00000;
keys[7][5] = -0.291418;
times[7][6] = 6.00000;
keys[7][6] = 0.213183;

names.push_back("LHipRoll");
times[8].arraySetSize(7);
keys[8].arraySetSize(7);

times[8][0] = 1.04000;
keys[8][0] = 0.471239;
times[8][1] = 2.56000;
keys[8][1] = 0.540010;
times[8][2] = 3.08000;
keys[8][2] = 0.322183;
times[8][3] = 3.36000;
keys[8][3] = 0.122762;
times[8][4] = 4.12000;
keys[8][4] = 0.363599;
times[8][5] = 5.00000;
keys[8][5] = 0.417134;
times[8][6] = 6.00000;
keys[8][6] = 0.0582500;

names.push_back("LHipYawPitch");
times[9].arraySetSize(7);
keys[9].arraySetSize(7);

times[9][0] = 1.04000;
keys[9][0] = -0.180970;
times[9][1] = 2.56000;
keys[9][1] = -0.253067;
times[9][2] = 3.08000;
keys[9][2] = -0.0628521;
times[9][3] = 3.36000;
keys[9][3] = -0.0505800;
times[9][4] = 4.12000;
keys[9][4] = -0.187106;
times[9][5] = 5.00000;
keys[9][5] = -0.243864;
times[9][6] = 6.00000;
keys[9][6] = -0.319030;

names.push_back("LKneePitch");
times[10].arraySetSize(8);
keys[10].arraySetSize(8);

times[10][0] = 1.04000;
keys[10][0] = -0.0890139;
times[10][1] = 2.56000;
keys[10][1] = 1.97575;
times[10][2] = 2.84000;
keys[10][2] = 1.97222;
times[10][3] = 3.08000;
keys[10][3] = 1.23918;
times[10][4] = 3.36000;
keys[10][4] = 0.244346;
times[10][5] = 4.12000;
keys[10][5] = 1.53589;
times[10][6] = 5.00000;
keys[10][6] = 0.624296;
times[10][7] = 6.00000;
keys[10][7] = -0.0766580;

names.push_back("LShoulderPitch");
times[11].arraySetSize(2);
keys[11].arraySetSize(2);

times[11][0] = 1.00000;
keys[11][0] = 1.52782;
times[11][1] = 2.24000;
keys[11][1] = 0.783833;

names.push_back("LShoulderRoll");
times[12].arraySetSize(2);
keys[12].arraySetSize(2);

times[12][0] = 1.00000;
keys[12][0] = 0.122678;
times[12][1] = 2.24000;
keys[12][1] = 0.197844;

names.push_back("LWristYaw");
times[13].arraySetSize(2);
keys[13].arraySetSize(2);

times[13][0] = 1.00000;
keys[13][0] = 0.0872665;
times[13][1] = 2.24000;
keys[13][1] = 0.0766580;

names.push_back("RAnklePitch");
times[14].arraySetSize(6);
keys[14].arraySetSize(6);

times[14][0] = 1.04000;
keys[14][0] = 0.0322560;
times[14][1] = 1.76000;
keys[14][1] = 0.0174533;
times[14][2] = 2.56000;
keys[14][2] = 0.0174533;
times[14][3] = 4.12000;
keys[14][3] = 0.0349066;
times[14][4] = 5.00000;
keys[14][4] = 0.0349066;
times[14][5] = 6.00000;
keys[14][5] = 0.115008;

names.push_back("RAnkleRoll");
times[15].arraySetSize(6);
keys[15].arraySetSize(6);

times[15][0] = 1.04000;
keys[15][0] = -0.331613;
times[15][1] = 1.76000;
keys[15][1] = -0.366519;
times[15][2] = 2.56000;
keys[15][2] = -0.366519;
times[15][3] = 4.12000;
keys[15][3] = -0.366519;
times[15][4] = 5.00000;
keys[15][4] = -0.347320;
times[15][5] = 6.00000;
keys[15][5] = 0.0843280;

names.push_back("RElbowRoll");
times[16].arraySetSize(7);
keys[16].arraySetSize(7);

times[16][0] = 1.08000;
keys[16][0] = 0.740964;
times[16][1] = 2.24000;
keys[16][1] = 0.849878;
times[16][2] = 2.60000;
keys[16][2] = 1.03396;
times[16][3] = 3.12000;
keys[16][3] = 1.36990;
times[16][4] = 4.16000;
keys[16][4] = 1.02015;
times[16][5] = 5.04000;
keys[16][5] = 0.707216;
times[16][6] = 6.04000;
keys[16][6] = 0.377323;

names.push_back("RElbowYaw");
times[17].arraySetSize(7);
keys[17].arraySetSize(7);

times[17][0] = 1.08000;
keys[17][0] = 1.15353;
times[17][1] = 2.24000;
keys[17][1] = 1.03694;
times[17][2] = 2.60000;
keys[17][2] = 0.954107;
times[17][3] = 3.12000;
keys[17][3] = 0.908086;
times[17][4] = 4.16000;
keys[17][4] = 1.23023;
times[17][5] = 5.04000;
keys[17][5] = 1.55697;
times[17][6] = 6.04000;
keys[17][6] = 1.14441;

names.push_back("RHand");
times[18].arraySetSize(7);
keys[18].arraySetSize(7);

times[18][0] = 1.08000;
keys[18][0] = 0.00317332;
times[18][1] = 2.24000;
keys[18][1] = 0.00327424;
times[18][2] = 2.60000;
keys[18][2] = 0.00327532;
times[18][3] = 3.12000;
keys[18][3] = 0.00329436;
times[18][4] = 4.16000;
keys[18][4] = 0.00317378;
times[18][5] = 5.04000;
keys[18][5] = 0.00324993;
times[18][6] = 6.04000;
keys[18][6] = 0.00187272;

names.push_back("RHipPitch");
times[19].arraySetSize(7);
keys[19].arraySetSize(7);

times[19][0] = 1.04000;
keys[19][0] = 0.231591;
times[19][1] = 2.56000;
keys[19][1] = 0.105804;
times[19][2] = 3.08000;
keys[19][2] = 0.122173;
times[19][3] = 3.36000;
keys[19][3] = 0.0843280;
times[19][4] = 4.12000;
keys[19][4] = 0.0904641;
times[19][5] = 5.00000;
keys[19][5] = 0.191709;
times[19][6] = 6.00000;
keys[19][6] = 0.210200;

names.push_back("RHipRoll");
times[20].arraySetSize(7);
keys[20].arraySetSize(7);

times[20][0] = 1.04000;
keys[20][0] = 0.343659;
times[20][1] = 2.56000;
keys[20][1] = 0.368202;
times[20][2] = 3.08000;
keys[20][2] = 0.368202;
times[20][3] = 3.36000;
keys[20][3] = 0.365133;
times[20][4] = 4.12000;
keys[20][4] = 0.366667;
times[20][5] = 5.00000;
keys[20][5] = 0.365133;
times[20][6] = 6.00000;
keys[20][6] = -0.101286;

names.push_back("RKneePitch");
times[21].arraySetSize(8);
keys[21].arraySetSize(8);

times[21][0] = 1.04000;
keys[21][0] = -0.0872665;
times[21][1] = 1.76000;
keys[21][1] = -0.0872665;
times[21][2] = 2.56000;
keys[21][2] = -0.0923461;
times[21][3] = 3.08000;
keys[21][3] = -0.0797261;
times[21][4] = 3.36000;
keys[21][4] = -0.0797261;
times[21][5] = 4.12000;
keys[21][5] = -0.0781920;
times[21][6] = 5.00000;
keys[21][6] = -0.0766580;
times[21][7] = 6.00000;
keys[21][7] = -0.0920820;

names.push_back("RShoulderPitch");
times[22].arraySetSize(7);
keys[22].arraySetSize(7);

times[22][0] = 1.08000;
keys[22][0] = 1.48649;
times[22][1] = 2.24000;
keys[22][1] = 1.60921;
times[22][2] = 2.60000;
keys[22][2] = 1.35917;
times[22][3] = 3.12000;
keys[22][3] = 1.41746;
times[22][4] = 4.16000;
keys[22][4] = 1.59847;
times[22][5] = 5.04000;
keys[22][5] = 1.63835;
times[22][6] = 6.04000;
keys[22][6] = 1.50021;

names.push_back("RShoulderRoll");
times[23].arraySetSize(7);
keys[23].arraySetSize(7);

times[23][0] = 1.08000;
keys[23][0] = -0.0230520;
times[23][1] = 2.24000;
keys[23][1] = -0.0706061;
times[23][2] = 2.60000;
keys[23][2] = -0.0199840;
times[23][3] = 3.12000;
keys[23][3] = -0.131966;
times[23][4] = 4.16000;
keys[23][4] = -0.118160;
times[23][5] = 5.04000;
keys[23][5] = -0.0230520;
times[23][6] = 6.04000;
keys[23][6] = -0.0352401;

names.push_back("RWristYaw");
times[24].arraySetSize(7);
keys[24].arraySetSize(7);

times[24][0] = 1.08000;
keys[24][0] = -0.244346;
times[24][1] = 2.24000;
keys[24][1] = -0.207132;
times[24][2] = 2.60000;
keys[24][2] = -0.239346;
times[24][3] = 3.12000;
keys[24][3] = -0.220938;
times[24][4] = 4.16000;
keys[24][4] = -0.202530;
times[24][5] = 5.04000;
keys[24][5] = -0.190258;
times[24][6] = 6.04000;
keys[24][6] = 0.127364;



try
{
  motion_proxy_ptr->angleInterpolation(names, keys, times, true);
}
catch(const std::exception&)
{

}

}

void Throw(AL::ALMotionProxy* motion_proxy_ptr)
{
// Choregraphe simplified export in c++.
// Add #include <alproxies/almotionproxy.h> at the beginning of this file.
	std::vector<std::string> names;
	AL::ALValue times, keys;
	names.reserve(14);
	times.arraySetSize(14);
	keys.arraySetSize(14);

	names.push_back("HeadPitch");
	times[0].arraySetSize(5);
	keys[0].arraySetSize(5);

	times[0][0] = 0.800000;
	keys[0][0] = -0.221657;
	times[0][1] = 1.16000;
	keys[0][1] = 0.107338;
	times[0][2] = 1.96000;
	keys[0][2] = 0.222388;
	times[0][3] = 2.76000;
	keys[0][3] = -0.179519;
	times[0][4] = 3.44000;
	keys[0][4] = 0.191986;

	names.push_back("HeadYaw");
	times[1].arraySetSize(3);
	keys[1].arraySetSize(3);

	times[1][0] = 1.16000;
	keys[1][0] = 0.124212;
	times[1][1] = 1.96000;
	keys[1][1] = 0.128814;
	times[1][2] = 2.76000;
	keys[1][2] = 0.00916204;

	names.push_back("LElbowRoll");
	times[2].arraySetSize(5);
	keys[2].arraySetSize(5);

	times[2][0] = 0.760000;
	keys[2][0] = -0.308923;
	times[2][1] = 1.12000;
	keys[2][1] = -1.10290;
	times[2][2] = 1.92000;
	keys[2][2] = -1.54462;
	times[2][3] = 2.72000;
	keys[2][3] = -1.54462;
	times[2][4] = 3.40000;
	keys[2][4] = -1.54462;

	names.push_back("LElbowYaw");
	times[3].arraySetSize(4);
	keys[3].arraySetSize(4);

	times[3][0] = 0.760000;
	keys[3][0] = -1.63014;
	times[3][1] = 1.12000;
	keys[3][1] = -1.41899;
	times[3][2] = 1.92000;
	keys[3][2] = -1.37451;
	times[3][3] = 2.72000;
	keys[3][3] = -1.18429;

	names.push_back("LHand");
	times[4].arraySetSize(4);
	keys[4].arraySetSize(4);

	times[4][0] = 0.560000;
	keys[4][0] = 0.00226893;
	times[4][1] = 0.750492;
	keys[4][1] = 0.954497;
	times[4][2] = 1.92000;
	keys[4][2] = 0.95;
	times[4][3] = 2.72000;
	keys[4][3] = 0.95;

// times[4][0] = 0.760000;
// keys[4][0] = 0.00226893;
// times[4][1] = 1.12000;
// keys[4][1] = 0.954497;
// times[4][2] = 1.92000;
// keys[4][2] = 0.95;
// times[4][3] = 2.72000;
// keys[4][3] = 0.95;

	names.push_back("LShoulderPitch");
	times[5].arraySetSize(4);
	keys[5].arraySetSize(4);

	times[5][0] = 0.760000;
	keys[5][0] = 0.331613;
	times[5][1] = 1.12000;
	keys[5][1] = -1.18651;
	times[5][2] = 1.92000;
	keys[5][2] = -0.429562;
	times[5][3] = 2.72000;
	keys[5][3] = 1.04921;

	names.push_back("LShoulderRoll");
	times[6].arraySetSize(4);
	keys[6].arraySetSize(4);

	times[6][0] = 0.760000;
	keys[6][0] = 0.193732;
	times[6][1] = 1.12000;
	keys[6][1] = 0.0137640;
	times[6][2] = 1.92000;
	keys[6][2] = 0.122678;
	times[6][3] = 2.72000;
	keys[6][3] = 0.0858620;


	names.push_back("LWristYaw");
	times[7].arraySetSize(4);
	keys[7].arraySetSize(4);

	times[7][0] = 1.08000;
	keys[7][0] = -1.12748;
	times[7][1] = 1.12000;
	keys[7][1] = -1.13980;
	times[7][2] = 1.92000;
	keys[7][2] = -1.13213;
	times[7][3] = 2.72000;
	keys[7][3] = -1.08765;

	names.push_back("RElbowRoll");
	times[8].arraySetSize(4);
	keys[8].arraySetSize(4);

	times[8][0] = 1.08000;
	keys[8][0] = 1.14441;
	times[8][1] = 1.88000;
	keys[8][1] = 1.13980;
	times[8][2] = 2.68000;
	keys[8][2] = 1.12753;
	times[8][3] = 3.36000;
	keys[8][3] = 0.982620;

	names.push_back("RElbowYaw");
	times[9].arraySetSize(4);
	keys[9].arraySetSize(4);

	times[9][0] = 1.08000;
	keys[9][0] = -0.00617796;
	times[9][1] = 1.88000;
	keys[9][1] = -4.19617e-05;
	times[9][2] = 2.68000;
	keys[9][2] = -0.0199840;
	times[9][3] = 3.36000;
	keys[9][3] = 0.226893;

	names.push_back("RHand");
	times[10].arraySetSize(4);
	keys[10].arraySetSize(4);

	times[10][0] = 1.08000;
	keys[10][0] = 0.000802851;
	times[10][1] = 1.88000;
	keys[10][1] = 0.000781907;
	times[10][2] = 2.68000;
	keys[10][2] = 0.000781907;
	times[10][3] = 3.36000;
	keys[10][3] = 0.00000;

	names.push_back("RShoulderPitch");
	times[11].arraySetSize(3);
	keys[11].arraySetSize(3);

	times[11][0] = 1.08000;
	keys[11][0] = 1.17969;
	times[11][1] = 1.88000;
	keys[11][1] = 1.19503;
	times[11][2] = 2.68000;
	keys[11][2] = 1.19810;

	names.push_back("RShoulderRoll");
	times[12].arraySetSize(4);
	keys[12].arraySetSize(4);

	times[12][0] = 1.08000;
	keys[12][0] = -0.566087;
	times[12][1] = 1.88000;
	keys[12][1] = -0.566087;
	times[12][2] = 2.68000;
	keys[12][2] = -0.570689;
	times[12][3] = 3.36000;
	keys[12][3] = -0.460767;

	names.push_back("RWristYaw");
	times[13].arraySetSize(3);
	keys[13].arraySetSize(3);

	times[13][0] = 1.08000;
	keys[13][0] = 0.486237;
	times[13][1] = 1.88000;
	keys[13][1] = 0.510779;
	times[13][2] = 2.68000;
	keys[13][2] = 0.681054;

	try
	{
		motion_proxy_ptr->angleInterpolation(names, keys, times, true);
	}
	catch(const std::exception&)
	{

	}

}



void Entangle(AL::ALMotionProxy* motion_proxy_ptr, bool hand)
{
// Choregraphe bezier export in c++.
// Add #include <alproxies/almotionproxy.h> at the beginning of this file.
	std::vector<std::string> names;
	AL::ALValue times, keys;
	names.reserve(26);
	times.arraySetSize(26);
	keys.arraySetSize(26);

	names.push_back("HeadPitch");
	times[0].arraySetSize(7);
	keys[0].arraySetSize(7);

	times[0][0] = 0.800000;
	keys[0][0] = ALVA(0.128814, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[0][1] = 1.60000;
	keys[0][1] = ALVA(0.128814, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[0][2] = 2.40000;
	keys[0][2] = ALVA(0.128814, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[0][3] = 3.20000;
	keys[0][3] = ALVA(0.130348, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[0][4] = 4.00000;
	keys[0][4] = ALVA(0.128814, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[0][5] = 4.80000;
	keys[0][5] = ALVA(0.130348, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[0][6] = 5.60000;
	keys[0][6] = ALVA(0.130348, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.00000, 0.00000));

	names.push_back("HeadYaw");
	times[1].arraySetSize(7);
	keys[1].arraySetSize(7);

	times[1][0] = 0.800000;
	keys[1][0] = ALVA(0.0183661, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[1][1] = 1.60000;
	keys[1][1] = ALVA(0.0183661, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[1][2] = 2.40000;
	keys[1][2] = ALVA(0.0183661, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[1][3] = 3.20000;
	keys[1][3] = ALVA(0.0168321, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[1][4] = 4.00000;
	keys[1][4] = ALVA(0.0183661, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[1][5] = 4.80000;
	keys[1][5] = ALVA(0.0168321, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[1][6] = 5.60000;
	keys[1][6] = ALVA(0.0168321, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.00000, 0.00000));

	names.push_back("LAnklePitch");
	times[2].arraySetSize(7);
	keys[2].arraySetSize(7);

	times[2][0] = 0.800000;
	keys[2][0] = ALVA(-1.18944, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[2][1] = 1.60000;
	keys[2][1] = ALVA(-1.18944, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[2][2] = 2.40000;
	keys[2][2] = ALVA(-1.18944, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[2][3] = 3.20000;
	keys[2][3] = ALVA(-1.18944, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[2][4] = 4.00000;
	keys[2][4] = ALVA(-1.18944, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[2][5] = 4.80000;
	keys[2][5] = ALVA(-1.18944, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[2][6] = 5.60000;
	keys[2][6] = ALVA(-1.18944, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.00000, 0.00000));

	names.push_back("LAnkleRoll");
	times[3].arraySetSize(7);
	keys[3].arraySetSize(7);

	times[3][0] = 0.800000;
	keys[3][0] = ALVA(0.0782759, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[3][1] = 1.60000;
	keys[3][1] = ALVA(0.0767419, ALVA(3, -0.266667, 0.000511328), ALVA(3, 0.266667, -0.000511328));
	times[3][2] = 2.40000;
	keys[3][2] = ALVA(0.0752079, ALVA(3, -0.266667, 0.00153398), ALVA(3, 0.266667, -0.00153398));
	times[3][3] = 3.20000;
	keys[3][3] = ALVA(0.0660040, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[3][4] = 4.00000;
	keys[3][4] = ALVA(0.0660040, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[3][5] = 4.80000;
	keys[3][5] = ALVA(0.0767419, ALVA(3, -0.266667, -0.00153399), ALVA(3, 0.266667, 0.00153399));
	times[3][6] = 5.60000;
	keys[3][6] = ALVA(0.0782759, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.00000, 0.00000));

	names.push_back("LElbowRoll");
	times[4].arraySetSize(7);
	keys[4].arraySetSize(7);

	times[4][0] = 0.800000;
	keys[4][0] = ALVA(-1.09370, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[4][1] = 1.60000;
	keys[4][1] = ALVA(-1.19188, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[4][2] = 2.40000;
	keys[4][2] = ALVA(-1.12745, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[4][3] = 3.20000;
	keys[4][3] = ALVA(-1.18881, ALVA(3, -0.266667, 0.0562467), ALVA(3, 0.266667, -0.0562467));
	times[4][4] = 4.00000;
	keys[4][4] = ALVA(-1.46493, ALVA(3, -0.266667, 0.00306809), ALVA(3, 0.266667, -0.00306809));
	times[4][5] = 4.80000;
	keys[4][5] = ALVA(-1.46800, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[4][6] = 5.60000;
	keys[4][6] = ALVA(-1.43118, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.00000, 0.00000));

	names.push_back("LElbowYaw");
	times[5].arraySetSize(7);
	keys[5].arraySetSize(7);

	times[5][0] = 0.800000;
	keys[5][0] = ALVA(-0.765508, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[5][1] = 1.60000;
	keys[5][1] = ALVA(-0.966462, ALVA(3, -0.266667, 0.0772114), ALVA(3, 0.266667, -0.0772114));
	times[5][2] = 2.40000;
	keys[5][2] = ALVA(-1.22878, ALVA(3, -0.266667, 0.00460194), ALVA(3, 0.266667, -0.00460194));
	times[5][3] = 3.20000;
	keys[5][3] = ALVA(-1.23338, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[5][4] = 4.00000;
	keys[5][4] = ALVA(-1.06004, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[5][5] = 4.80000;
	keys[5][5] = ALVA(-1.10606, ALVA(3, -0.266667, 0.0107380), ALVA(3, 0.266667, -0.0107380));
	times[5][6] = 5.60000;
	keys[5][6] = ALVA(-1.12446, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.00000, 0.00000));

	names.push_back("LHand");
	times[6].arraySetSize(7);
	keys[6].arraySetSize(7);

	times[6][0] = 0.800000;
	keys[6][0] = ALVA(0.00674395, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[6][1] = 1.60000;
	keys[6][1] = ALVA(0.00674395, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[6][2] = 2.40000;
	keys[6][2] = ALVA(0.00674395, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[6][3] = 3.20000;
	keys[6][3] = ALVA(0.00674395, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[6][4] = 4.00000;
	keys[6][4] = ALVA(0.00675093, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[6][5] = 4.80000;
	keys[6][5] = ALVA(0.00674395, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[6][6] = 5.60000;
	keys[6][6] = ALVA(0.00675093, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.00000, 0.00000));

	names.push_back("LHipPitch");
	times[7].arraySetSize(7);
	keys[7].arraySetSize(7);

	times[7][0] = 0.800000;
	keys[7][0] = ALVA(-0.740880, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[7][1] = 1.60000;
	keys[7][1] = ALVA(-0.753152, ALVA(3, -0.266667, 0.0122721), ALVA(3, 0.266667, -0.0122721));
	times[7][2] = 2.40000;
	keys[7][2] = ALVA(-0.874338, ALVA(3, -0.266667, 0.0611043), ALVA(3, 0.266667, -0.0611043));
	times[7][3] = 3.20000;
	keys[7][3] = ALVA(-1.11978, ALVA(3, -0.266667, 0.0475539), ALVA(3, 0.266667, -0.0475539));
	times[7][4] = 4.00000;
	keys[7][4] = ALVA(-1.16733, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[7][5] = 4.80000;
	keys[7][5] = ALVA(-0.949504, ALVA(3, -0.266667, -0.0613600), ALVA(3, 0.266667, 0.0613600));
	times[7][6] = 5.60000;
	keys[7][6] = ALVA(-0.799172, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.00000, 0.00000));

	names.push_back("LHipRoll");
	times[8].arraySetSize(7);
	keys[8].arraySetSize(7);

	times[8][0] = 0.800000;
	keys[8][0] = ALVA(-0.0797260, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[8][1] = 1.60000;
	keys[8][1] = ALVA(-0.0797260, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[8][2] = 2.40000;
	keys[8][2] = ALVA(-0.0889301, ALVA(3, -0.266667, 0.00562469), ALVA(3, 0.266667, -0.00562469));
	times[8][3] = 3.20000;
	keys[8][3] = ALVA(-0.113474, ALVA(3, -0.266667, 0.00511332), ALVA(3, 0.266667, -0.00511332));
	times[8][4] = 4.00000;
	keys[8][4] = ALVA(-0.119610, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[8][5] = 4.80000;
	keys[8][5] = ALVA(-0.108872, ALVA(3, -0.266667, -0.00511332), ALVA(3, 0.266667, 0.00511332));
	times[8][6] = 5.60000;
	keys[8][6] = ALVA(-0.0889301, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.00000, 0.00000));

	names.push_back("LHipYawPitch");
	times[9].arraySetSize(7);
	keys[9].arraySetSize(7);

	times[9][0] = 0.800000;
	keys[9][0] = ALVA(-0.248466, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[9][1] = 1.60000;
	keys[9][1] = ALVA(-0.250000, ALVA(3, -0.266667, 0.000511327), ALVA(3, 0.266667, -0.000511327));
	times[9][2] = 2.40000;
	keys[9][2] = ALVA(-0.251534, ALVA(3, -0.266667, 0.000511330), ALVA(3, 0.266667, -0.000511330));
	times[9][3] = 3.20000;
	keys[9][3] = ALVA(-0.253068, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[9][4] = 4.00000;
	keys[9][4] = ALVA(-0.253068, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[9][5] = 4.80000;
	keys[9][5] = ALVA(-0.246932, ALVA(3, -0.266667, -0.00613596), ALVA(3, 0.266667, 0.00613596));
	times[9][6] = 5.60000;
	keys[9][6] = ALVA(-0.205514, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.00000, 0.00000));

	names.push_back("LKneePitch");
	times[10].arraySetSize(7);
	keys[10].arraySetSize(7);

	times[10][0] = 0.800000;
	keys[10][0] = ALVA(2.11255, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[10][1] = 1.60000;
	keys[10][1] = ALVA(2.11255, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[10][2] = 2.40000;
	keys[10][2] = ALVA(2.11255, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[10][3] = 3.20000;
	keys[10][3] = ALVA(2.11255, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[10][4] = 4.00000;
	keys[10][4] = ALVA(2.11255, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[10][5] = 4.80000;
	keys[10][5] = ALVA(2.11255, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[10][6] = 5.60000;
	keys[10][6] = ALVA(2.11255, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.00000, 0.00000));

	names.push_back("LShoulderPitch");
	times[11].arraySetSize(7);
	keys[11].arraySetSize(7);

	times[11][0] = 0.800000;
	keys[11][0] = ALVA(1.42044, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[11][1] = 1.60000;
	keys[11][1] = ALVA(1.34834, ALVA(3, -0.266667, 0.0720980), ALVA(3, 0.266667, -0.0720980));
	times[11][2] = 2.40000;
	keys[11][2] = ALVA(0.944902, ALVA(3, -0.266667, 0.135759), ALVA(3, 0.266667, -0.135759));
	times[11][3] = 3.20000;
	keys[11][3] = ALVA(0.533790, ALVA(3, -0.266667, 0.138827), ALVA(3, 0.266667, -0.138827));
	times[11][4] = 4.00000;
	keys[11][4] = ALVA(0.111940, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[11][5] = 4.80000;
	keys[11][5] = ALVA(0.115008, ALVA(3, -0.266667, -0.00306797), ALVA(3, 0.266667, 0.00306797));
	times[11][6] = 5.60000;
	keys[11][6] = ALVA(0.225456, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.00000, 0.00000));

	names.push_back("LShoulderRoll");
	times[12].arraySetSize(7);
	keys[12].arraySetSize(7);

	times[12][0] = 0.800000;
	keys[12][0] = ALVA(0.0597839, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[12][1] = 1.60000;
	keys[12][1] = ALVA(0.477032, ALVA(3, -0.266667, -0.0866710), ALVA(3, 0.266667, 0.0866710));
	times[12][2] = 2.40000;
	keys[12][2] = ALVA(0.579810, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[12][3] = 3.20000;
	keys[12][3] = ALVA(0.469362, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[12][4] = 4.00000;
	keys[12][4] = ALVA(0.515382, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[12][5] = 4.80000;
	keys[12][5] = ALVA(0.450954, ALVA(3, -0.266667, 0.0360490), ALVA(3, 0.266667, -0.0360490));
	times[12][6] = 5.60000;
	keys[12][6] = ALVA(0.299088, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.00000, 0.00000));

	names.push_back("LWristYaw");
	times[13].arraySetSize(7);
	keys[13].arraySetSize(7);

	times[13][0] = 0.800000;
	keys[13][0] = ALVA(0.131882, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[13][1] = 1.60000;
	keys[13][1] = ALVA(0.124212, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[13][2] = 2.40000;
	keys[13][2] = ALVA(0.125746, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[13][3] = 3.20000;
	keys[13][3] = ALVA(-0.0614018, ALVA(3, -0.266667, 0.00460219), ALVA(3, 0.266667, -0.00460219));
	times[13][4] = 4.00000;
	keys[13][4] = ALVA(-0.0660040, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[13][5] = 4.80000;
	keys[13][5] = ALVA(-0.0644701, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[13][6] = 5.60000;
	keys[13][6] = ALVA(-0.0675380, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.00000, 0.00000));

	names.push_back("RAnklePitch");
	times[14].arraySetSize(7);
	keys[14].arraySetSize(7);

	times[14][0] = 0.800000;
	keys[14][0] = ALVA(-1.18630, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[14][1] = 1.60000;
	keys[14][1] = ALVA(-1.18630, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[14][2] = 2.40000;
	keys[14][2] = ALVA(-1.18630, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[14][3] = 3.20000;
	keys[14][3] = ALVA(-1.18630, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[14][4] = 4.00000;
	keys[14][4] = ALVA(-1.18630, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[14][5] = 4.80000;
	keys[14][5] = ALVA(-1.18630, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[14][6] = 5.60000;
	keys[14][6] = ALVA(-1.18630, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.00000, 0.00000));

	names.push_back("RAnkleRoll");
	times[15].arraySetSize(7);
	keys[15].arraySetSize(7);

	times[15][0] = 0.800000;
	keys[15][0] = ALVA(-0.0812600, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[15][1] = 1.60000;
	keys[15][1] = ALVA(-0.0812600, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[15][2] = 2.40000;
	keys[15][2] = ALVA(-0.0797260, ALVA(3, -0.266667, -0.00153399), ALVA(3, 0.266667, 0.00153399));
	times[15][3] = 3.20000;
	keys[15][3] = ALVA(-0.0705221, ALVA(3, -0.266667, -0.00204531), ALVA(3, 0.266667, 0.00204531));
	times[15][4] = 4.00000;
	keys[15][4] = ALVA(-0.0674541, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[15][5] = 4.80000;
	keys[15][5] = ALVA(-0.0797260, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[15][6] = 5.60000;
	keys[15][6] = ALVA(-0.0797260, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.00000, 0.00000));

	names.push_back("RElbowRoll");
	times[16].arraySetSize(7);
	keys[16].arraySetSize(7);

	times[16][0] = 0.800000;
	keys[16][0] = ALVA(1.07077, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[16][1] = 1.60000;
	keys[16][1] = ALVA(1.07231, ALVA(3, -0.266667, -0.00153398), ALVA(3, 0.266667, 0.00153398));
	times[16][2] = 2.40000;
	keys[16][2] = ALVA(1.10145, ALVA(3, -0.266667, -0.00460241), ALVA(3, 0.266667, 0.00460241));
	times[16][3] = 3.20000;
	keys[16][3] = ALVA(1.10606, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[16][4] = 4.00000;
	keys[16][4] = ALVA(1.10299, ALVA(3, -0.266667, 0.00306843), ALVA(3, 0.266667, -0.00306843));
	times[16][5] = 4.80000;
	keys[16][5] = ALVA(1.06310, ALVA(3, -0.266667, 0.00306796), ALVA(3, 0.266667, -0.00306796));
	times[16][6] = 5.60000;
	keys[16][6] = ALVA(1.06004, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.00000, 0.00000));

	names.push_back("RElbowYaw");
	times[17].arraySetSize(7);
	keys[17].arraySetSize(7);

	times[17][0] = 0.800000;
	keys[17][0] = ALVA(0.788434, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[17][1] = 1.60000;
	keys[17][1] = ALVA(0.789968, ALVA(3, -0.266667, -0.00153398), ALVA(3, 0.266667, 0.00153398));
	times[17][2] = 2.40000;
	keys[17][2] = ALVA(0.822182, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[17][3] = 3.20000;
	keys[17][3] = ALVA(0.822182, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[17][4] = 4.00000;
	keys[17][4] = ALVA(0.822182, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[17][5] = 4.80000;
	keys[17][5] = ALVA(0.822182, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[17][6] = 5.60000;
	keys[17][6] = ALVA(0.822182, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.00000, 0.00000));

	names.push_back("RHand");
	times[18].arraySetSize(7);
	keys[18].arraySetSize(7);

	times[18][0] = 0.800000;
	keys[18][0] = ALVA(0.000502655, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[18][1] = 1.60000;
	keys[18][1] = ALVA(0.000558504, ALVA(3, -0.266667, -3.72336e-05), ALVA(3, 0.266667, 3.72336e-05));
	times[18][2] = 2.40000;
	keys[18][2] = ALVA(0.000726057, ALVA(3, -0.266667, -0.000160571), ALVA(3, 0.266667, 0.000160571));
	times[18][3] = 3.20000;
	keys[18][3] = ALVA(0.00152193, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[18][4] = 4.00000;
	keys[18][4] = ALVA(0.00152193, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[18][5] = 4.80000;
	keys[18][5] = ALVA(0.00151495, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[18][6] = 5.60000;
	keys[18][6] = ALVA(0.00152193, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.00000, 0.00000));

	names.push_back("RHipPitch");
	times[19].arraySetSize(7);
	keys[19].arraySetSize(7);

	times[19][0] = 0.800000;
	keys[19][0] = ALVA(-0.742498, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[19][1] = 1.60000;
	keys[19][1] = ALVA(-0.756304, ALVA(3, -0.266667, 0.0138062), ALVA(3, 0.266667, -0.0138062));
	times[19][2] = 2.40000;
	keys[19][2] = ALVA(-0.882092, ALVA(3, -0.266667, 0.0634053), ALVA(3, 0.266667, -0.0634053));
	times[19][3] = 3.20000;
	keys[19][3] = ALVA(-1.13674, ALVA(3, -0.266667, 0.0460200), ALVA(3, 0.266667, -0.0460200));
	times[19][4] = 4.00000;
	keys[19][4] = ALVA(-1.18276, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[19][5] = 4.80000;
	keys[19][5] = ALVA(-0.949588, ALVA(3, -0.266667, -0.0634053), ALVA(3, 0.266667, 0.0634053));
	times[19][6] = 5.60000;
	keys[19][6] = ALVA(-0.802324, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.00000, 0.00000));

	names.push_back("RHipRoll");
	times[20].arraySetSize(7);
	keys[20].arraySetSize(7);

	times[20][0] = 0.800000;
	keys[20][0] = ALVA(0.0798099, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[20][1] = 1.60000;
	keys[20][1] = ALVA(0.0782759, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[20][2] = 2.40000;
	keys[20][2] = ALVA(0.0859461, ALVA(3, -0.266667, -0.00536899), ALVA(3, 0.266667, 0.00536899));
	times[20][3] = 3.20000;
	keys[20][3] = ALVA(0.110490, ALVA(3, -0.266667, -0.00153422), ALVA(3, 0.266667, 0.00153422));
	times[20][4] = 4.00000;
	keys[20][4] = ALVA(0.112024, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[20][5] = 4.80000;
	keys[20][5] = ALVA(0.101286, ALVA(3, -0.266667, 0.00511336), ALVA(3, 0.266667, -0.00511336));
	times[20][6] = 5.60000;
	keys[20][6] = ALVA(0.0813439, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.00000, 0.00000));

	names.push_back("RHipYawPitch");
	times[21].arraySetSize(7);
	keys[21].arraySetSize(7);

	times[21][0] = 0.800000;
	keys[21][0] = ALVA(-0.248466, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[21][1] = 1.60000;
	keys[21][1] = ALVA(-0.250000, ALVA(3, -0.266667, 0.000511327), ALVA(3, 0.266667, -0.000511327));
	times[21][2] = 2.40000;
	keys[21][2] = ALVA(-0.251534, ALVA(3, -0.266667, 0.000511330), ALVA(3, 0.266667, -0.000511330));
	times[21][3] = 3.20000;
	keys[21][3] = ALVA(-0.253068, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[21][4] = 4.00000;
	keys[21][4] = ALVA(-0.253068, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[21][5] = 4.80000;
	keys[21][5] = ALVA(-0.246932, ALVA(3, -0.266667, -0.00613596), ALVA(3, 0.266667, 0.00613596));
	times[21][6] = 5.60000;
	keys[21][6] = ALVA(-0.205514, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.00000, 0.00000));

	names.push_back("RKneePitch");
	times[22].arraySetSize(7);
	keys[22].arraySetSize(7);

	times[22][0] = 0.800000;
	keys[22][0] = ALVA(2.11255, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[22][1] = 1.60000;
	keys[22][1] = ALVA(2.11255, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[22][2] = 2.40000;
	keys[22][2] = ALVA(2.11255, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[22][3] = 3.20000;
	keys[22][3] = ALVA(2.11255, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[22][4] = 4.00000;
	keys[22][4] = ALVA(2.11255, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[22][5] = 4.80000;
	keys[22][5] = ALVA(2.11255, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[22][6] = 5.60000;
	keys[22][6] = ALVA(2.11255, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.00000, 0.00000));

	names.push_back("RShoulderPitch");
	times[23].arraySetSize(7);
	keys[23].arraySetSize(7);

	times[23][0] = 0.800000;
	keys[23][0] = ALVA(1.44047, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[23][1] = 1.60000;
	keys[23][1] = ALVA(1.44047, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[23][2] = 2.40000;
	keys[23][2] = ALVA(1.42820, ALVA(3, -0.266667, 0.0122718), ALVA(3, 0.266667, -0.0122718));
	times[23][3] = 3.20000;
	keys[23][3] = ALVA(1.25179, ALVA(3, -0.266667, 0.0337481), ALVA(3, 0.266667, -0.0337481));
	times[23][4] = 4.00000;
	keys[23][4] = ALVA(1.21804, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[23][5] = 4.80000;
	keys[23][5] = ALVA(1.22724, ALVA(3, -0.266667, -0.00920388), ALVA(3, 0.266667, 0.00920388));
	times[23][6] = 5.60000;
	keys[23][6] = ALVA(1.39138, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.00000, 0.00000));

	names.push_back("RShoulderRoll");
	times[24].arraySetSize(7);
	keys[24].arraySetSize(7);

	times[24][0] = 0.800000;
	keys[24][0] = ALVA(-0.0690720, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[24][1] = 1.60000;
	keys[24][1] = ALVA(-0.0690720, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[24][2] = 2.40000;
	keys[24][2] = ALVA(-0.0675380, ALVA(3, -0.266667, -0.000766992), ALVA(3, 0.266667, 0.000766992));
	times[24][3] = 3.20000;
	keys[24][3] = ALVA(-0.0644701, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[24][4] = 4.00000;
	keys[24][4] = ALVA(-0.0675380, ALVA(3, -0.266667, 0.000766992), ALVA(3, 0.266667, -0.000766992));
	times[24][5] = 4.80000;
	keys[24][5] = ALVA(-0.0690720, ALVA(3, -0.266667, 0.000511328), ALVA(3, 0.266667, -0.000511328));
	times[24][6] = 5.60000;
	keys[24][6] = ALVA(-0.0706060, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.00000, 0.00000));

	names.push_back("RWristYaw");
	times[25].arraySetSize(7);
	keys[25].arraySetSize(7);

	times[25][0] = 0.800000;
	keys[25][0] = ALVA(-0.0920820, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[25][1] = 1.60000;
	keys[25][1] = ALVA(-0.0920820, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[25][2] = 2.40000;
	keys[25][2] = ALVA(-0.0905480, ALVA(3, -0.266667, -0.00102266), ALVA(3, 0.266667, 0.00102266));
	times[25][3] = 3.20000;
	keys[25][3] = ALVA(-0.0859461, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[25][4] = 4.00000;
	keys[25][4] = ALVA(-0.0859461, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[25][5] = 4.80000;
	keys[25][5] = ALVA(-0.0874801, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.266667, 0.00000));
	times[25][6] = 5.60000;
	keys[25][6] = ALVA(-0.0874801, ALVA(3, -0.266667, -0.00000), ALVA(3, 0.00000, 0.00000));

	try
	{
	motion_proxy_ptr->angleInterpolationBezier(names, times, keys);
		// motion_proxy_ptr->angleInterpolation(names, keys, times, true);
	}
	catch(const std::exception&)
	{

	}

}
