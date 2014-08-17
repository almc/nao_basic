#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Pose2D.h>

#include <behavior_trees/rosaction.h>
#include <nao_basic/ball_tracker_common.h>
#include <nao_basic/motions_common.h>
#include <nao_basic/robot_config.h>
#include <nao_basic/activity.h>
#include <nao_basic/confirmation.h>

#include <alerror/alerror.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/altexttospeechproxy.h>
#include <alproxies/alrobotpostureproxy.h>

#include <alproxies/alledsproxy.h>
#include <alproxies/alvideodeviceproxy.h>
#include <alvision/alimage.h>
#include <alvision/alvisiondefinitions.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <iostream>
#include <geometry_msgs/Pose2D.h>
#include <unistd.h>
#include <math.h>

ros::Publisher confirmation_pub;
ros::Publisher ball_pos_pub;
ros::Publisher ball_size_pub;

int hue_l = 0;
int hue_h = 43;
int sat_l = 113;
int sat_h = 255;
int val_l = 128;
int val_h = 255;

int hue_l_2 = -1;
int hue_h_2 = -1;
int sat_l_2 = -1;
int sat_h_2 = -1;
int val_l_2 = -1;
int val_h_2 = -1;

void setupBallColor(std::string ball_color);

class ActionDispatcher : ROSAction
{
public:
	bool init_;
	ros::Duration execute_time_;
	ros::Time msg_timestamp_;
	ros::Time time_at_pos_;
	geometry_msgs::Pose2D goal_position_;
	// geometry_msgs::Pose2D last_ball_pos_;
	AL::ALMotionProxy* motion_proxy_ptr;
	AL::ALRobotPostureProxy* posture_proxy_ptr;
	AL::ALLedsProxy* leds_proxy_ptr;
	AL::ALTextToSpeechProxy* speech_proxy_ptr;
	AL::ALVideoDeviceProxy* cam_proxy_ptr;
	cv::Mat image;
	std::string clientName;

	int closeness_count;
	std::string activity_name_;

	ActionDispatcher(std::string name, std::string robot_ip):
		ROSAction(name),
		init_(false),
		execute_time_((ros::Duration) 0),
		msg_timestamp_((ros::Time) 0),
		closeness_count(0),
		activity_name_("none")	// action_name_ belong to parent class
		{
			std::cout << "Robot ip to use is: " << robot_ip << std::endl;
			motion_proxy_ptr  = new AL::ALMotionProxy(robot_ip, 9559);
			posture_proxy_ptr = new AL::ALRobotPostureProxy(robot_ip, 9559);
			leds_proxy_ptr    = new AL::ALLedsProxy(robot_ip, 9559);
			speech_proxy_ptr  = new AL::ALTextToSpeechProxy(robot_ip, 9559);
			cam_proxy_ptr     = new AL::ALVideoDeviceProxy(robot_ip, 9559);
		}

	~ActionDispatcher()
		{
			std::cout << "*** Destructor(): " << std::endl;
			delete motion_proxy_ptr;
			delete posture_proxy_ptr;
			delete leds_proxy_ptr;
			delete speech_proxy_ptr;
		}

	void initialize_camera()
		{
			sleep(1.0);
			cam_proxy_ptr->setParam(AL::kCameraSelectID, 1);
			// Subscribe a client image requiring 320*240 and BGR colorspace.
			clientName = cam_proxy_ptr->subscribe("BallTracker", AL::kQVGA, AL::kBGRColorSpace, 30);
			// Create an cv::Mat header to wrap into an opencv image.
			image = cv::Mat(cv::Size(320, 240), CV_8UC3);
			// refreshImage(cam_proxy_ptr, clientName, imgHeader);
			setwindowSettings();
			ball_tracker_initialize();
		}

	void initialize()
		{
			std::cout << "*** Initialize(): " << std::endl;
			sleep(1.0);
			std::cout << "initializing now" << std::endl;
			//Set the stiffness so the robot can move
			AL::ALValue stiffness_name("Body");
			AL::ALValue stiffness(1.0f);
			AL::ALValue stiffness_time(1.0f);
			motion_proxy_ptr->stiffnessInterpolation(stiffness_name,
			                                         stiffness,
			                                         stiffness_time);
			motion_proxy_ptr->moveInit();
			initialize_camera();
			set_feedback(RUNNING);
		}

	void finalize()
		{
			// ball_tracker_finalize();
			cam_proxy_ptr->unsubscribe(clientName);
			std::cout << "*** Finalize(): " << std::endl;
			closeness_count = 0;
			init_ = false;
			deactivate();
		}


	int Crouch_Function()
		{
			std::cout << "*** Crouch_Function(): " << std::endl;
			posture_proxy_ptr->goToPosture("Crouch", 0.8);
			float duration = 30.0f;
			leds_proxy_ptr->rasta(duration);
			posture_proxy_ptr->goToPosture("StandInit", 1.0);
			nao_basic::confirmation msg_conf;
			msg_conf.name = "crouch";
			msg_conf.done = 1;
			confirmation_pub.publish(msg_conf);
			activity_name_ = "none";
			return 0;
		}

	int Idle_Function()
		{
			std::cout << "*** Idle_Function(): " << std::endl;
			// nao_basic::confirmation msg_conf;
			// msg_conf.name = "idle";
			// msg_conf.done = 1;
			// confirmation_pub.publish(msg_conf);
			// activity_name_ = "none";
			return 0;
		}

	int GotoWP_Function()
		{
			std::cout << "*** GotoWP_Function(): " << std::endl;
			AL::ALValue walk_config;
			// walk_config.arrayPush(AL::ALValue::array("MaxStepFrequency", frequency));
			walk_config.arrayPush(AL::ALValue::array("StepHeight", 0.005));
			if (fabs (goal_position_.y) < 0.10)
			{
				// std::cout << "case1: moving to " << goal_position_ << std::endl;
				motion_proxy_ptr->post.moveTo(goal_position_.x, 0.0, 0.0, walk_config);
				sleep(1.0);
			}
			else
			{
				// std::cout << "case2: moving to " << goal_position_ << std::endl;
				motion_proxy_ptr->post.moveTo(0.0, 0.0,
				                              atan2(goal_position_.y, goal_position_.x),
				                              walk_config);
			}
			activity_name_ = "none";
			return 0;
		}

	int ThrowBall_Function()
		{
			std::cout << "*** ThrowBall_Function(): " << std::endl;
			sleep(2.0);
			Throw(motion_proxy_ptr);
			sleep(2.0);
			nao_basic::confirmation msg_conf;
			msg_conf.name = "throw";
			msg_conf.done = 1;
			confirmation_pub.publish(msg_conf);
			activity_name_ = "none";
			return 0;
		}

	int DetectBall_Function(geometry_msgs::Pose2D& ball_pos)
		{
			std::cout << "*** DetectBall_Function(): " << std::endl;
			AL::ALValue img = cam_proxy_ptr->getImageRemote(clientName);
			image.data = (uchar*) img[6].GetBinary();
			cam_proxy_ptr->releaseImage(clientName);
			cv::Mat image_clone1 = image.clone();
			cv::Mat image_clone2 = image.clone();
			int pixels_ball = -1;
			std::pair<int,int> ballPosCam(-1, -1);
			if (hue_l_2 != -1 && hue_h_2 != -1 && sat_l_2 != -1 &&
			    sat_h_2 != -1 && val_l_2 != -1 && val_h_2 != -1)
			{
				ballPosCam = GetThresholdedImage(image_clone1, image_clone2,
				                                 hue_l, hue_h, sat_l,
				                                 sat_h, val_l, val_h,
				                                 CV_RGB(0,0,255),
				                                 &pixels_ball,
				                                 hue_l_2, hue_h_2,
				                                 sat_l_2, sat_h_2,
				                                 val_l_2, val_h_2);
			}
			else
			{
				ballPosCam = GetThresholdedImage(image_clone1, image_clone2,
				                                 hue_l, hue_h, sat_l,
				                                 sat_h, val_l, val_h,
				                                 CV_RGB(0,0,255),
				                                 &pixels_ball);
			}
			if (ballPosCam.first != -1) // found something
			{
				// publish ball position and size on ros topics
				std::pair<float, float> ballPosWorld =
					worldBallPosFromImgCoords(*motion_proxy_ptr, ballPosCam, 320, 240, 1);
				// std::cout << "Ball pos in world: ("
				//           << ballPosWorld.first << ", "
				//           << ballPosWorld.second << ")" << std::endl;
				geometry_msgs::Pose2D msg;
				ball_pos.x = ballPosWorld.first;
				ball_pos.y = ballPosWorld.second;
				// ball_pos_pub.publish(msg);
				// std_msgs::Int8 size_msg;
				// size_msg.data = pixels_ball;
				// ball_size_pub.publish(size_msg);
				// std::cout << "pixels ball" << pixels_ball << std::endl;
				// rotate head so that ball stays in view
				if (pixels_ball > 8) // found something big enough
					TrackBallWithHead(motion_proxy_ptr, ballPosCam.first, ballPosCam.second, 320, 240);
				else                 // found something not big enough
					TrackBallWithHead(motion_proxy_ptr, 320/2, 240/2, 320, 240);
			}
			else                     // found nothing
			{
				TrackBallWithHead(motion_proxy_ptr, 320/2, 240/2, 320, 240);
			}
			cv::imshow("Display Window"     , image);
			cv::imshow("Blob Window"        , image_clone1);
			cv::waitKey(30);
			return 0;
		}

	int GraspBall_Function()
		{
			geometry_msgs::Pose2D ball_pos;
			DetectBall_Function(ball_pos);
			std::cout << "*** GraspBall_Function(): " << std::endl;
			// if ((ros::Time::now() - time_at_pos_).toSec() < 0.2)
			if (true)
			{
				float goal_x = 0.15;
				float goal_y = 0.09;
				float error_x = ball_pos.x - goal_x;
				float error_y = ball_pos.y - goal_y;
				if (fabs(error_x) < 0.05 && fabs(error_y) < 0.05)
				{
					std::cout << "Closeness count " << closeness_count << std::endl;
					closeness_count++;
					if (closeness_count > 10)
					{
						motion_proxy_ptr->stopMove();
						// ****************************************
						OpenHand(motion_proxy_ptr);
						Bend2(motion_proxy_ptr);
						MoveHand2(motion_proxy_ptr);
						CloseHand(motion_proxy_ptr);
						if (CheckLHand(motion_proxy_ptr))
						{
							speech_proxy_ptr->say("I have it, I rock");
							UnBend2(motion_proxy_ptr);
							// confirmation
							nao_basic::confirmation msg_conf;
							msg_conf.name = "grasp";
							msg_conf.done = 1;
							confirmation_pub.publish(msg_conf);
							activity_name_ = "none";
							TrackBallWithHead(motion_proxy_ptr, 320/2, 240/2, 320, 240);
							return 0;
						}
						else
						{
							speech_proxy_ptr->say("I missed it, bad luck.");
							UnBend2(motion_proxy_ptr);
							MoveBack(motion_proxy_ptr);
							activity_name_ = "grasp";
							closeness_count = 0;
							TrackBallWithHead(motion_proxy_ptr, 320/2, 240/2, 320, 240);
							return 0;
						}
						// ****************************************
					}
					// return 0;
				}
				else
				{
					closeness_count = 0;
				}
				error_x = error_x >  0.6 ?  0.6 : error_x;
				error_x = error_x < -0.6 ? -0.6 : error_x;
				error_y = error_y >  0.6 ?  0.6 : error_y;
				error_y = error_y < -0.6 ? -0.6 : error_y;
				AL::ALValue walk_config;
				float frequency = 0.1/(5*closeness_count+(1.0/(fabs(error_x)+fabs(error_y))));
				walk_config.arrayPush(AL::ALValue::array("MaxStepFrequency", frequency));
				walk_config.arrayPush(AL::ALValue::array("StepHeight", 0.01));
				motion_proxy_ptr->post.moveTo(error_x, error_y, 0.0, walk_config);
			}
			else
			{
				std::cout << "My ball position is too old, cant use it" << std::endl;
			}
			set_feedback(RUNNING);
			return 0;
		}

	int executeCB(ros::Duration dt)
		{
			if (!init_)
			{
				initialize();
				init_ = true;
			}
			if (!activity_name_.compare("crouch"))
			{
				std::cout << "Received Crouch" << std::endl;
				set_feedback(RUNNING);
				return Crouch_Function();
			}
			else if (!activity_name_.compare("idle"))
			{
				std::cout << "Received Idle" << std::endl;
				set_feedback(RUNNING);
				return Idle_Function();
			}
			else if (!activity_name_.compare("goto"))
			{
				std::cout << "Received GotoWP" << std::endl;
				set_feedback(RUNNING);
				return GotoWP_Function();
			}
			else if (!activity_name_.compare("grasp"))
			{
				std::cout << "Received Grasp" << std::endl;
				set_feedback(RUNNING);
				return GraspBall_Function();
			}
			else if (!activity_name_.compare("throw"))
			{
				std::cout << "Received Throw" << std::endl;
				set_feedback(RUNNING);
				return ThrowBall_Function();
			}
			else if (!activity_name_.compare("none"))
			{
				std::cout << "Received Nothing" << std::endl;
				set_feedback(RUNNING);
				return 0;
			}
			else
			{
				set_feedback(RUNNING);
				activity_name_ = "none";
				return 0;
			}
		}

	void resetCB()
		{
			std::cout << "*** ResetCB(): " << std::endl;
			execute_time_ = (ros::Duration) 0;
		}

	void MessageReceivedCB(const nao_basic::activity::ConstPtr &msg)
		{
			std::cout << "Received message from LTL script" << std::endl;
			msg_timestamp_   = ros::Time::now();
			activity_name_   = msg->type;
			goal_position_.x = 0.01*msg->x;
			goal_position_.y = 0.01*msg->y;
			std::cout << activity_name_   << std::endl;
			std::cout << goal_position_.x << std::endl;
			std::cout << goal_position_.y << std::endl;
		}

	// void BallPosReceivedCB(const geometry_msgs::Pose2D::ConstPtr &msg)
	// 	{
	// 		std::cout << "Received ball position" << std::endl;
	// 		time_at_pos_ = ros::Time::now();
	// 		last_ball_pos_.x = msg->x;
	// 		last_ball_pos_.y = msg->y;
	// 	}

};

int main(int argc, char** argv)
{
	std::cout << "Hello, world!" << std::endl;
	// specify which options are available as cmd line arguments
	setupCmdLineReader();
	// read agent id from command line parameters (--agent=mario)
	std::string agent = readAgentFromCmdLine(argc, argv);
	// read robot ip from command line parameters (--robot_ip=mario.local)
	std::string robot_ip = readRobotIPFromCmdLine(argc, argv);
	// read ball color from command line arguments (--color=red)
	std::string ball_color = readColorFromCmdLine(argc, argv);
	setupBallColor(ball_color);
	std::string bt_node_name = "BallTrackerRed";
	if (ball_color == "red") {
		bt_node_name = "BallTrackerRed";
	} else if (ball_color == "green") {
		bt_node_name = "BallTrackerGreen";
	} else if (ball_color == "blue") {
		bt_node_name = "BallTrackerBlue";
	} else if (ball_color == "yellow") {
		bt_node_name = "BallTrackerYellow";
	}
	ros::init(argc, argv, std::string("ActionDispatcher") + "_" + agent); // name used for bt.txt
	ActionDispatcher server(ros::this_node::getName(), robot_ip);
	ros::NodeHandle n;
	ros::Subscriber waypoint_sub =
		n.subscribe<nao_basic::activity>(std::string("next_move") + "_" + agent,
		                                 1, &ActionDispatcher::MessageReceivedCB, &server);
	// ros::Subscriber ball_pos_sub =
	// 	n.subscribe<geometry_msgs::Pose2D>(std::string("ball_pos") + "_" + agent,
	// 	                                   1, &ActionDispatcher::BallPosReceivedCB, &server);
	// ball_pos_pub  =
	// 	n.advertise<geometry_msgs::Pose2D>(std::string("ball_pos") + "_" + agent, 1000);
	// ball_size_pub =
	// 	n.advertise<std_msgs::Int8>(std::string("ball_size") + "_" + agent, 1000);
	confirmation_pub =
		n.advertise<nao_basic::confirmation>(std::string("activity_done") + "_" + agent, 10);
	ros::spin();
	return 0;
}

void setupBallColor(std::string ball_color)
{
	int red_hue_l = 84;
	int red_hue_h = 180;
	int red_sat_l = 158;
	int red_sat_h = 255;
	int red_val_l = 88;
	int red_val_h = 256;

	int red_hue_l_2 = 0;
	int red_hue_h_2 = 0;
	int red_sat_l_2 = red_sat_l;
	int red_sat_h_2 = red_sat_h;
	int red_val_l_2 = red_val_l;
	int red_val_h_2 = red_val_h;

	int green_hue_l = 40;
	int green_hue_h = 95;
	int green_sat_l = 33;
	int green_sat_h = 202;
	int green_val_l = 42;
	int green_val_h = 130;

	int blue_hue_l = 74;
	int blue_hue_h = 114;
	int blue_sat_l = 96;
	int blue_sat_h = 233;
	int blue_val_l = 61;
	int blue_val_h = 158;

	int yellow_hue_l = 20;
	int yellow_hue_h = 80;
	int yellow_sat_l = 120;
	int yellow_sat_h = 256;
	int yellow_val_l = 20;
	int yellow_val_h = 256;

	if (ball_color == "red")
	{
		hue_l = red_hue_l;
		hue_h = red_hue_h;
		sat_l = red_sat_l;
		sat_h = red_sat_h;
		val_l = red_val_l;
		val_h = red_val_h;
		// two ranges for HUE are needed (discontinuity)
		hue_l_2 = red_hue_l_2;
		hue_h_2 = red_hue_h_2;
		sat_l_2 = red_sat_l_2;
		sat_h_2 = red_sat_h_2;
		val_l_2 = red_val_l_2;
		val_h_2 = red_val_h_2;
	}
	else if (ball_color == "green")
	{
		hue_l = green_hue_l;
		hue_h = green_hue_h;
		sat_l = green_sat_l;
		sat_h = green_sat_h;
		val_l = green_val_l;
		val_h = green_val_h;
	}
	else if (ball_color == "blue")
	{
		hue_l = blue_hue_l;
		hue_h = blue_hue_h;
		sat_l = blue_sat_l;
		sat_h = blue_sat_h;
		val_l = blue_val_l;
		val_h = blue_val_h;
	}
	else if (ball_color == "yellow")
	{
		hue_l = yellow_hue_l;
		hue_h = yellow_hue_h;
		sat_l = yellow_sat_l;
		sat_h = yellow_sat_h;
		val_l = yellow_val_l;
		val_h = yellow_val_h;
	}
	else
	{
		hue_l = red_hue_l;
		hue_h = red_hue_h;
		sat_l = red_sat_l;
		sat_h = red_sat_h;
		val_l = red_val_l;
		val_h = red_val_h;
	}
}
