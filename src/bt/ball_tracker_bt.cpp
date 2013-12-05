#include <nao_basic/ball_tracker_common.h>
#include <nao_basic/motions_common.h>
#include <nao_basic/robot_config.h>

#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Pose2D.h>
#include <behavior_trees/rosaction.h>

#include <iostream>
#include <unistd.h>
#include <math.h>

#include <alproxies/almotionproxy.h>
#include <alproxies/alvideodeviceproxy.h>
#include <alvision/alimage.h>
#include <alvision/alvisiondefinitions.h>
#include <alerror/alerror.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/features2d/features2d.hpp>

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

class BallTracker : ROSAction
{
public:

	ros::Duration execute_time_;
	bool init;

	AL::ALMotionProxy motionProxy;
	AL::ALVideoDeviceProxy camProxy;
	cv::Mat image;
	std::string clientName;

	BallTracker(std::string name, std::string robot_ip):
		ROSAction(name),
		execute_time_((ros::Duration) 0),
		init(false),
		motionProxy(robot_ip, 9559),
		camProxy(robot_ip, 9559)
		{
			// std::string robotIP = "192.168.0.198";
			std::cout << "Robot ip to use is: " << robot_ip << std::endl;
		}

	~BallTracker()
		{
			std::cout << "Called finalize on bt node" << std::endl;
			finalize();
		}


	void initialize()
		{
			sleep(1.0);
			camProxy.setParam(AL::kCameraSelectID, 1);
			// Subscribe a client image requiring 320*240 and BGR colorspace.
			clientName = camProxy.subscribe("BallTracker", AL::kQVGA, AL::kBGRColorSpace, 30);
			// Create an cv::Mat header to wrap into an opencv image.
			image = cv::Mat(cv::Size(320, 240), CV_8UC3);
			// refreshImage(camProxy, clientName, imgHeader);
			setwindowSettings();
			ball_tracker_initialize();
			set_feedback(RUNNING);
		}

	void finalize()
		{
			// disableStiffness();
			// Clean up before exiting!
			ball_tracker_finalize();
			camProxy.unsubscribe(clientName);
		}

	int executeCB(ros::Duration dt)
		{
			std::cout << "**Ball tracker -%- Executing Main Task, elapsed_time: "
			          << dt.toSec() << std::endl;
			std::cout << "**Ball tracker -%- execute_time: "
			          << execute_time_.toSec() << std::endl;
			execute_time_ += dt;

			if (!init) {
				initialize();
				init = true;
			}

			AL::ALValue img = camProxy.getImageRemote(clientName);
			image.data = (uchar*) img[6].GetBinary();
			camProxy.releaseImage(clientName);

			cv::Mat image_clone1 = image.clone();
			cv::Mat image_clone2 = image.clone();

			// std::cout << typeid(cv_ptr->image).name() << std::endl;

			// std::pair<int, int> ballPosCam = GetThresholdedImage(image_clone1, image_clone2,
			//                                                      blue_hue_l, blue_hue_h, blue_sat_l,
			//                                                      blue_sat_h, blue_val_l, blue_val_h,
			//                                                      CV_RGB(0,0,255));

			int pixels_ball = -1;
			// nr_pixels_ball_ptr = &init_ptr_value;
			std::pair<int,int> ballPosCam(-1, -1);
			if (hue_l_2 != -1 && hue_h_2 != -1 && sat_l_2 != -1 &&
			    sat_h_2 != -1 && val_l_2 != -1 && val_h_2 != -1)
			{

				// Two threshold ranges are used for example for red

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
				// Only one threshold range used
				ballPosCam = GetThresholdedImage(image_clone1, image_clone2,
				                                 hue_l, hue_h, sat_l,
				                                 sat_h, val_l, val_h,
				                                 CV_RGB(0,0,255),
				                                 &pixels_ball);
			}
			// If ball was found
			if (ballPosCam.first != -1)
			{
				// Publish ball position and size on ros topics
				std::pair<float, float> ballPosWorld = worldBallPosFromImgCoords(motionProxy, ballPosCam, 320, 240, 1);
				std::cout << "Ball pos in world: (" << ballPosWorld.first << ", " << ballPosWorld.second << ")" << std::endl;
				geometry_msgs::Pose2D msg;
				msg.x = ballPosWorld.first;
				msg.y = ballPosWorld.second;
				ball_pos_pub.publish(msg);
				std_msgs::Int8 size_msg;
				size_msg.data = pixels_ball;
				ball_size_pub.publish(size_msg);
				// Rotate head so that ball stays in view
				TrackBallWithHead(&motionProxy, ballPosCam.first, ballPosCam.second, 320, 240);
			}
			cv::imshow("Display Window"     , image);
			cv::imshow("Blob Window"        , image_clone1);
			cv::waitKey(30);
			return 0;
		}

	void resetCB()
		{
			execute_time_ = (ros::Duration) 0;
		}

};

int main(int argc, char** argv)
{
	std::cout << "Hello, world!" << std::endl;
	//Specify which options are available as cmd line arguments
	setupCmdLineReader();
	//Read robot ip from command line parameters (--robot_ip=192.168.0.100 for example)
	std::string robot_ip = readRobotIPFromCmdLine(argc, argv);
	//Read ball color from command line arguments
	std::string ball_color = readColorFromCmdLine(argc, argv);
	std::cout << "Using ball color: " << ball_color << std::endl;
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
	ros::init(argc, argv, bt_node_name); // name used for bt.txt
	ros::NodeHandle n;
	ball_pos_pub = n.advertise<geometry_msgs::Pose2D>("ball_pos", 1000);
	ball_size_pub = n.advertise<std_msgs::Int8>("ball_size", 1000);
	BallTracker server(ros::this_node::getName(), robot_ip);
	ros::spin();
	return 0;
}

void setupBallColor(std::string ball_color) {

	int red_hue_l = 0;
	int red_hue_h = 16;
	int red_sat_l = 152;
	int red_sat_h = 255;
	int red_val_l = 40;
	int red_val_h = 255;



	int red_hue_l_2 = 0;
	int red_hue_h_2 = 21;
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

	int blue_hue_l = 60;
	int blue_hue_h = 120;
	int blue_sat_l = 60;
	int blue_sat_h = 180;
	int blue_val_l = 40;
	int blue_val_h = 200;

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
