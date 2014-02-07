#include <nao_basic/motions_common.h>
#include <nao_basic/robot_config.h>

#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Pose2D.h>
#include <behavior_trees/rosaction.h>

#include <iostream>
#include <unistd.h>
#include <math.h>

#include <alerror/alerror.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/altexttospeechproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <alproxies/alledsproxy.h>


class LedShiner : ROSAction
{
public:
bool init_;
ros::Duration execute_time_;
ros::Time time_at_pos_;
	AL::ALMotionProxy* motion_proxy_ptr;
	AL::ALRobotPostureProxy* posture_proxy_ptr;
	AL::ALLedsProxy* leds_proxy_ptr;
	AL::ALTextToSpeechProxy* speech_proxy_ptr;

	LedShiner(std::string name, std::string robot_ip):
		ROSAction(name),
		init_(false),
		//has_bent_(false),
		//has_moved_hand_(false),
		execute_time_((ros::Duration) 0),
		time_at_pos_((ros::Time) 0)
		{
			// std::string robotIP = "192.168.0.198";
			std::cout << "Robot ip to use is: " << robot_ip << std::endl;
			motion_proxy_ptr  = new AL::ALMotionProxy(robot_ip, 9559);
			posture_proxy_ptr = new AL::ALRobotPostureProxy(robot_ip, 9559);
			leds_proxy_ptr    = new AL::ALLedsProxy(robot_ip, 9559);
			speech_proxy_ptr  = new AL::ALTextToSpeechProxy(robot_ip, 9559);
		}

	~LedShiner()
		{
			delete motion_proxy_ptr;
		}

	void initialize()
		{
			sleep(1.0);
			//Set the stiffness so the robot can move
			AL::ALValue stiffness_name("Body");
			AL::ALValue stiffness(1.0f);
			AL::ALValue stiffness_time(1.0f);
			motion_proxy_ptr->stiffnessInterpolation(stiffness_name,
			                                         stiffness,
			                                         stiffness_time);
			motion_proxy_ptr->moveInit();
			set_feedback(RUNNING);
		}

	void finalize()
		{
			init_ = false;
			deactivate();
		}

	int executeCB(ros::Duration dt)
		{
			std::cout << "**LedShiner -%- Executing Main Task, elapsed_time: "
			          << dt.toSec() << std::endl;
			std::cout << "**LedShiner -%- execute_time: "
			          << execute_time_.toSec() << std::endl;
			execute_time_ += dt;

			if (!init_)
			{
				initialize();
				init_ = true;
				posture_proxy_ptr->goToPosture("Crouch", 0.8);
				float duration = 3.0f;
				leds_proxy_ptr->rasta(duration);
				posture_proxy_ptr->goToPosture("StandInit", 1.0);
				set_feedback(SUCCESS);
				finalize();
				return 1;
			}
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
	ros::init(argc, argv, "Cycle"); // name used for bt.txt
	//Read robot ip from command line parameters (--robot_ip=192.168.0.100 for example)
	setupCmdLineReader();
	std::string robot_ip = readRobotIPFromCmdLine(argc, argv);
	LedShiner server(ros::this_node::getName(), robot_ip);
	//ros::NodeHandle n;
	// ros::Subscriber ball_pos_sub = n.subscribe<geometry_msgs::Pose2D>("ball_pos", 1,
	//                                                                   &HandMover::BallPosReceived,
	//                                                                   &server);
	ros::spin();
	return 0;
}
