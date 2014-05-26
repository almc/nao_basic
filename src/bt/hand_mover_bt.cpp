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

#include <alproxies/almotionproxy.h>
#include <alproxies/altexttospeechproxy.h>
#include <alerror/alerror.h>



class HandMover : ROSAction
{
public:
	bool init_;
	bool has_bent_;
	bool has_moved_hand_;
	ros::Duration execute_time_;
	ros::Time time_at_pos_;
	geometry_msgs::Pose2D last_ball_pos_;
	geometry_msgs::Pose2D static_ball_pos_;
	AL::ALMotionProxy* motion_proxy_ptr;
	AL::ALTextToSpeechProxy* speech_proxy_ptr;

	HandMover(std::string name, std::string robot_ip):
		ROSAction(name),
		init_(false),
		has_bent_(false),
		has_moved_hand_(false),
		execute_time_((ros::Duration) 0),
		time_at_pos_((ros::Time) 0)
		{
			// std::string robotIP = "192.168.0.198";
			std::cout << "Robot ip to use is: " << robot_ip << std::endl;
			motion_proxy_ptr = new AL::ALMotionProxy(robot_ip, 9559);
			speech_proxy_ptr = new AL::ALTextToSpeechProxy(robot_ip, 9559);
		}

	~HandMover()
		{
			delete motion_proxy_ptr;
			delete speech_proxy_ptr;
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
			has_bent_ = false;
			has_moved_hand_ = false;
			init_ = false;
			// deactivate();
		}

	int executeCB(ros::Duration dt)
		{
			std::cout << "**HandMover -%- Executing Main Task, elapsed_time: "
			          << dt.toSec() << std::endl;
			std::cout << "**HandMover -%- execute_time: "
			          << execute_time_.toSec() << std::endl;
			execute_time_ += dt;

			if (!init_)
			{
				initialize();
				init_ = true;
				has_bent_ = false;
				busy = true;
				OpenHand(motion_proxy_ptr);
				Bend2(motion_proxy_ptr);
				MoveHand2(motion_proxy_ptr);
				CloseHand(motion_proxy_ptr);
				if (CheckLHand(motion_proxy_ptr))
				{
					speech_proxy_ptr->say("I have it, I rock");
					UnBend2(motion_proxy_ptr);
					set_feedback(SUCCESS);
					finalize();
					return 1;
				}
				else
				{
					speech_proxy_ptr->say("I miss it, bad luck.");
					UnBend2(motion_proxy_ptr);
					// MoveBack(motion_proxy_ptr);
					set_feedback(FAILURE);
					finalize();
					return 1;
				}

			}
			set_feedback(RUNNING); // this never happens
			return 0;
		}

	void resetCB()
		{
			execute_time_ = (ros::Duration) 0;
		}

	void BallPosReceived(const geometry_msgs::Pose2D::ConstPtr &msg)
		{
			time_at_pos_ = ros::Time::now();
			last_ball_pos_.x = msg->x;
			last_ball_pos_.y = msg->y;
		}
};

int main(int argc, char** argv)
{
	std::cout << "Hello, world!" << std::endl;
	// specify which options are available as cmd line arguments
	setupCmdLineReader();
	// read agent id from command line parameters (--agent=mario)
	std::string agent = readAgentFromCmdLine(argc, argv);
	// read robot ip from command line parameters (--robot_ip=192.168.0.100 for example)
	std::string robot_ip = readRobotIPFromCmdLine(argc, argv);
	ros::init(argc, argv, std::string("HandMover") + "_" + agent); // name used for bt.txt
	HandMover server(ros::this_node::getName(), robot_ip);
	ros::NodeHandle n;
	ros::Subscriber ball_pos_sub =
		n.subscribe<geometry_msgs::Pose2D>(std::string("ball_pos") + "_" + agent, 1,
		                                   &HandMover::BallPosReceived, &server);
	ros::spin();
	return 0;
}
