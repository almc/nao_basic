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
#include <alerror/alerror.h>



class BallGrasper : ROSAction
{
public:
	bool init_;
	bool has_closed_hand_;
	bool in_check_hand_pose_;
	ros::Duration execute_time_;
	ros::Time time_at_pos_;
	geometry_msgs::Pose2D last_ball_pos_;
	int last_ball_size_;
	AL::ALMotionProxy* motion_proxy_ptr;

	BallGrasper(std::string name, std::string robot_ip):
		ROSAction(name),
		init_(false),
		has_closed_hand_(false),
		in_check_hand_pose_(false),
		execute_time_((ros::Duration) 0),
		time_at_pos_((ros::Time) 0),
		last_ball_size_(0)
		{
			// std::string robotIP = "192.168.0.198";
			std::cout << "Robot ip to use is: " << robot_ip << std::endl;
			motion_proxy_ptr = new AL::ALMotionProxy(robot_ip, 9559);
		}

	~BallGrasper()
		{
			delete motion_proxy_ptr;
		}

	void initialize()
		{
			sleep(1.0);
			set_feedback(RUNNING);
		}

	void finalize()
		{
			has_closed_hand_ = false;
			in_check_hand_pose_ = false;
		}

	int executeCB(ros::Duration dt)
		{
			std::cout << "**Grasper -%- Executing Main Task, elapsed_time: "
			          << dt.toSec() << std::endl;
			std::cout << "**Grasper -%- execute_time: "
			          << execute_time_.toSec() << std::endl;
			execute_time_ += dt;

			if (!init_)
			{
				initialize();
				init_ = true;
			}

			if (!has_closed_hand_)
			{
				sleep(2.0);
				CloseHand(motion_proxy_ptr);
				has_closed_hand_ = true;
			}
			else if (!in_check_hand_pose_)
			{
				//Move the hand to a position in front of nao camera.
				CheckHand(motion_proxy_ptr);
				in_check_hand_pose_ = true;
			}
			else if (in_check_hand_pose_)
			{
				if ( (ros::Time::now() - time_at_pos_).toSec() < 0.2)
				{
					std::cout << "Ball radius : " << last_ball_size_ << std::endl;
					//If ball is seen as big enough, it is in hand.
					if (last_ball_size_ > 80)
					{
						set_feedback(SUCCESS);
						return 1;
					}
					else
					{
						OpenHand(motion_proxy_ptr);
						has_closed_hand_ = false;
						in_check_hand_pose_ = false;
					}
				}
				OpenHand(motion_proxy_ptr);
				has_closed_hand_ = false;
				in_check_hand_pose_ = false;
				set_feedback(FAILURE);
				return 1;
			}
			set_feedback(RUNNING);
			return 0;
		}

	void resetCB()
		{
			execute_time_ = (ros::Duration) 0;
		}

	void BallPosReceived(const geometry_msgs::Pose2D::ConstPtr &msg)
		{
			// std::cout << "Received ball position!" << std::endl;
			time_at_pos_ = ros::Time::now();
			last_ball_pos_.x = msg->x;
			last_ball_pos_.y = msg->y;
		}

	void BallSizeReceived(const std_msgs::Int8::ConstPtr &msg)
		{
			last_ball_size_ = msg->data;
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
	ros::init(argc, argv, std::string("BallGrasper") + "_" + agent); // name used for bt.txt
	BallGrasper server(ros::this_node::getName(), robot_ip);
	ros::NodeHandle n;
	ros::Subscriber ball_pos_sub =
		n.subscribe<geometry_msgs::Pose2D>(std::string("ball_pos") + "_" + agent, 1,
		                                   &BallGrasper::BallPosReceived, &server);
	ros::Subscriber ball_size_sub =
		n.subscribe<std_msgs::Int8>(std::string("ball_size") + "_" + agent, 1,
		                            &BallGrasper::BallSizeReceived, &server);
	ros::spin();
	return 0;
}
