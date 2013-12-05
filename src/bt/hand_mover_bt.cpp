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
		}

	~HandMover()
		{}

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
			delete motion_proxy_ptr;
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
			}
			if (!has_bent_)
			{
				Bend(motion_proxy_ptr);
				has_bent_ = true;
				static_ball_pos_ = last_ball_pos_;
				set_feedback(RUNNING);
				return 0;
				// AL::ALValue stiffness_name("RArm");
				// AL::ALValue stiffness(0.0f);
				// AL::ALValue stiffness_time(1.0f);
				// motion_proxy_ptr->stiffnessInterpolation(stiffness_name,
				// 										stiffness,
				// 										stiffness_time);

			}
			if ( has_bent_ /*&& (ros::Time::now() - time_at_pos_).toSec() < 0.2*/)
			{
				// Ideal position of where ball should be for reaching it (webots behaves differently from real world!)
				float ideal_x =  0.18;
				float ideal_y = -0.18;

				// float distance_from_ideal = sqrt((ideal_x-last_ball_pos_.x)*(ideal_x-last_ball_pos_.x) + (ideal_y-last_ball_pos_.y)*(ideal_y-last_ball_pos_.y));
				// float distance_from_feet = sqrt(last_ball_pos_.x*last_ball_pos_.x + last_ball_pos_.y*last_ball_pos_.y);

				float distance_from_ideal = sqrt((ideal_x-static_ball_pos_.x)*(ideal_x-static_ball_pos_.x) + (ideal_y-static_ball_pos_.y)*(ideal_y-static_ball_pos_.y));
				float distance_from_feet = sqrt(static_ball_pos_.x*static_ball_pos_.x + static_ball_pos_.y*static_ball_pos_.y);

				if (distance_from_feet < 0.28 && distance_from_ideal < 0.07)
				{
					OpenHand(motion_proxy_ptr);
					float y_compensation = 0.03; //-0.04; When the hand approaches the ball, a part of the ball is covered and position is incorrect
					float hand_pos_error = MoveHand(motion_proxy_ptr, static_ball_pos_.x, static_ball_pos_.y+y_compensation, 0.15);
					if (hand_pos_error < 0.03)
					{
						set_feedback(SUCCESS);
						// has_succeeded_ = true;
						return 1;
					}
					else
					{
						set_feedback(RUNNING);
						return 0;
					}
					// std::cout << "Grasped!" << std::endl;
					// has_moved_hand_ = true;
				}
				else
				{
					std::cout << "Ball is too far away to reach!"
					          << " Ideal: (" << ideal_x << ", " << ideal_y
					          << ") Actual: (" << static_ball_pos_.x
					          << ", " << static_ball_pos_.y << ")" << std::endl;
				}
			}
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
	ros::init(argc, argv, "HandMover"); // name used for bt.txt
	//Read robot ip from command line parameters (--robot_ip=192.168.0.100 for example)
	setupCmdLineReader();
	std::string robot_ip = readRobotIPFromCmdLine(argc, argv);
	HandMover server(ros::this_node::getName(), robot_ip);
	ros::NodeHandle n;
	ros::Subscriber ball_pos_sub = n.subscribe<geometry_msgs::Pose2D>("ball_pos", 1,
	                                                                  &HandMover::BallPosReceived,
	                                                                  &server);
	ros::spin();
	return 0;
}
