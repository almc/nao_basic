#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Pose2D.h>

#include <behavior_trees/rosaction.h>
#include <nao_basic/robot_config.h>
#include <nao_basic/activity.h>

#include <iostream>
#include <unistd.h>
#include <math.h>

#include <alproxies/almotionproxy.h>
#include <alerror/alerror.h>

class GoToWaypoint : ROSAction
{
public:
	bool init_;
	ros::Duration execute_time_;
	ros::Time time_at_pos_;
	geometry_msgs::Pose2D last_ball_pos_;
	AL::ALMotionProxy* motion_proxy_ptr;

	bool has_succeeded;
	int closeness_count;

	GoToWaypoint(std::string name, std::string robot_ip):
		ROSAction(name),
		init_(false),
		execute_time_((ros::Duration) 0),
		time_at_pos_((ros::Time) 0),
		has_succeeded(false),
		closeness_count(0)
		// last_ball_pos_(geometry_msgs::Pose2D(0,0,0))
		{
			std::cout << "Robot ip to use is: " << robot_ip << std::endl;

			motion_proxy_ptr = new AL::ALMotionProxy(robot_ip, 9559);
		}

	~GoToWaypoint()
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
			has_succeeded = false;
			closeness_count = 0;
			init_ = false;
			deactivate();
		}

	int executeCB(ros::Duration dt)
		{
			std::cout << "**GoToWaypoint -%- Executing Main Task, elapsed_time: "
			          << dt.toSec() << std::endl;
			std::cout << "**GoToWaypoint -%- execute_time: "
			          << execute_time_.toSec() << std::endl;
			execute_time_ += dt;

			if (!init_)
			{
				initialize();
				init_ = true;
			}
			if ( (ros::Time::now() - time_at_pos_).toSec() < 0.2)
			{
				//Goal position of ball relative to ROBOT_FRAME
				float goal_x = 0.00;
				float goal_y = 0.00;

				float error_x = last_ball_pos_.x - goal_x;
				float error_y = last_ball_pos_.y - goal_y;

				if ((fabs(error_x) < 0.05 && fabs(error_y) < 0.05) || has_succeeded)
				{
					std::cout << "Closeness count " << closeness_count << std::endl;
					closeness_count++;
					//If the NAO has been close for enough iterations, we consider to goal reached
					if (closeness_count > 3)
					{
						has_succeeded = true;
						motion_proxy_ptr->stopMove();
						set_feedback(SUCCESS);
						finalize();
					}
					return 1;
				}
				else
				{
					closeness_count = 0;
				}
				//Limit the "error" in order to limit the walk speed
				error_x = error_x >  0.6 ?  0.6 : error_x;
				error_x = error_x < -0.6 ? -0.6 : error_x;
				error_y = error_y >  0.6 ?  0.6 : error_y;
				error_y = error_y < -0.6 ? -0.6 : error_y;
				// float speed_x = error_x * 1.0/(2+5*closeness_count);
				// float speed_y = error_y * 1.0/(2+5*closeness_count);
				// float frequency = 0.1/(5*closeness_count+(1.0/(fabs(error_x)+fabs(error_y)))); //Frequency of foot steps
				// motion_proxy_ptr->setWalkTargetVelocity(speed_x, speed_y, 0.0, frequency);
				// ALMotionProxy::setWalkTargetVelocity(const float& x, const float& y, const float& theta, const float& frequency)
				AL::ALValue walk_config;
				//walk_config.arrayPush(AL::ALValue::array("MaxStepFrequency", frequency));
				walk_config.arrayPush(AL::ALValue::array("StepHeight", 0.01)); //Lower value of step height gives smoother walking
				if (fabs (last_ball_pos_.y) < 0.10)
				{
					// std::cout << "case1: moving to " << last_ball_pos_ << std::endl;
					motion_proxy_ptr->post.moveTo(last_ball_pos_.x, 0.0, 0.0, walk_config);
					sleep(2.0);	// important to have the sleep here
				}
				else
				{
					// std::cout << "case2: moving to " << last_ball_pos_ << std::endl;
					motion_proxy_ptr->post.moveTo(0.0, 0.0,
					                              atan2(last_ball_pos_.y, last_ball_pos_.x),
					                              walk_config);
				}
			}
			else if (has_succeeded)
			{
				set_feedback(SUCCESS);
				return 1;
			}
			else if ( (ros::Time::now() - time_at_pos_).toSec() > 0.2)
			{
				set_feedback(RUNNING);
				return 0;
			}
			set_feedback(RUNNING);
			return 0;
		}

	void resetCB()
		{
			execute_time_ = (ros::Duration) 0;
		}

	void NewWaypointReceived(const nao_basic::activity::ConstPtr &msg)
		{
			std::cout << "Received waypoint position!" << std::endl;
			time_at_pos_ = ros::Time::now();
			last_ball_pos_.x = 0.01*msg->x;
			last_ball_pos_.y = 0.01*msg->y;
		}
};

int main(int argc, char** argv)
{
	std::cout << "Hello, world!" << std::endl;
	// specify which options are available as cmd line arguments
	setupCmdLineReader();
	// read agent id from command line parameters (--agent=mario)
	std::string agent = readAgentFromCmdLine(argc, argv);
	ros::init(argc, argv, std::string("GoToWaypoint") + "_" + agent); // name used for bt.txt
	//Read robot ip from command line parameters (--robot_ip=192.168.0.100 for example)
	std::string robot_ip = readRobotIPFromCmdLine(argc, argv);
	GoToWaypoint server(ros::this_node::getName(), robot_ip);
	ros::NodeHandle n;
	ros::Subscriber ball_pos_sub = n.subscribe<nao_basic::activity>(std::string("next_move") + "_" + agent, 1,
	                                                        &GoToWaypoint::NewWaypointReceived,
	                                                        &server);
	ros::spin();
	return 0;
}
