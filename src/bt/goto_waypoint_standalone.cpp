#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Pose2D.h>
#include <nao_basic/robot_config.h>
#include <ltl3_NTUA/activity.h>

#include <iostream>
#include <unistd.h>
#include <math.h>

#include <alproxies/almotionproxy.h>
#include <alerror/alerror.h>

class GoToWaypoint 
{
public:
	bool init_;
	ros::Duration execute_time_;
	ros::Time time_at_pos_;
	geometry_msgs::Pose2D last_ball_pos_;
	AL::ALMotionProxy* motion_proxy_ptr;
	int closeness_count;

	GoToWaypoint(std::string name, std::string robot_ip):
		init_(false),
		execute_time_((ros::Duration) 0),
		time_at_pos_((ros::Time) 0),
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
		}

	void finalize()
		{
			closeness_count = 0;
			init_ = false;
			AL::ALValue stiffness_name("Body");
			AL::ALValue stiffness(0.0f);
			AL::ALValue stiffness_time(1.0f);
			motion_proxy_ptr->stiffnessInterpolation(stiffness_name,
			                                         stiffness,
			                                         stiffness_time);
		}

	int executeCB()
		{
			/*std::cout << "**GoToWaypoint -%- Executing Main Task, elapsed_time: "
			          << dt.toSec() << std::endl;
			std::cout << "**GoToWaypoint -%- execute_time: "
			          << execute_time_.toSec() << std::endl;
			*/

			if (!init_)
			{
				initialize();
				init_ = true;
			}
			if ( (ros::Time::now() - time_at_pos_).toSec() < 0.2)
			{
				// motion_proxy_ptr->setWalkTargetVelocity(speed_x, speed_y, 0.0, frequency);
				// ALMotionProxy::setWalkTargetVelocity(const float& x, const float& y, const float& theta, const float& frequency)
				AL::ALValue walk_config;
				walk_config.arrayPush(AL::ALValue::array("MaxStepFrequency", 0.2));
				walk_config.arrayPush(AL::ALValue::array("StepHeight", 0.005)); //Lower value of step height gives smoother walking
				if (fabs(last_ball_pos_.y) < 0.30)
				{
					std::cout << "case1: moving to " << last_ball_pos_ << std::endl;
					//motion_proxy_ptr->post.moveTo(last_ball_pos_.x, 0.0, 0.0, walk_config);
					//motion_proxy_ptr->post.moveTo(last_ball_pos_.x, last_ball_pos_.y, 0.0, walk_config);
					motion_proxy_ptr->post.moveTo(last_ball_pos_.x, last_ball_pos_.y, atan2(last_ball_pos_.y, last_ball_pos_.x), walk_config);

					//sleep(2.0);	// important to have the sleep here
				}
				else
				{
					std::cout << "case2: moving to " << last_ball_pos_ << std::endl;
					motion_proxy_ptr->post.moveTo(0.0, 0.0,
					                              atan2(last_ball_pos_.y, last_ball_pos_.x),
					                              walk_config);
					//sleep(2.0);	// important to have the sleep here
				}
			}
			return 0;
		}

	void resetCB()
		{
			execute_time_ = (ros::Duration) 0;
		}

	void NewWaypointReceived(const ltl3_NTUA::activity::ConstPtr &msg)
		{
			//std::cout << "Received waypoint position!" << std::endl;
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
	std::cout << "before ip thing" << std::endl;
	std::string robot_ip = readRobotIPFromCmdLine(argc, argv);
	GoToWaypoint server(ros::this_node::getName(), robot_ip);
	ros::NodeHandle n;
	ros::Subscriber ball_pos_sub = n.subscribe<ltl3_NTUA::activity>(std::string("next_move") + "_" + agent + "_relative", 1,
	                                                        &GoToWaypoint::NewWaypointReceived,
	                                                        &server);
	

	std::cout << "before running main loop" << std::endl;
	ros::Rate rate(5);
	while (ros::ok())
	{
		//std::cout << "running main loop" << std::endl;
		server.executeCB();
		ros::spinOnce();
		rate.sleep();
	}
	server.finalize();
	return 0;
}
