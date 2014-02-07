#include <nao_basic/motions_common.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <actionlib/client/simple_action_client.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Pose2D.h>

#include <nao_basic/activity.h>



#include <behavior_trees/rosaction.h>
#include <nao_basic/robot_config.h>

#include <iostream>
#include <unistd.h>
#include <math.h>

#include <alproxies/almotionproxy.h>
#include <alproxies/altexttospeechproxy.h>
#include <alerror/alerror.h>


ros::Publisher chatter_pub;

int counter=0;



class GoToWaypoint : ROSAction
{
public:
	bool init_;
	ros::Duration execute_time_;
	ros::Time time_at_pos_;
	nao_basic::activity message_;
	AL::ALTextToSpeechProxy* speech_proxy_ptr;
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
		// message_(geometry_msgs::Pose2D(0,0,0))
		{
			std::cout << "Robot ip to use is: " << robot_ip << std::endl;

			motion_proxy_ptr = new AL::ALMotionProxy(robot_ip, 9559);
			speech_proxy_ptr = new AL::ALTextToSpeechProxy(robot_ip, 9559);

		}

	~GoToWaypoint()
		{
			delete motion_proxy_ptr;
			delete speech_proxy_ptr;
		}

	void initialize()
		{


			// write "right" to ltl.planner
			std_msgs::String msg;
			std::stringstream ss;
			ss << "right" << std::endl;
			std::cout << "**AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAGoToWaypoint -%- Executing Main Task, elapsed_time: " << std::endl;

			msg.data = ss.str();
			// ROS_INFO("%s", msg.data.c_str());
			chatter_pub.publish(msg);
			std::cout << "**msg:"
			          << msg << std::endl;
			counter++;

			sleep(1.0);
			//Set the stiffness so the robot can move
			AL::ALValue stiffness_name("Body");
			AL::ALValue stiffness(1.0f);
			AL::ALValue stiffness_time(1.0f);
			motion_proxy_ptr->stiffnessInterpolation(stiffness_name,
			                                         stiffness,
			                                         stiffness_time);
			if( message_.type == "goto")
				{
				 motion_proxy_ptr->moveInit();
				}
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

			std::cout << "**Counter value:" << counter << std::endl;
			if (counter > 1)
				std::cout << "********************************************************************************" << std::endl;


			if (!init_)
			{
				initialize();
				init_ = true;
			}
			if ( (ros::Time::now() - time_at_pos_).toSec() < 0.2 )
			{
				if( message_.type == "goto")
				{
						// Goal position of ball relative to ROBOT_FRAME
						float goal_x = 0.00;
						float goal_y = 0.00;
						float error_x = message_.x - goal_x;
						float error_y = message_.y - goal_y;
						if (fabs(error_x) < 0.12 && fabs(error_y) < 0.12)
						{
							std::cout << "Closeness count " << closeness_count << std::endl;
							closeness_count++;
							//If the NAO has been close for enough iterations, we consider to goal reached
							if (closeness_count > 0)
							{
								motion_proxy_ptr->stopMove();
								set_feedback(SUCCESS);
								// std::cout << "sleeeping for 2 second before destroying thread" << std::endl;
								// sleep(2.0);
								finalize();
								return 1;
							}
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
						//Lower value of step height gives smoother walking
						// std::cout << "y " << message_.y << std::endl;
						if (fabs(message_.y) < 0.10)
						{
							// walk_config.arrayPush(AL::ALValue::array("StepHeight", 0.01));
							// motion_proxy_ptr->post.moveTo(message_.x, 0.0, 0.0, walk_config);
							motion_proxy_ptr->post.moveTo(message_.x, 0.0, 0.0);
							sleep(2.0);
							//motion_proxy_ptr->post.stopMove();
						}
						else
						{
							// walk_config.arrayPush(AL::ALValue::array("StepHeight", 0.005));
							// motion_proxy_ptr->post.moveTo(0.0, 0.0, message_.y/fabs(message_.y)*0.1, walk_config);
							motion_proxy_ptr->post.moveTo(0.0, 0.0, message_.y/fabs(message_.y)*0.1);
							//sleep(3.0);
							//motion_proxy_ptr->post.stopMove();
						}
				}

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
			time_at_pos_ = ros::Time::now();
			message_.type = msg->type;
			message_.x = 0.01*msg->x;
			message_.y = 0.01*msg->y;
			std::cout << "Message Type " << message_.type << std::endl;

		}
};

int main(int argc, char** argv)
{
	std::cout << "Hello, world!" << std::endl;
	ros::init(argc, argv, "MoveRight"); // name used for bt.txt
	//Read robot ip from command line parameters (--robot_ip=192.168.0.100 for example)
	setupCmdLineReader();
	std::string robot_ip = readRobotIPFromCmdLine(argc, argv);
	GoToWaypoint server(ros::this_node::getName(), robot_ip);
	ros::NodeHandle n;


	ros::Subscriber ball_pos_sub = n.subscribe<nao_basic::activity>("next_move", 1,
	                                                        &GoToWaypoint::NewWaypointReceived,
	                                                        &server);
	chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
	ros::spin();
	return 0;
}
