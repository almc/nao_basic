#include <behavior_trees/rosaction.h>
#include <nao_basic/robot_config.h>
#include <nao_basic/activity.h>
#include <nao_basic/confirmation.h>

#include <alerror/alerror.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/altexttospeechproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <alproxies/alledsproxy.h>

#include <iostream>
#include <geometry_msgs/Pose2D.h>

ros::Publisher confirmation_pub;

class ActionDispatcher : ROSAction
{
public:
	bool init_;
	ros::Duration execute_time_;
	ros::Time msg_timestamp_;
	geometry_msgs::Pose2D goal_position_;
	AL::ALMotionProxy* motion_proxy_ptr;
	AL::ALRobotPostureProxy* posture_proxy_ptr;
	AL::ALLedsProxy* leds_proxy_ptr;
	AL::ALTextToSpeechProxy* speech_proxy_ptr;

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
		}

	~ActionDispatcher()
		{
			std::cout << "*** Destructor(): " << std::endl;
			delete motion_proxy_ptr;
			delete posture_proxy_ptr;
			delete leds_proxy_ptr;
			delete speech_proxy_ptr;
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
			set_feedback(RUNNING);
		}

	void finalize()
		{
			std::cout << "*** Finalize(): " << std::endl;
			closeness_count = 0;
			init_ = false;
			deactivate();
		}


	int Crouch_Function()
		{
			std::cout << "*** Crouch_Function(): " << std::endl;
			posture_proxy_ptr->goToPosture("Crouch", 0.8);
			float duration = 3.0f;
			leds_proxy_ptr->rasta(duration);
			posture_proxy_ptr->goToPosture("StandInit", 1.0);
			nao_basic::confirmation msg_conf;
			msg_conf.name = "crouch";
			msg_conf.done = 1;
			confirmation_pub.publish(msg_conf);
			return 0;
		}

	int Idle_Function()
		{
			std::cout << "*** Idle_Function(): " << std::endl;
			nao_basic::confirmation msg_conf;
			msg_conf.name = "idle";
			msg_conf.done = 1;
			confirmation_pub.publish(msg_conf);
			return 0;
		}

	int GotoWP_Function()
		{
			std::cout << "*** GotoWP_Function(): " << std::endl;
			//Goal position of ball relative to ROBOT_FRAME
			float goal_x = 0.00;
			float goal_y = 0.00;
			// float error_x = goal_position_.x - goal_x;
			// float error_y = goal_position_.y - goal_y;
			AL::ALValue walk_config;
			// walk_config.arrayPush(AL::ALValue::array("MaxStepFrequency", frequency));
			// walk_config.arrayPush(AL::ALValue::array("StepHeight", 0.01));
			if (fabs (goal_position_.y) < 0.08)
			{
				motion_proxy_ptr->post.moveTo(0.0, goal_position_.x, 0.0, walk_config);
			}
			else
			{
				motion_proxy_ptr->post.moveTo(0.0, 0.0,
				                              atan2(goal_position_.y, goal_position_.x),
				                              walk_config);
			}
			return 0;
		}

	int executeCB(ros::Duration dt)
		{
			// std::cout << "**ActionDispatcher -%- Executing Main Task, elapsed_time: "
			//           << dt.toSec() << std::endl;
			// std::cout << "**ActionDispatcher -%- execute_time: "
			//           << execute_time_.toSec() << std::endl;
			// execute_time_ += dt;
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
			else if (!activity_name_.compare("none"))
			{
				std::cout << "Received Nothing" << std::endl;
				set_feedback(RUNNING);
				return 0;
			}
			else
			{
				set_feedback(RUNNING);
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
			std::cout << activity_name_ << std::endl;
			std::cout << goal_position_.x << std::endl;
			std::cout << goal_position_.y << std::endl;
		}
};

int main(int argc, char** argv)
{
	std::cout << "Hello, world!" << std::endl;
	// specify which options are available as cmd line arguments
	setupCmdLineReader();
	// read agent id from command line parameters (--agent=mario)
	std::string agent = readAgentFromCmdLine(argc, argv);
	// read robot ip from command line parameters (--robot_ip=mario)
	std::string robot_ip = readRobotIPFromCmdLine(argc, argv);
	ros::init(argc, argv, std::string("ActionDispatcher") + "_" + agent); // name used for bt.txt
	ActionDispatcher server(ros::this_node::getName(), robot_ip);
	ros::NodeHandle n;
	ros::Subscriber waypoint_sub =
		n.subscribe<nao_basic::activity>(std::string("next_move") + "_" + agent,
		                                 1, &ActionDispatcher::MessageReceivedCB, &server);
	confirmation_pub =
		n.advertise<nao_basic::confirmation>(std::string("activity_done") + "_" + agent, 10);
	ros::spin();
	return 0;
}
