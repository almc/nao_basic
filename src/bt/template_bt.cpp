#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
// #include <std_srvs/Empty.h>
#include <behavior_trees/rosaction.h>

class Walk : ROSAction
{
public:

	Walk(std::string name) :
		ROSAction(name),
		execute_time_((ros::Duration) 0),
		init(false)
		{

		}

	~Walk()
		{}

	void initialize()
		{
			// enableStiffness();
			send_feedback(RUNNING);
		}

	void finalize()
		{
			// disableStiffness();
		}

	void executeCB(ros::Duration dt)
		{
			std::cout << "**Walk -%- Executing Main Task, elapsed_time: "
			          << dt.toSec() << std::endl;
			std::cout << "**Walk -%- execute_time: "
			          << execute_time_.toSec() << std::endl;
			execute_time_ += dt;

			if (!init)
				initialize();

			if (execute_time_.toSec() >= 2)
			{
				send_feedback(SUCCESS);
				send_result(SUCCESS);
				finalize();
				stop();
			}
		}

	void resetCB()
		{
			execute_time_ = (ros::Duration) 0;
		}

	ros::Duration execute_time_;
	bool init;
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Walk"); // name used for bt.txt
	Walk server(ros::this_node::getName());
	ros::spin();
	return 0;
}
