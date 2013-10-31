#include <nao_basic/robot_config.h>

po::options_description desc("Allowed options");

void setupCmdLineReader()
{
	addCmdLineOption("robot_ip");
	addCmdLineOption("color");
}

void addCmdLineOption(std::string argumentName)
{
	desc.add_options()
		(argumentName.c_str(), po::value<std::string>(), "Custom argument");
}

std::string readCmdLineOption(int argc, char** argv, std::string argument_name)
{
	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);
	po::notify(vm);

	std::string return_string = "";

	if (vm.count(argument_name))
	{
		return_string = vm[argument_name].as<std::string>();
		// std::cout << "Robot IP was set to " << robot_ip << std::endl;
	}
	else
	{
		std::cout << "Robot IP was not set" << std::endl;;
	}
	return return_string;
}

std::string readRobotIPFromCmdLine(int argc, char** argv)
{

	std::string robot_ip = readCmdLineOption(argc, argv, "robot_ip");

	if (robot_ip.length() < 1)
	{
		robot_ip = "localhost";
	}

	return robot_ip;
}

std::string readColorFromCmdLine(int argc, char** argv)
{

	std::string color_string = readCmdLineOption(argc, argv, "color");

	if (color_string.length() < 1)
	{
		color_string = "red";
	}
	return color_string;
}
