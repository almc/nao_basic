#ifndef CONFIG_H_
#define CONFIG_H_

#include <boost/program_options.hpp>

namespace po = boost::program_options;

void setupCmdLineReader();
void addCmdLineOption(std::string argumentName);
std::string readCmdLineOption(std::string argumentName);
std::string readRobotIPFromCmdLine(int argc, char** argv);
std::string readColorFromCmdLine(int argc, char** argv);

#endif
