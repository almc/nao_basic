cmake_minimum_required(VERSION 2.6)
project (NaoBasic)
message(Compiling: ${PROJECT_NAME})

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

find_package(ALCOMMON  REQUIRED)
find_package(ALPROXIES REQUIRED)
find_package(ALVISION  REQUIRED)
find_package(ALVALUE   REQUIRED)




rosbuild_init()

# Settings
add_definitions(-lglut -Wall -g -O0 -Wno-deprecated -static -Bstatic -std=gnu++0x)

# Includes
set(COMMON_INCLUDES ${PROJECT_SOURCE_DIR}/src/common)
include_directories(${COMMON_INCLUDES})

# Project source
file(GLOB_RECURSE COMMON_SRC ${PROJECT_SOURCE_DIR}/src/common/*.cpp)

#########################################################
# FIND ALDEBARAN (naoqi-sdk-1.14.3-linux64/include /lib
#########################################################
include_directories(${ALCOMMON_INCLUDE_DIRS})

#########################################################
# FIND OPENCV
#########################################################
find_package(rosbuild REQUIRED COMPONENTS behavior_trees)
find_package(OpenCV REQUIRED)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
# rosbuild_genmsg()
#uncomment if you have defined services
# rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

# rosbuild_add_executable(TestBT src/bt/template_bt.cpp)
# rosbuild_link_boost(TestBT thread)

rosbuild_add_executable(BallTrackerBT src/bt/ball_tracker_bt.cpp ${COMMON_SRC})
target_link_libraries(BallTrackerBT ${ALPROXIES_LIBRARIES} ${ALCOMMON_LIBRARIES} ${ALVISION_LIBRARIES} ${ALVALUE_LIBRARIES} ${OpenCV_LIBS})
rosbuild_link_boost(BallTrackerBT thread program_options)

rosbuild_add_executable(BallGrasperBT src/bt/ball_grasper_bt.cpp src/common/motions_common.cpp src/common/robot_config.cpp)
target_link_libraries(BallGrasperBT ${ALPROXIES_LIBRARIES} ${ALCOMMON_LIBRARIES} ${ALVALUE_LIBRARIES})
rosbuild_link_boost(BallGrasperBT thread program_options)

rosbuild_add_executable(HandMover2BT src/bt/hand_mover2_bt.cpp src/common/motions_common.cpp src/common/robot_config.cpp)
target_link_libraries(HandMover2BT ${ALPROXIES_LIBRARIES} ${ALCOMMON_LIBRARIES} ${ALVALUE_LIBRARIES})
rosbuild_link_boost(HandMover2BT thread program_options)

rosbuild_add_executable(BallThrowerBT src/bt/ball_thrower_bt.cpp src/common/motions_common.cpp src/common/robot_config.cpp)
target_link_libraries(BallThrowerBT ${ALPROXIES_LIBRARIES} ${ALCOMMON_LIBRARIES} ${ALVALUE_LIBRARIES})
rosbuild_link_boost(BallThrowerBT thread program_options)

rosbuild_add_executable(WalkerBT src/bt/walker_bt.cpp src/common/robot_config.cpp)
target_link_libraries(WalkerBT ${ALPROXIES_LIBRARIES} ${ALCOMMON_LIBRARIES} ${ALVALUE_LIBRARIES})
rosbuild_link_boost(WalkerBT thread program_options)


rosbuild_add_executable(MissionFailedBT src/bt/mission_failed_bt.cpp src/common/robot_config.cpp)
target_link_libraries(MissionFailedBT ${ALPROXIES_LIBRARIES} ${ALCOMMON_LIBRARIES} ${ALVALUE_LIBRARIES})
rosbuild_link_boost(MissionFailedBT thread program_options)


rosbuild_add_executable(HandMover22ndBT src/bt/hand_mover22nd_bt.cpp src/common/motions_common.cpp src/common/robot_config.cpp)
target_link_libraries(HandMover22ndBT ${ALPROXIES_LIBRARIES} ${ALCOMMON_LIBRARIES} ${ALVALUE_LIBRARIES})
rosbuild_link_boost(HandMover22ndBT thread program_options)


rosbuild_add_executable(Walker2ndBT src/bt/walker2nd_bt.cpp src/common/robot_config.cpp)
target_link_libraries(Walker2ndBT ${ALPROXIES_LIBRARIES} ${ALCOMMON_LIBRARIES} ${ALVALUE_LIBRARIES})
rosbuild_link_boost(Walker2ndBT thread program_options)

rosbuild_add_executable(GoToWaypointBT src/bt/goto_waypoint_bt.cpp src/common/robot_config.cpp)
target_link_libraries(GoToWaypointBT ${ALPROXIES_LIBRARIES} ${ALCOMMON_LIBRARIES} ${ALVALUE_LIBRARIES})
rosbuild_link_boost(GoToWaypointBT thread program_options)

rosbuild_add_executable(BallDetector src/test/ball_detector.cpp)
target_link_libraries(BallDetector ${ALPROXIES_LIBRARIES} ${ALCOMMON_LIBRARIES} ${ALVISION_LIBRARIES} ${ALVALUE_LIBRARIES} ${OpenCV_LIBS})
# target_link_libraries(BallDetector alproxies alcommon alvision alvalue ${OpenCV_LIBS})
