cmake_minimum_required(VERSION 2.8.3)
set (CMAKE_CXX_STANDARD 11)
project(i3dr_sbri)

find_package(catkin REQUIRED COMPONENTS
                roscpp std_msgs tf rospy)

include_directories(
	include
    ${catkin_INCLUDE_DIRS}
)

catkin_package( 
    INCLUDE_DIRS include
	CATKIN_DEPENDS roscpp std_msgs
	LIBRARIES ${PROJECT_NAME}
#  DEPENDS system_lib
)
