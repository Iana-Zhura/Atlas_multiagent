cmake_minimum_required(VERSION 2.8.12)
project(pytorch_motion_planner)
set(${INSTALL_BENCHMARK} BOOL "ON")

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if (${INSTALL_BENCHMARK})
    add_subdirectory(benchmark)
endif()

if (${INSTALL_ROS})
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        nav_msgs
        geometry_msgs
        message_generation
        )

catkin_python_setup()

catkin_package()

install(PROGRAMS
        scripts/pytorch_motion_planner_node.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
endif()