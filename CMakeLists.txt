cmake_minimum_required(VERSION 2.8.3)
project(visual_pose_estimator)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs cv_bridge)

## Declare a catkin package
catkin_package()

add_executable(single_aruco src/single_aruco.cpp)
add_executable(multiple_aruco src/multiple_aruco.cpp)

find_package(OpenCV REQUIRED)
include_directories(include ${catkin_INCLUDE_DIRS})
target_link_libraries(single_aruco ${catkin_LIBRARIES})
target_link_libraries(multiple_aruco ${catkin_LIBRARIES})

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(single_aruco ${OpenCV_LIBRARIES})
target_link_libraries(multiple_aruco ${OpenCV_LIBRARIES})