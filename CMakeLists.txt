cmake_minimum_required(VERSION 2.8.3)
project(visual_pose_estimator)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs cv_bridge tf)

## Declare a catkin package
catkin_package()

add_executable(single_aruco src/single_aruco.cpp)
add_executable(multiple_aruco src/multiple_aruco.cpp)

find_package(OpenCV 3.2 REQUIRED)

set(OpenCV_INCLUDE_DIRS
	/home/foxbot/repo/opencv/opencv-3.3.1/include/opencv
	/home/foxbot/repo/opencv/opencv-3.3.1/include/opencv2
)

set(OpenCV_LIBRARIES
  opencv_core
  opencv_highgui
  opencv_imgcodecs
  opencv_aruco
)

message(STATUS "OpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV_LIBS = ${OpenCV_LIBS}")

include_directories(include
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS})
target_link_libraries(
	single_aruco
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES})
target_link_libraries(
	multiple_aruco
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES})
