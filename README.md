ROS package for position estimator based on monocular camera and known arucos position

With the current build config, the following export is needed
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/foxbot/repo/opencv/opencv-3.3.1/release/lib

could be tested with:
$ roslaunch foxbot_ros docking_simulation.launch
$ roslaunch visual_pose_estimator aruco_pose_stf.launch