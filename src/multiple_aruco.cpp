/*
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 *  Description: Script to estimate pose from a multiple aruco markers with known size and pose,
 *  inspired from: https://docs.opencv.org/trunk/d5/dae/tutorial_aruco_detection.html
 *  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/foxbot/repo/opencv/opencv-3.3.1/release/lib
 *  Author: Matthieu Magnon
 *  Libraries: opencv3
 *
 */
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>

#include "opencv2/core/core.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/aruco.hpp>

#include <stdlib.h>
#include <stdio.h>
#include <iostream>

static const std::string OPENCV_WINDOW = "Image window";


// camera matrix and disto coeff must be determined
// cameraMatrix = [fx, 0, cx; 0, fy, cy; 0, 0, 1]
cv::Mat cameraMatrix;
// distCoeffsTable = [k1, k2, p1, p2 [, k3 [, k4, k5, k6]]]
std::vector<float> distCoeffs;

cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

geometry_msgs::Pose *ar1_pose = new geometry_msgs::Pose;
geometry_msgs::Pose *ar2_pose = new geometry_msgs::Pose;

void imageCallback(const sensor_msgs::Image& msg);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "single_aruco");

    ros::NodeHandle nh;

    ROS_INFO("Visual pose estimator node launched");

    // Get camera matrix
    ros::Duration timeout(10, 0); // 10sec
    sensor_msgs::CameraInfoConstPtr cam_msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/csi_cam_0/camera_info", nh, timeout);
    double P[3][3] = {{cam_msg->P[0], cam_msg->P[1], cam_msg->P[2]},
                      {cam_msg->P[4], cam_msg->P[5], cam_msg->P[6]},
                      {cam_msg->P[8], cam_msg->P[9], cam_msg->P[10]}};
    cameraMatrix = cv::Mat(3, 3,  CV_64F, P);
    ROS_INFO("Camera matrix : ");
    ROS_INFO("[%f, %f, %f]", cam_msg->P[0], cam_msg->P[1], cam_msg->P[2]);
    ROS_INFO("[%f, %f, %f]", cam_msg->P[4], cam_msg->P[5], cam_msg->P[6]);
    ROS_INFO("[%f, %f, %f]", cam_msg->P[8], cam_msg->P[9], cam_msg->P[10]);

    double D[5] = {cam_msg->D[0], cam_msg->D[1], cam_msg->D[2], cam_msg->D[3], cam_msg->D[4]};
    ROS_INFO("Distorsion coefficients : ");
    ROS_INFO("[%f, %f, %f, %f, %f]", cam_msg->D[0], cam_msg->D[1], cam_msg->D[2], cam_msg->D[3], cam_msg->D[4]);

    for (int i = 0; i < 5; i++)
        distCoeffs.push_back(D[i]);

    /* Define aruco pose in world frame */
    ar1_pose->position.x = 0.0;
    ar1_pose->position.y = 0.0;
    ar1_pose->position.z = 0.1;
    ar1_pose->orientation.w = 1.0;
    ar1_pose->orientation.x = 0.0;
    ar1_pose->orientation.y = 0.0;
    ar1_pose->orientation.z = 0.0;

    ar2_pose->position.x = 0.0;
    ar2_pose->position.y = 0.2;
    ar2_pose->position.z = 0.1;
    ar2_pose->orientation.w = 1.0;
    ar2_pose->orientation.x = 0.0;
    ar2_pose->orientation.y = 0.0;
    ar2_pose->orientation.z = 0.0;

    ros::Subscriber img_rect_sub = nh.subscribe("/csi_cam_0/image_raw", 10, imageCallback);

    ros::spin();
}

void imageCallback(const sensor_msgs::Image& msg)
{
    cv_bridge::CvImagePtr cv_ptr;

    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
        cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Detect arucos
    std::vector<std::vector<cv::Point2f> > corners;
    std::vector<int> ids;
    cv::aruco::detectMarkers(cv_ptr->image, dictionary, corners, ids);

    // if at least one marker detected
    if (ids.size() != 0) {

        std::vector<cv::Vec3d> rvecs, tvecs;

        ROS_INFO("%i arucos detected", (int)ids.size());

        cv::aruco::estimatePoseSingleMarkers(corners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);

        // draw axis for each marker
        for(int i=0; i<ids.size(); i++) {
            auto rvec = rvecs[i];
            auto tvec = tvecs[i];
            auto corner = corners[i];
            ROS_INFO(" *** aruco %i *** ", (int)ids[i]);
            auto c1 = corner[0];
            ROS_INFO("corner1 XY [pixel] = [%f %f]", c1.x, c1.y);

            ROS_INFO("pos XYZ = [%f, %f, %f], att = [%f, %f, %f]", tvec[0], tvec[1], tvec[2], rvec[0], rvec[1], rvec[2]);
            cv::aruco::drawAxis(cv_ptr->image, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
        }

        // Convert Rvec and tvec to pose
        for (int i = 0; i < ids.size(); i++) {
            auto rvec = rvecs[i];
            auto tvec = tvecs[i];

            // rvec is the Rodrigue vector, describing the rotation from the camera frame to the marker frame
            // then tvec is the rotationnal vector

            double theta = sqrt(rvec[0] * rvec[0] + rvec[1] * rvec[1] + rvec[2] * rvec[2]);
            ROS_INFO("theta = %f", theta * 57.0);

            double qw, qx, qy, qz;
            tf::Quaternion q(cos(theta / 2.0), rvec[0] * sin(theta / 2.0), rvec[1] * sin(theta / 2.0), rvec[2] * sin(theta / 2.0));
            q.normalize();

            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            ROS_INFO("RPY = [%f, %f, %f] deg", roll * 57.0, pitch * 57.0, yaw * 57.0);
            ROS_INFO("XYZ = [%f, %f, %f] mm", tvec[0] * 1000.0, tvec[1] * 1000.0, tvec[2] * 1000.0);
        }

    }


    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
    
    return;
}
