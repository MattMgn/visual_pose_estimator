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
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "opencv2/core/core.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/aruco.hpp>

#include <stdlib.h>
#include <stdio.h>
#include <iostream>

#define DEBUG
#define HOMOGENEOUS_COORD_NB   4

#define EXTRINSIC_FROM_TOPIC

#define CAMERA_NAME "/camera/rgb"
//#define CAMERA_NAME "/csi_cam_0"


// cameraMatrix = [fx, 0, cx; 0, fy, cy; 0, 0, 1]
cv::Mat cameraMatrix;
// distCoeffsTable = [k1, k2, p1, p2 [, k3 [, k4, k5, k6]]]
std::vector<float> distCoeffs;

cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

geometry_msgs::Pose *ar0_pose = new geometry_msgs::Pose;
geometry_msgs::Pose *ar1_pose = new geometry_msgs::Pose;

geometry_msgs::TransformStamped cam_pose;
//tf::TransformListener * arucos_listener;
tf::TransformBroadcaster * cam_broadcaster;

double arucoId0Coord[HOMOGENEOUS_COORD_NB] = {0.0, 0.0, 0.1, 1.0};
double arucoId1Coord[HOMOGENEOUS_COORD_NB] = {0.1, 0.0, 0.1, 1.0};

void imageCallback(const sensor_msgs::Image& msg);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "multiple_aruco");
    ros::NodeHandle nh;
    ROS_INFO("Visual pose estimator node launched");

    cam_broadcaster = new tf::TransformBroadcaster();

    /* Get camera matrix parameters */

#ifdef EXTRINSIC_FROM_TOPIC
    ros::Duration timeout(10, 0); // 10sec
    std::string camera_info_str = CAMERA_NAME + std::string("/camera_info");
    sensor_msgs::CameraInfoConstPtr cam_msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_info_str, nh, timeout);
    //sensor_msgs::CameraInfoConstPtr cam_msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/csi_cam_0/camera_info", nh, timeout);

    double P[3][3] = {{cam_msg->P[0], cam_msg->P[1], cam_msg->P[2]},
                      {cam_msg->P[4], cam_msg->P[5], cam_msg->P[6]},
                      {cam_msg->P[8], cam_msg->P[9], cam_msg->P[10]}};
    cameraMatrix = cv::Mat(3, 3,  CV_64F, P);
    double D[5] = {cam_msg->D[0], cam_msg->D[1], cam_msg->D[2], cam_msg->D[3], cam_msg->D[4]};
    for (int i = 0; i < 5; i++)
        distCoeffs.push_back(D[i]);
#else
    // hardcoded way
    double P[3][3] = {{1340.368408, 0.000000, 655.600605},
                      {0.000000, 1338.741943, 337.300296},
                      {0.000000, 0.000000, 1.000000}};
    cameraMatrix = cv::Mat(3, 3,  CV_64F, P);
    double D[5] = {0.169290, -0.241683, -0.006518, 0.003605, 0.000000};
    for (int i = 0; i < 5; i++)
        distCoeffs.push_back(D[i]);

#endif

    ROS_INFO("Camera matrix : ");
    ROS_INFO("[%f, %f, %f]", cam_msg->P[0], cam_msg->P[1], cam_msg->P[2]);
    ROS_INFO("[%f, %f, %f]", cam_msg->P[4], cam_msg->P[5], cam_msg->P[6]);
    ROS_INFO("[%f, %f, %f]", cam_msg->P[8], cam_msg->P[9], cam_msg->P[10]);

    ROS_INFO("Distorsion coefficients : ");
    ROS_INFO("[%f, %f, %f, %f, %f]", cam_msg->D[0], cam_msg->D[1], cam_msg->D[2], cam_msg->D[3], cam_msg->D[4]);

    /* Get aruco pose in world frame */
    // aruco_id0
    tf::TransformListener arucos_listener;
    tf::StampedTransform transform;
    arucos_listener.waitForTransform("odom", "aruco_id0", ros::Time::now(), ros::Duration(3.0));
    arucos_listener.lookupTransform("odom", "aruco_id0", ros::Time::now(), transform);
    ar0_pose->position.x = (float)transform.getOrigin().x();
    ar0_pose->position.y = (float)transform.getOrigin().y();
    ar0_pose->position.z = (float)transform.getOrigin().z();
    tf::quaternionTFToMsg(transform.getRotation(), ar0_pose->orientation);

    // aruco_id1
    arucos_listener.waitForTransform("odom", "aruco_id1", ros::Time(0), ros::Duration(3.0));
    arucos_listener.lookupTransform("odom", "aruco_id1", ros::Time(0), transform);
    ar1_pose->position.x = (float)transform.getOrigin().x();
    ar1_pose->position.y = (float)transform.getOrigin().y();
    ar1_pose->position.z = (float)transform.getOrigin().z();
    tf::quaternionTFToMsg(transform.getRotation(), ar1_pose->orientation);

    std::string image_name_str = CAMERA_NAME + std::string("/image_raw");
    ros::Subscriber img_rect_sub = nh.subscribe(image_name_str, 10, imageCallback);

    ros::spin();
}

void imageCallback(const sensor_msgs::Image& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;

    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    /* Detect arucos markers */
    cv::aruco::detectMarkers(cv_ptr->image, dictionary, corners, ids);

    if (ids.size() != 0) {
        std::vector<cv::Vec3d> rvecs, tvecs;

        /* Estimate aruco pose with respect to the camera */
        cv::aruco::estimatePoseSingleMarkers(corners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);

        /* Draw triedra */
        for(int i = 0; i < ids.size(); i++) {
            auto rvec = rvecs[i];
            auto tvec = tvecs[i];
            auto corner = corners[i];
            cv::aruco::drawAxis(cv_ptr->image, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
        }

        for (int i = 0; i < ids.size(); i++) {
            auto rvec = rvecs[i];
            auto tvec = tvecs[i];

            /* Compute extrinsic parameter matrix for homogeneous coordinates, corresponding to the transformation
             * from marker coordinates to the camera coordinates.
             * The marker corrdinate system is centered on the middle of the marker, with the Z axis perpendicular
             * to the marker plane */
            cv::Mat rotationMatrix(3, 3, CV_64F), extrinsicMatrix(3, 4, CV_64F);
            cv::Rodrigues(rvec, rotationMatrix);
            cv::hconcat(rotationMatrix, tvec, extrinsicMatrix);

            /* Compute marker coordinates expressed */
            double arucoCoord[4] = {0.0};
            arucoCoord[3] = 1.0;
            switch ((int)ids[i]) {
                case 0:
                    arucoCoord[0] = (double)ar0_pose->position.x; arucoCoord[1] = (double)ar0_pose->position.y; arucoCoord[2] = (double)ar0_pose->position.z;
                    break;
                case 1:
                    arucoCoord[0] = ar1_pose->position.x; arucoCoord[1] = ar1_pose->position.y; arucoCoord[2] = ar1_pose->position.z;
                    break;
                default:
                    ROS_WARN("Unknown pose for Aruco ID %i", (int)ids[i]); 
            }
            cv::Mat arucoCoordVector(4, 1, CV_64F, arucoCoord);
            cv::Mat arucoCoordVectorWorldFrame(4, 1, CV_64F);
            arucoCoordVectorWorldFrame = extrinsicMatrix*arucoCoordVector;

#ifdef DEBUG_0
            std::cout << "***** ID " << (int)ids[i] << " *****" << std::endl;
            std::cout << "extrinsicMatrix" << std::endl;
            std::cout << extrinsicMatrix << std::endl;
            std::cout << "arucoCoordVector" << std::endl;
            std::cout << arucoCoordVector << std::endl;
            std::cout << "arucoCoordVectorWorldFrame" << std::endl;
            std::cout << arucoCoordVectorWorldFrame << std::endl;
#endif

            /* Compute quaternion from Rodrigue vector */
            double theta = sqrt(rvec[0] * rvec[0] + rvec[1] * rvec[1] + rvec[2] * rvec[2]);
            tf::Quaternion q(cos(theta / 2.0), rvec[0] * sin(theta / 2.0), rvec[1] * sin(theta / 2.0), rvec[2] * sin(theta / 2.0));
            q.normalize();

            cam_pose.header.stamp = ros::Time::now();
            cam_pose.header.frame_id = "world";
            cam_pose.child_frame_id = "camera_frame";
            cam_pose.transform.translation.x = arucoCoordVectorWorldFrame.at<double>(0,0);
            cam_pose.transform.translation.y = arucoCoordVectorWorldFrame.at<double>(1,0);
            cam_pose.transform.translation.z = arucoCoordVectorWorldFrame.at<double>(2,0);
            tf::quaternionTFToMsg(q, cam_pose.transform.rotation);
        }

        /* TODO: add filtering between multiples arucos */
        cam_broadcaster->sendTransform(cam_pose);

    }

#ifdef DEBUG
    cv::imshow("viewer", cv_ptr->image);
    cv::waitKey(3);
#endif

    return;
}
