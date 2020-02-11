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
 *  Description: Script to estimate pose from a single aruco market with known size, inspired from:
 *  https://docs.opencv.org/trunk/d5/dae/tutorial_aruco_detection.html
 *  No ROS libraries used
 *  Author: Matthieu Magnon
 *  Libraries: opencv3
 *
 */
#include <ros/ros.h>

#include "opencv2/core/core.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/aruco.hpp>

#include <stdlib.h>
#include <stdio.h>
#include <iostream>

cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);


// camera matrix and disto coeff must be determined
// cameraMatrix = [fx, 0, cx; 0, fy, cy; 0, 0, 1]
cv::Mat cameraMatrix = (cv::Mat_<float>(3,3) <<  -4.1802327176423804e-001, 5.0715244063187526e-001, 0.0,
                                                0.0, 6.5746697944293521e+002, 2.3950000000000000e+002,
                                                0.0, 0.0, 1.0);
// distCoeffsTable = [k1, k2, p1, p2 [, k3 [, k4, k5, k6]]]

float distCoeffsTable[5] = {5.44787247e-02, 1.23043244e-01, -4.52559581e-04, 5.47011732e-03, -6.83110234e-01};

std::vector<float> distCoeffs;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "single_aruco");

    ros::NodeHandle nh;

    // load image in context
    cv::Mat inputImage;
    inputImage = cv::imread("/home/matt/catkin_ws/src/visual_pose_estimator/test/singlemarkersoriginal.jpg");

    // load param
    for (int i = 0; i < 5; i++)
        distCoeffs.push_back(distCoeffsTable[i]);

    cv::imshow("in", inputImage);
    cv::waitKey();

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners;
    cv::Mat outputImage = inputImage.clone();

    cv::aruco::detectMarkers(inputImage, dictionary, corners, ids);
    cv::aruco::drawDetectedMarkers(outputImage, corners, ids);

    // if at least one marker detected
    if (ids.size() > 0) {
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
            cv::aruco::drawAxis(outputImage, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
        }
    }

    cv::imshow("out", outputImage);
    cv::waitKey();

    ros::spinOnce();
    
    return 0;
}

