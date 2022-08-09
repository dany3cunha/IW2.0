#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <zed_interfaces/PlaneStamped.h>

// Standard includes
#include <stdio.h>
#include <string.h>
#include <std_msgs/String.h>

// ZED includes
#include <sl/Camera.hpp>

// OpenCV includes
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

// OpenCV dep
#include <opencv2/cvconfig.h>
#include <bits/stdc++.h>
#include <iostream>
#include <stdio.h>
#include <stdarg.h>

// Using std and sl namespaces
using namespace std;
using namespace sl;

#define planeDuration 1 // plane duration on rviz publication in seconds
#define open_exposure 10 // inital exposure in opening camera
void meshToPlaneMarker(visualization_msgs::MarkerPtr &plane_marker, sl::Mesh mesh, sl::Pose pose);
void planeAsCustomMessage(zed_interfaces::PlaneStampedPtr &planeMsg, sl::Plane plane);
cv::Mat slMat2cvMat(sl::Mat &input);
/**
 * @brief Mapping between MAT_TYPE and CV_TYPE
 * @param type 
 * @return int 
 */
int getOCVtype(sl::MAT_TYPE type);
void adjustCameraExposure(cv::Mat cv_image, int &exposure);