#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
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

#include "opencv2/core/cuda.hpp"
#include <opencv2/cudaimgproc.hpp>

// OpenCV dep
#include <opencv2/cvconfig.h>
#include <bits/stdc++.h>
#include <iostream>
#include <stdio.h>
#include <stdarg.h>

// Using std and sl namespaces
using namespace std;
using namespace sl;

#define ROS_loopRate 5
#define planeDuration 1 // plane duration on rviz publication in seconds
#define open_exposure 10 // inital exposure in opening camera
#define open_gain 10 // constant pretended gain 

#define minExposure_thres 0.05 // minimal exposure percentage, if lower than this value then exposure increase
#define maxExposure_thres 5 // maximum exposure percentage, if grater than this value then exposure decreases

void meshToPlaneMarker(visualization_msgs::MarkerPtr &plane_marker, sl::Mesh mesh, sl::Pose pose);
void planeAsCustomMessage(zed_interfaces::PlaneStampedPtr &planeMsg, sl::Plane plane);
cv::Mat slMat2cvMat(sl::Mat &input);
cv::cuda::GpuMat slMat2cvCudaMat(sl::Mat &input);
/**
 * @brief Mapping between MAT_TYPE and CV_TYPE
 * @param type 
 * @return int 
 */
int getOCVtype(sl::MAT_TYPE type);
void adjustCameraExposure(cv::cuda::GpuMat cv_image, int &exposure);
void planesVectors(Plane plane, Camera zed);
