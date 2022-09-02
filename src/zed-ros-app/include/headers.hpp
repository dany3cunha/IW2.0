#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <zed_interfaces/PlaneStamped.h>

// Standard includes
#include <stdio.h>
#include <iostream>
#include <string.h>
#include <std_msgs/String.h>

// ZED includes
#include <sl/Camera.hpp>

// OpenCV includes
#include <opencv2/opencv.hpp>
#include <opencv2/core/utility.hpp>

#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaimgproc.hpp>

// Using std and sl namespaces
using namespace std;
using namespace sl;

int ROS_loopRate = 5;
bool printCameraInfo = true;
int planeDuration = 1;  // plane duration on rviz publication in seconds
int open_exposure = 10; // inital exposure in opening camera
int open_gain = 10;     // constant pretended gain

bool use_MEC = true; // Use manual exposure control while ZED SDK don't fix auto exposure control for local streaming
bool printExposureInfo = true;
double minExposure_thres = 0.05; // minimal exposure percentage, if lower than this value then exposure increase
double maxExposure_thres = 5.00; // maximum exposure percentage, if grater than this value then exposure decreases

double hsv_percentage = 0.0;
/**
 * @brief converts the mesh to visualization_msgs::MarkerPtr for ROS publish
 *
 * @param plane_marker
 * @param mesh
 * @param pose
 */
void meshToPlaneMarker(visualization_msgs::MarkerPtr &plane_marker, sl::Mesh mesh, sl::Pose pose);

void planeAsCustomMessage(zed_interfaces::PlaneStampedPtr &planeMsg, sl::Plane plane);

/**
 * @brief  Converts sl:Mat object to cv::cuda::GpuMat
 * 
 * @param input 
 * @return cv::cuda::GpuMat 
 */
cv::cuda::GpuMat slMat2cvCudaMat(sl::Mat &input);

/**
 * @brief Uses OpenCV functions to manipulate image and adjust the camera exposure manually
 *
 * @param cv_image
 * @param exposure
 */
void adjustCameraExposure(cv::cuda::GpuMat cv_image, int &exposure);
/**
 * @brief At start read the params from the launch file
 *
 */
void readROSparams();
