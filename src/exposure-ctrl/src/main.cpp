#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

// Standard includes
#include <stdio.h>
#include <string.h>
#include <std_msgs/String.h>

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

#define ROS_loopRate 10
#define open_exposure 10 // inital exposure in opening camera
#define open_gain 10     // constant pretended gain

#define minExposure_thres 0.05 // minimal exposure percentage, if lower than this value then exposure increase
#define maxExposure_thres 5    // maximum exposure percentage, if grater than this value then exposure decreases

int exposure = open_exposure;

void img_cb(const sensor_msgs::ImageConstPtr &msg)
{
    cv::Mat image_zed_gpu = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::cuda::GpuMat frame_left_cuda;
        std::cout<<"teste"<<std::endl;
    frame_left_cuda.upload(image_zed_gpu);
        std::cout<<"testea"<<std::endl;

    cv::cuda::cvtColor(frame_left_cuda, frame_left_cuda, cv::COLOR_BGR2HSV);
    frame_left_cuda.download(image_zed_gpu);

    int sum = 0;

    for (int i = 0; i < image_zed_gpu.rows; i++)
    {
        for (int j = 0; j < image_zed_gpu.cols; j++)
        {
            cv::Vec3b HSV = image_zed_gpu.at<cv::Vec3b>(i, j);
            if (HSV.val[2] > 200)
                sum++;
        }
    }

    double x = 100 * double(sum) / (double(image_zed_gpu.rows) * double(image_zed_gpu.cols));

    if (x > maxExposure_thres)
    {
        if (exposure > 1)
        {
            exposure--;
        }
        else
        {
            exposure = 0;
        }
    }
    if (x < minExposure_thres)
    {
        if (exposure <= 99)
        {
            exposure++;
        }
    }

    frame_left_cuda.~GpuMat();
    image_zed_gpu.~Mat();

    std::cout << "exposure: " << exposure << std::endl;
    return;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "exposure_ctrl");
    ros::NodeHandle nh;

    ros::Publisher cmd_ConfigPub = nh.advertise<std_msgs::String>("/zed2_cmd_config", 1, false);

    ros::Subscriber sub1 = nh.subscribe<sensor_msgs::Image>("/zed2/zed_node/rgb/image_rect_color", 1, img_cb);


    // Setup the gain to a fixed value at start
    /*    std_msgs::String msg;
        msg.data = to_string((int)sl::VIDEO_SETTINGS::GAIN) + "," + to_string(open_gain);
        cmd_ConfigPub.publish(msg);

        int exposure = open_exposure;*/

    ros::spin();
}