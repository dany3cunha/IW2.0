#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_frame");
    ros::NodeHandle node;

    tf::TransformBroadcaster br;
    tf::Transform transform;

    ros::Rate rate(10.0);

    while (node.ok())
    {

        transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
        transform.setRotation(tf::createQuaternionFromRPY(-M_PI / 2, 0, 0));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "zed2_left_camera_frame", "lidar_frame"));
        rate.sleep();
    }
    return 0;
};
