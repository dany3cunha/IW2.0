#include <ros/ros.h>
#include <shape_msgs/Plane.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

ros::Publisher pub;
pcl::PointCloud<pcl::PointXYZRGB> my_cloud;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
  // Container for original & filtered data
  pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;

  sor.setInputCloud(cloudPtr);
  sor.setLeafSize(0.01, 0.01, 0.01);
  sor.filter(cloud_filtered);

  std::cout << " Original: width " << cloud->width << " height " << cloud->height << std::endl;
  std::cout << " Filter: width " << cloud_filtered.width << " height " << cloud_filtered.height << std::endl;

  // Convert to ROS data type
  // sensor_msgs::PointCloud2 output;
  //::moveFromPCL(cloud_filtered, output);

  pcl::fromPCLPointCloud2(cloud_filtered, my_cloud);
}

bool is_OnPlane(boost::array<double, 4UL> coef, pcl::PointXYZRGB point, float threshold)
{
  auto a = coef.at(0);
  auto b = coef.at(1);
  auto c = coef.at(2);
  auto d = coef.at(3);
  int OFFSET = 0;

  if (a * point.x + b * point.y + c * point.z <= (d + OFFSET) * (1 + threshold) )
    if (a * point.x + b * point.y + c * point.z >= (d + OFFSET) * (1 - threshold) )
      return true;
  return false;
}

void plane_cb(const shape_msgs::Plane plane)
{

  for (int i = 0; i < my_cloud.size(); i++)
  {
    if (!is_OnPlane(plane.coef, my_cloud.at(i), 0.1))
    {
      my_cloud.at(i) = my_cloud.at(my_cloud.size() - 1);
      my_cloud.resize(my_cloud.size() - 1);
      --i;
    }
  }

  pcl::PCLPointCloud2 virtual_Laser;
  pcl::toPCLPointCloud2(my_cloud, virtual_Laser);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(virtual_Laser, output);

  // Publish the data
  pub.publish(output);

  return;
}
int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub1 = nh.subscribe<sensor_msgs::PointCloud2>("/zed2/zed_node/point_cloud/cloud_registered", 1, cloud_cb);
  ros::Subscriber sub2 = nh.subscribe<shape_msgs::Plane>("/zed2_floor_plane", 1, plane_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);

  // Spin
  ros::spin();
}
