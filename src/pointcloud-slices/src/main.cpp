#include <ros/ros.h>
#include <shape_msgs/Plane.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <cmath>

using namespace std;

ros::Publisher pub;
boost::array<double, 4UL> current_planeCoefs;

bool use_VoxelGrid = true;

void plane_cb(const shape_msgs::Plane plane)
{
  // Update current plane coefficients
  current_planeCoefs = plane.coef;
}

float euclideanDistance(pcl::PointXYZRGB point)
{
  return sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2));
}

bool planeCoefsEmpty()
{
  if (current_planeCoefs.at(0) == 0 && current_planeCoefs.at(1) == 0 && current_planeCoefs.at(2) == 0)
    return true;
  return false;
}

bool is_OnCurrentPlane(pcl::PointXYZRGB point, double OFFSET)
{
  double threshold = 0.005;
  auto a = current_planeCoefs.at(0);
  auto b = current_planeCoefs.at(1);
  auto c = current_planeCoefs.at(2);
  auto d = current_planeCoefs.at(3);
  // float OFFSET = -0.8;
  // if (a * point.x + b * point.y + c * point.z <= (d + OFFSET) * (1 + threshold))
  //   if (a * point.x + b * point.y + c * point.z >= (d + OFFSET) * (1 - threshold))
  // std::cout << "d: " << d << std::endl;

  if (a * point.x + b * (point.y) + c * point.z <= (OFFSET + threshold))
    if (a * point.x + b * (point.y) + c * point.z >= (OFFSET - threshold))
      return true;
  return false;
}

sensor_msgs::PointCloud2 create_PointCloudSlices(pcl::PointCloud<pcl::PointXYZRGB> my_cloud)
{

  pcl::PointCloud<pcl::PointXYZRGB> my_cloud2;
  my_cloud2 = my_cloud;
  my_cloud2.clear();

  double plane_coef_d = current_planeCoefs.at(3);
  double init_OFFSET = 1.2*plane_coef_d;
  double incr = 0.05;
  double curr_OFFSET = -plane_coef_d + 0.10*init_OFFSET;

  while (curr_OFFSET <= init_OFFSET)
  {
    for (int i = 0; i < my_cloud.size(); i++)
    {

      if (is_OnCurrentPlane(my_cloud.at(i), curr_OFFSET))
      {
        my_cloud2.push_back(my_cloud.at(i));
        pcl::PointXYZRGB point = my_cloud2.points.at(my_cloud2.size() - 1);
        float distance = euclideanDistance(point);
        if (distance >= 1.3)
        {
          my_cloud2.points.at(my_cloud2.size() - 1).r = 0;
          my_cloud2.points.at(my_cloud2.size() - 1).g = 255;
          my_cloud2.points.at(my_cloud2.size() - 1).b = 0;
        }
        else if (distance >= 0.8)
        {
          my_cloud2.points.at(my_cloud2.size() - 1).r = 255;
          my_cloud2.points.at(my_cloud2.size() - 1).g = 255;
          my_cloud2.points.at(my_cloud2.size() - 1).b = 0;
        }
        else
        {
          my_cloud2.points.at(my_cloud2.size() - 1).r = 255;
          my_cloud2.points.at(my_cloud2.size() - 1).g = 0;
          my_cloud2.points.at(my_cloud2.size() - 1).b = 0;
        }
      }
    }

    curr_OFFSET = curr_OFFSET + incr;
  }
  pcl::PCLPointCloud2 virtual_Laser;
  pcl::toPCLPointCloud2(my_cloud2, virtual_Laser);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(virtual_Laser, output);

  return output;
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
  if (planeCoefsEmpty())
    return;

  // Container for original & filtered data
  pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  pcl::PointCloud<pcl::PointXYZRGB> final_cloud;
  if (use_VoxelGrid)
  {
    // VoxelGrid filter
    pcl::PCLPointCloud2 cloud_filtered;
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;

    sor.setInputCloud(cloudPtr);
    sor.setLeafSize(0.01, 0.01, 0.01);
    sor.filter(cloud_filtered);

    pcl::fromPCLPointCloud2(cloud_filtered, final_cloud);
  }
  else
  {
    // No filter
    pcl::fromPCLPointCloud2(*cloud, final_cloud);
  }

  sensor_msgs::PointCloud2 output = create_PointCloudSlices(final_cloud);

  pub.publish(output);
}

int main(int argc, char **argv)
{

  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

  ros::init(argc, argv, "pointcloud_slices");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub1 = nh.subscribe<sensor_msgs::PointCloud2>("/zed2/zed_node/point_cloud/cloud_registered", 1, cloud_cb);
  ros::Subscriber sub2 = nh.subscribe<shape_msgs::Plane>("/zed2_floor_plane", 1, plane_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2>("/output", 1);

  // Spin
  ros::spin();
}