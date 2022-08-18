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

ros::Publisher pub;
pcl::PointCloud<pcl::PointXYZRGB> my_cloud;
float euclideanDistance(pcl::PointXYZRGB point);
// float minZ = -INFINITY;

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
  
  // std::cout << " Original: width " << cloud->width << " height " << cloud->height << std::endl;
  // std::cout << " Filter: width " << cloud_filtered.width << " height " << cloud_filtered.height << std::endl;

  // Convert to ROS data type
  // sensor_msgs::PointCloud2 output;
  // pcl_conversions::moveFromPCL(cloud_filtered, output);

  // Publish the data
  // pub.publish(output);

  //Filter
  pcl::fromPCLPointCloud2(cloud_filtered, my_cloud);
  // Don't filter
  //pcl::fromPCLPointCloud2(*cloud, my_cloud);
}
float threshold = 0.005;

bool is_OnPlane(boost::array<double, 4UL> coef, pcl::PointXYZRGB point, float OFFSET)
{
  auto a = coef.at(0);
  auto b = coef.at(1);
  auto c = coef.at(2);
  auto d = coef.at(3);
  // float OFFSET = -0.8;

  // if (a * point.x + b * point.y + c * point.z <= (d + OFFSET) * (1 + threshold))
  //   if (a * point.x + b * point.y + c * point.z >= (d + OFFSET) * (1 - threshold))
  if (a * point.x + b * (point.y-0.2) + c * point.z <= (OFFSET + threshold))
    if (a * point.x + b * (point.y-0.2) + c * point.z >= (OFFSET - threshold))
      return true;
  return false;
}

void plane_cb(const shape_msgs::Plane plane)
{
  /*
  for (int i = 0; i < my_cloud.size(); i++)
  {
    if (!is_OnPlane(plane.coef, my_cloud.at(i), -0.8))
    {
      my_cloud.at(i) = my_cloud.at(my_cloud.size() - 1);
      my_cloud.resize(my_cloud.size() - 1);
      --i;
    }
  }
  */

  pcl::PointCloud<pcl::PointXYZRGB> my_cloud2;

  my_cloud2 = my_cloud;

  my_cloud2.clear();

  float init_OFFSET = 0.7;
  /*if (minZ > 0)
    init_OFFSET = minZ;
  else
    init_OFFSET = -minZ;
    */
  float incr = 0.05;
  float curr_OFFSET = -init_OFFSET;
  while (curr_OFFSET <= init_OFFSET)
  {
    for (int i = 0; i < my_cloud.size(); i++)
    {

      if (is_OnPlane(plane.coef, my_cloud.at(i), curr_OFFSET))
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

  // Publish the data
  pub.publish(output);

  return;
}

void minZ_cb(const visualization_msgs::Marker msg)
{

  /*
  minZ = -INFINITY;
  for (int cnt = 0; cnt < msg.points.size(); cnt++)
  {
    if (msg.points[cnt].z > minZ)
    {
      minZ = msg.points[cnt].z;
    }
  }
  return;
  */
}

float euclideanDistance(pcl::PointXYZRGB point)
{
  return sqrt(pow(point.x,2) + pow(point.y,2) + pow(point.z,2));
}

int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub1 = nh.subscribe<sensor_msgs::PointCloud2>("/zed2/zed_node/point_cloud/cloud_registered", 1, cloud_cb);
  ros::Subscriber sub2 = nh.subscribe<shape_msgs::Plane>("/zed2_floor_plane", 1, plane_cb);
  ros::Subscriber sub3 = nh.subscribe<visualization_msgs::Marker>("/zed2/zed_node/plane_marker", 1, minZ_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);

  // Spin
  ros::spin();
}
