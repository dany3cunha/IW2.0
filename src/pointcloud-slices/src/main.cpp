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

#define relative_height_default 0.2
#define relative_starting_default 0.1
using namespace std;

ros::Publisher pub;
boost::array<double, 4UL> current_planeCoefs;

bool use_VoxelGrid = true;
double dist_slices = 0.05;
double safe_zone = 1.3;
double danger_zone = 0.8;
bool static_height = false;
double static_height_val = 1.0;
double relative_height = relative_height_default;
double relative_starting_height = relative_starting_default;

/**
 * @brief At start read the params from the launch file
 *
 */
void readROSparams();
/**
 * @brief save plane equation in a global variable current_planeCoefs
 *
 * @param plane
 */
void plane_cb(const shape_msgs::Plane plane)
{
  // Update current plane coefficients
  current_planeCoefs = plane.coef;
}
/**
 * @brief calculates the distance of the camera to a given point
 *
 * @param point
 * @return float
 */
float euclideanDistance(pcl::PointXYZRGB point)
{
  return sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2));
}
/**
 * @brief Verify if the global variable current_planeCoefs if is empty
 *
 * @return true
 * @return false
 */
bool planeCoefsEmpty()
{
  if (current_planeCoefs.at(0) == 0 && current_planeCoefs.at(1) == 0 && current_planeCoefs.at(2) == 0)
    return true;
  return false;
}
/**
 * @brief Verify if a given point belongs to the actual plane stored in the currente plane
 *
 * @param point Point of the PointCloud
 * @param OFFSET Need the offset for the position in height for the slice
 * @return true
 * @return false
 */
bool is_OnCurrentPlane(pcl::PointXYZRGB point, double OFFSET)
{
  double threshold = 0.005;
  auto a = current_planeCoefs.at(0);
  auto b = current_planeCoefs.at(1);
  auto c = current_planeCoefs.at(2);
  auto d = current_planeCoefs.at(3);

  if (a * point.x + b * point.y + c * point.z <= (OFFSET + threshold))
    if (a * point.x + b * point.y + c * point.z >= (OFFSET - threshold))
      return true;
  return false;
}
/**
 * @brief Create a PointCloudSlices object for diferents OFFSETs
 *
 * @param my_cloud
 * @return sensor_msgs::PointCloud2
 */
sensor_msgs::PointCloud2 create_PointCloudSlices(pcl::PointCloud<pcl::PointXYZRGB> my_cloud)
{

  pcl::PointCloud<pcl::PointXYZRGB> my_cloud2;
  my_cloud2 = my_cloud;
  my_cloud2.clear();

  double plane_coef_d = current_planeCoefs.at(3);

  double init_OFFSET = (1 + relative_height) * plane_coef_d;
  double curr_OFFSET = -plane_coef_d + relative_starting_height * init_OFFSET;
  if (static_height)
  {
    init_OFFSET = static_height_val;
  }

  while (curr_OFFSET <= init_OFFSET)
  {
    for (int i = 0; i < my_cloud.size(); i++)
    {

      if (is_OnCurrentPlane(my_cloud.at(i), curr_OFFSET))
      {
        my_cloud2.push_back(my_cloud.at(i));
        pcl::PointXYZRGB point = my_cloud2.points.at(my_cloud2.size() - 1);
        float distance = euclideanDistance(point);
        if (distance >= safe_zone)
        {
          my_cloud2.points.at(my_cloud2.size() - 1).r = 0;
          my_cloud2.points.at(my_cloud2.size() - 1).g = 255;
          my_cloud2.points.at(my_cloud2.size() - 1).b = 0;
        }
        else if (distance >= danger_zone)
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

    curr_OFFSET = curr_OFFSET + dist_slices;
  }
  pcl::PCLPointCloud2 virtual_Laser;
  pcl::toPCLPointCloud2(my_cloud2, virtual_Laser);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(virtual_Laser, output);

  return output;
}
/**
 * @brief Callback for every PointCloud published
 *
 * @param cloud_msg
 */
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
    // No Voxel Grid filter
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

  readROSparams();

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub1 = nh.subscribe<sensor_msgs::PointCloud2>("/zed2/zed_node/point_cloud/cloud_registered", 1, cloud_cb);
  ros::Subscriber sub2 = nh.subscribe<shape_msgs::Plane>("/zed2_floor_plane", 1, plane_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2>("/output", 1);

  // Spin
  ros::spin();
}

void readROSparams()
{
  if (ros::param::has("/safe_zone"))
    ros::param::get("/safe_zone", safe_zone);

  if (ros::param::has("/danger_zone"))
    ros::param::get("/danger_zone", danger_zone);

  if (ros::param::has("/distance_between_slices"))
    ros::param::get("/distance_between_slices", dist_slices);

  if (ros::param::has("/static_height"))
    ros::param::get("/static_height", static_height);

  if (ros::param::has("/static_height_value"))
    ros::param::get("/static_height_value", static_height_val);

  if (ros::param::has("/relative_starting_height"))
    ros::param::get("/relative_starting_height", relative_starting_height);

  if (ros::param::has("/relative_height_value") && !static_height)
  {
    ros::param::get("/relative_height_value", relative_height);
    if (relative_height >= 1.0 || relative_height <= 0.0)
      relative_height = relative_height_default;
  }
  if (ros::param::has("/use_voxelgrid"))
    ros::param::get("/use_voxelgrid", use_VoxelGrid);

  std::cout << std::endl << std::endl << relative_starting_height << std::endl << std::endl;
}