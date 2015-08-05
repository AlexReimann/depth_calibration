#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32MultiArray.h>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>

ros::Publisher pub_plane_coefficients_;

pcl::PointCloud<pcl::PointXYZ>::Ptr center_extraction(pcl::PCLPointCloud2Ptr cloudPtr)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(*cloudPtr, *input_cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr center_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PointXYZ min;
  pcl::PointXYZ max;
  pcl::getMinMax3D(*input_cloud, min, max);

  double used_plane_ratio = 0.8;
  double center_radius_x = (max.x - min.x) * 0.5 * used_plane_ratio;
  double center_radius_y = (max.y - min.y) * 0.5 * used_plane_ratio;

  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(input_cloud);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(-center_radius_x, center_radius_x);
  pass.filter(*temp_cloud);

  pass.setInputCloud(temp_cloud);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-center_radius_y, center_radius_y);
  pass.filter(*center_cloud);

  return center_cloud;
}

boost::shared_ptr<pcl::ModelCoefficients> getPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud)
{
  boost::shared_ptr<pcl::ModelCoefficients> plane_coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices temp_indicies;

  pcl::SACSegmentation<pcl::PointXYZ> segmentation;

  segmentation.setOptimizeCoefficients(true);
  segmentation.setModelType(pcl::SACMODEL_PLANE);
  segmentation.setMethodType(pcl::SAC_RANSAC);

  segmentation.setDistanceThreshold(0.1); //we gotta catch'em all, so set that low
  segmentation.setMaxIterations(1000); //wanna be the very best, so set that high

  segmentation.setInputCloud(temp_cloud);
  segmentation.segment(temp_indicies, *plane_coefficients);

  return plane_coefficients;
}

void publish_coefficients(std::vector<float>& plane_coefficients)
{
  std_msgs::Float32MultiArray plane_msg;
  std::vector<std_msgs::MultiArrayDimension> dimArray;

  std_msgs::MultiArrayDimension dim;
  dim.label = "length";
  dim.size = 4;
  dim.stride = 4;
  dimArray.push_back(dim);

  plane_msg.layout.dim = dimArray;
  plane_msg.data = plane_coefficients;
  pub_plane_coefficients_.publish(plane_msg);
}

void update_plane_coeff(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) //boost::shared_ptr<pcl::PCLPointCloud2> cloudPtr)
{
  boost::shared_ptr<pcl::PCLPointCloud2> cloud(new pcl::PCLPointCloud2);
  pcl::PCLPointCloud2Ptr cloudPtr(cloud);
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_center = center_extraction(cloudPtr);

  if (cloud_center->width * cloud_center->height == 0)
  {
//    ROS_ERROR_THROTTLE(1.0, "Depth calibration could not extract the center of the point cloud.");
    return;
  }

  boost::shared_ptr<pcl::ModelCoefficients> plane_coefficients = getPlane(cloud_center);

  if (plane_coefficients->values.size() == 0)
  {
    ROS_WARN("Plane was not found!");
    return;
  }

  publish_coefficients(plane_coefficients->values);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "center_plane_extractor");
  ros::NodeHandle nh, nh_priv("~");

  pub_plane_coefficients_ = nh.advertise<std_msgs::Float32MultiArray>("plane_coefficients", 1);
  ros::Subscriber sub_cloud = nh.subscribe<sensor_msgs::PointCloud2>("cloud", 1, update_plane_coeff);

  ros::Rate spin_rate(30);

  while (ros::ok())
  {
    spin_rate.sleep();
    ros::spinOnce();
  }
}

