#include <time.h>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Empty.h>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>

image_geometry::PinholeCameraModel camera_model_;
cv::Mat depth_multiplier_correction_;
boost::mutex multiplier_mutex_;

std::vector<float> plane_coefficients_;
boost::mutex coefficients_mutex_;

ros::Publisher pub_multiplier_matrix_;
ros::Publisher pub_calibrated_depth_;

bool calibration_finished_;
bool depth_updated_;

using namespace std;

void camera_info_cb(const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  if (camera_model_.initialized())
    return;

  camera_model_.fromCameraInfo(info_msg);
  if (depth_multiplier_correction_.empty())
    depth_multiplier_correction_ = cv::Mat::ones(camera_model_.fullResolution(), CV_64F);
}

void depth_image_cb(const sensor_msgs::ImageConstPtr& image_msg)
{
  cv_bridge::CvImagePtr cv_depth_image;
  try
  {
    cv_depth_image = cv_bridge::toCvCopy(image_msg, "16UC1");
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception with cv_depth_image: %s", e.what());
    return;
  }

  if (multiplier_mutex_.try_lock())
  {
    cv::Mat depth_double;
    cv_depth_image->image.convertTo(depth_double, CV_64F);

    depth_double = (depth_double).mul(depth_multiplier_correction_);
    multiplier_mutex_.unlock();

    depth_double.convertTo(cv_depth_image->image, CV_16U);
    pub_calibrated_depth_.publish(cv_depth_image->toImageMsg());
    depth_updated_ = true;
  }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr center_extraction(pcl::PCLPointCloud2Ptr cloudPtr)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(*cloudPtr, *input_cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr center_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PointXYZ min;
  pcl::PointXYZ max;
  pcl::getMinMax3D(*input_cloud, min, max);

  double center_radius_x = (max.x - min.x) * 0.5 * 0.3; //30%
  double center_radius_y = (max.y - min.y) * 0.5 * 0.3; //30%

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
  // Optional
  segmentation.setOptimizeCoefficients(true);
  // Mandatory
  segmentation.setModelType(pcl::SACMODEL_PLANE);
  segmentation.setMethodType(pcl::SAC_RANSAC);
  segmentation.setDistanceThreshold(0.0005); //we gotta catch'em all, so set that low
  segmentation.setMaxIterations(1000); //wanna be the very best, so set that high

  segmentation.setInputCloud(temp_cloud);
  segmentation.segment(temp_indicies, *plane_coefficients);

  return plane_coefficients;
}

void update_plane_coeff(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) //boost::shared_ptr<pcl::PCLPointCloud2> cloudPtr)
{
  if (!depth_updated_ || calibration_finished_)
    return;

  boost::shared_ptr<pcl::PCLPointCloud2> cloud(new pcl::PCLPointCloud2);
  pcl::PCLPointCloud2Ptr cloudPtr(cloud);
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_center = center_extraction(cloudPtr);

  if (cloud_center->width * cloud_center->height == 0)
  {
    ROS_ERROR_THROTTLE(1.0, "Depth calibration could not extract the center of the point cloud.");
    return;
  }

  boost::shared_ptr<pcl::ModelCoefficients> plane_coefficients = getPlane(cloud_center);

  if (plane_coefficients->values.size() == 0)
  {
    ROS_WARN("Plane was not found!");
    return;
  }

  coefficients_mutex_.lock();
  plane_coefficients_ = plane_coefficients->values;
  coefficients_mutex_.unlock();
}

cv::Mat make_column_index_matrix(unsigned int height, unsigned int width)
{
  cv::Mat index_matrix(height, width, CV_64F);

  for (unsigned int row_index = 0; row_index < index_matrix.rows; ++row_index)
  {
    double* row_ptr = index_matrix.ptr<double>(row_index);

    for (int column_index = 0; column_index < index_matrix.cols; ++column_index)
      row_ptr[column_index] = column_index;
  }

  return index_matrix;
}

cv::Mat calculate_plane(std::vector<float>& plane_coefficients, unsigned int height, unsigned int width)
{
  double center_x = camera_model_.cx();
  double center_y = camera_model_.cy();

  double mm_to_m = 0.001;
  double constant_x = mm_to_m / camera_model_.fx();
  double constant_y = mm_to_m / camera_model_.fy();

  double a = plane_coefficients[0];
  double b = plane_coefficients[1];
  double c = plane_coefficients[2];
  double d = plane_coefficients[3];

  cv::Mat x_index = make_column_index_matrix(height, width);
  cv::Mat y_index = make_column_index_matrix(width, height).t();

//  ROS_INFO("plane coeff: %f, %f, %f, %f", plane_coefficients[0], plane_coefficients[1],
//           plane_coefficients[2], plane_coefficients[3]);

  return (-1 * d) / (a * (x_index - center_x) * constant_x + b * (y_index - center_y) * constant_y + c * mm_to_m);
}

void publish_multiplier()
{
  ROS_ERROR("Publishing the multiplier was never tested, so check the stuff!");

  std_msgs::Float64MultiArray multiplier_msg;

  std::vector<std_msgs::MultiArrayDimension> dimArray;

  std_msgs::MultiArrayDimension dim;
  dim.label = "height";
  dim.size = depth_multiplier_correction_.rows;
  dim.stride = depth_multiplier_correction_.rows * depth_multiplier_correction_.cols;
  dimArray.push_back(dim);

  dim = std_msgs::MultiArrayDimension();
  dim.label = "width";
  dim.size = depth_multiplier_correction_.cols;
  dim.stride = depth_multiplier_correction_.cols;
  dimArray.push_back(dim);

  multiplier_msg.layout.dim = dimArray;

  const double* data_pointer_begin = depth_multiplier_correction_.ptr<double>(0);
  const double* data_pointer_end = data_pointer_begin
      + depth_multiplier_correction_.rows * depth_multiplier_correction_.cols;
  std::vector<double> multiplier_vector(data_pointer_begin, data_pointer_end);

  multiplier_msg.data = multiplier_vector;

  pub_multiplier_matrix_.publish(multiplier_msg);
}

void calibration_cb(const sensor_msgs::ImageConstPtr& image_msg)
{
  boost::mutex::scoped_lock multiplier_lock(multiplier_mutex_);

  if(calibration_finished_)
    return;

  cv_bridge::CvImagePtr cv_depth_image;
  try
  {
    cv_depth_image = cv_bridge::toCvCopy(image_msg, "16UC1");
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception with cv_depth_image: %s", e.what());
    return;
  }

  std::vector<float> plane_coefficients;
  coefficients_mutex_.lock();
  if (plane_coefficients_.size() == 0)
  {
    coefficients_mutex_.unlock();
    return;
  }
  depth_updated_ = false;

  plane_coefficients = std::vector<float>(plane_coefficients_);
  plane_coefficients_.clear();
  coefficients_mutex_.unlock();

  cv::Mat depth_double;
  cv_depth_image->image.convertTo(depth_double, CV_64F);

  cv::Mat plane = calculate_plane(plane_coefficients, depth_double.rows, depth_double.cols);

  double min = 0;
  double max = 0;
  cv::minMaxLoc(plane, &min, &max);
  ROS_INFO("Estimated plane mean: %f, min: %f, max %f", cv::mean(plane)[0], min, max);

  //update incrementally
  depth_multiplier_correction_ = (plane / depth_double).mul(depth_multiplier_correction_);

  cv::minMaxLoc(depth_multiplier_correction_, &min, &max);
  ROS_INFO("Multiplier mean: %f, min: %f, max %f", cv::mean(depth_multiplier_correction_)[0], min, max);
  ROS_INFO("---------------------------------------------------------------------------");

//  void publish_multiplier(); //maybe for later
}

void save_multiplier(const std_msgs::EmptyConstPtr& empty)
{
  boost::mutex::scoped_lock multiplier_lock(multiplier_mutex_);
  calibration_finished_ = true;

  ROS_INFO("Saving calibration");
  cv::FileStorage file_storage("camera_info/depth_calibration.yaml", cv::FileStorage::WRITE);

  time_t rawtime;
  time(&rawtime);

  file_storage << "calibration_date" << asctime(localtime(&rawtime));
  file_storage << "depth_multiplier" << depth_multiplier_correction_;
  file_storage.release();

  ROS_INFO("Saving finished.");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "depth_calibrator");
  ros::NodeHandle nh, nh_priv("~");

  calibration_finished_ = false;
  depth_updated_ = false;

  pub_calibrated_depth_ = nh.advertise<sensor_msgs::Image>("calibrated_depth", 1);

  ros::Subscriber sub_depth_camera_info = nh.subscribe<sensor_msgs::CameraInfo>("camera_info_relay", 1, camera_info_cb);
  ros::Subscriber sub_depth_image = nh.subscribe<sensor_msgs::Image>("depth_relay", 1, depth_image_cb);
  ros::Subscriber sub_calibration = nh.subscribe<sensor_msgs::Image>("calibrated_depth", 1, calibration_cb);
  ros::Subscriber sub_cloud = nh.subscribe<sensor_msgs::PointCloud2>("calibrated_cloud", 1, update_plane_coeff);
  ros::Subscriber sub_save = nh.subscribe<std_msgs::Empty>("save_calibration_trigger", 1, save_multiplier);

  int rate = 30;
  ros::Rate spin_rate(rate);

  while (ros::ok())
  {
    spin_rate.sleep();
    ros::spinOnce();
  }
}
