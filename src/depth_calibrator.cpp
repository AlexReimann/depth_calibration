#include <time.h>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Empty.h>

image_geometry::PinholeCameraModel camera_model_;
cv::Mat depth_multiplier_correction_;

cv::Mat valid_depth_count_;
cv::Mat depth_sum_;

cv::Mat planes_sum_;
unsigned int planes_count_;

std::vector<float> plane_coefficients_;
boost::mutex coefficients_mutex_;

ros::Publisher pub_multiplier_matrix_;
ros::Publisher pub_calibrated_depth_;

double plane_distance_;
std::string calibration_file_path_;

bool calibration_finished_;
bool depth_updated_;

using namespace std;

void camera_info_cb(const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  if (camera_model_.initialized())
    return;

  camera_model_.fromCameraInfo(info_msg);
  if (planes_sum_.empty())
  {
    valid_depth_count_ = cv::Mat::zeros(camera_model_.fullResolution(), CV_64F);
    depth_sum_ = cv::Mat::zeros(camera_model_.fullResolution(), CV_64F);

    planes_sum_ = cv::Mat::zeros(camera_model_.fullResolution(), CV_64F);
    planes_count_ = 0;
  }
}

void depth_image_cb(const sensor_msgs::ImageConstPtr& image_msg)
{
  if(!calibration_finished_)
  {
    pub_calibrated_depth_.publish(image_msg);
    depth_updated_ = true;
    return;
  }

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

  cv::Mat depth_double;
  cv_depth_image->image.convertTo(depth_double, CV_64F);

  depth_double = (depth_double).mul(depth_multiplier_correction_);

  depth_double.convertTo(cv_depth_image->image, CV_16U);
  pub_calibrated_depth_.publish(cv_depth_image->toImageMsg());
  depth_updated_ = true;
}

void update_plane_coeff(const std_msgs::Float32MultiArrayConstPtr& plane_coeffcients)
{
  if (!depth_updated_ || calibration_finished_)
    return;

  coefficients_mutex_.lock();
  plane_coefficients_ = plane_coeffcients->data;
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
  double d = -plane_distance_;

  if(plane_distance_ == 0.0)
  {
    d = plane_coefficients[3];
  }

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

  if (calibration_finished_)
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

  if(plane_distance_ == 0.0)
    ROS_INFO("Estimated plane distance: %f", std::abs(plane_coefficients[3]));

  double min = 0;
  double max = 0;
  cv::minMaxLoc(depth_double, &min, &max);
  ROS_INFO("Measured depth mean = %f, min = %f, max = %f", cv::mean(depth_double)[0], min, max);

  cv::minMaxLoc(plane, &min, &max);
  ROS_INFO("Estimated plane: mean = %f, min = %f, max =  %f", cv::mean(plane)[0], min, max);

  cv::Mat is_valid = (depth_double != 0) / 255;
  cv::Mat is_valid_converted;
  is_valid.convertTo(is_valid_converted, CV_64F);

  valid_depth_count_ += is_valid_converted;
  depth_sum_ += depth_double;

  planes_sum_ += plane;
  planes_count_++;
}

void save_multiplier(const std_msgs::EmptyConstPtr& empty)
{
  ///TODO Make this numerical stable
  depth_multiplier_correction_ = (planes_sum_ / planes_count_) / (depth_sum_ / valid_depth_count_);

  double min = 0;
  double max = 0;
  cv::minMaxLoc(depth_multiplier_correction_, &min, &max);
  ROS_INFO("Multiplier mean: %f, min: %f, max %f", cv::mean(depth_multiplier_correction_)[0], min, max);

  //  void publish_multiplier(); //maybe for later

  calibration_finished_ = true;

  ROS_INFO_STREAM("Saving calibration to " << calibration_file_path_);
  cv::FileStorage file_storage(calibration_file_path_, cv::FileStorage::WRITE);

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

  ros::Subscriber sub_plane = nh.subscribe<std_msgs::Float32MultiArray>("plane_coefficients", 1, update_plane_coeff); ///
  ros::Subscriber sub_save = nh.subscribe<std_msgs::Empty>("save_calibration_trigger", 1, save_multiplier);

  nh_priv.param("calibration_file_path", calibration_file_path_, std::string("camera_info/depth_calibration.yaml"));
  nh_priv.param("plane_distance", plane_distance_, 0.0);

  if(plane_distance_ == 0.0)
  {
    ROS_WARN("plane_distance arg is zero. The average estimated distance will be used which might be inaccurate.");
  }

  int rate = 30;
  ros::Rate spin_rate(rate);

  while (ros::ok())
  {
    spin_rate.sleep();
    ros::spinOnce();
  }
}
