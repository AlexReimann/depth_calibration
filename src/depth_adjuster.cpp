#include <depth_calibration/depth_adjuster.h>

#include <depth_calibration/border_value.h>
#include <pluginlib/class_list_macros.h>

namespace depth_calibration
{
void DepthAdjuster::onInit()
{
  ros::NodeHandle& nh = getNodeHandle();

  sub_depth_raw_ = nh.subscribe<sensor_msgs::Image>("input_depth_raw", 1, &DepthAdjuster::apply_calibration_cb, this);
  pub_calibrated_depth_raw_ = nh.advertise<sensor_msgs::Image>("output_depth_raw", 1);

  sub_camera_info_ = nh.subscribe<sensor_msgs::CameraInfo>("input_camera_info", 1, &DepthAdjuster::relay_camera_info,
                                                           this);
  pub_camera_info_relay_ = nh.advertise<sensor_msgs::CameraInfo>("output_camera_info_relay", 1);

  ros::NodeHandle& private_nh = getPrivateNodeHandle();
  bool enable;
  private_nh.param("enable", enable, true);

  std::string calibration_file_path;
  private_nh.param("calibration_file_path", calibration_file_path, std::string("camera_info/depth_calibration.yaml"));

  if (enable)
    load_calibration(calibration_file_path);
  else
    ROS_INFO("Depth calibration disabled by launch file");

  private_nh.param("unknown_depth_distance", unknown_depth_distance_, 0.0);
  unknown_depth_distance_ = unknown_depth_distance_ * 1000; //convert to depth value, which are in mm
}

void DepthAdjuster::load_calibration(std::string file_path)
{
  cv::FileStorage file_storage(file_path, cv::FileStorage::READ);

  if (!file_storage.isOpened())
  {
    ROS_WARN("Cannot load depth calibration file: No file at %s", file_path.c_str());
    return;
  }

  file_storage["depth_multiplier"] >> depth_multiplier_correction_;
  file_storage.release();

  if (depth_multiplier_correction_.empty())
    ROS_WARN("Depth calibration couldn't be loaded: Couldn't find depth_multiplier in file");
  else
    ROS_INFO("Depth calibration file loaded successfully");
}

void DepthAdjuster::apply_calibration_cb(const sensor_msgs::ImageConstPtr& depth_msg)
{
  if (depth_multiplier_correction_.empty())
  {
    //just relay to make sure stuff still works
    pub_calibrated_depth_raw_.publish(depth_msg);
    return;
  }

  cv_bridge::CvImagePtr cv_depth_image;
  try
  {
    cv_depth_image = cv_bridge::toCvCopy(depth_msg, "16UC1");
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_WARN_THROTTLE(1.0, "cv_bridge exception in depth_adjuster with cv_depth_image: %s", e.what());
    return;
  }

  cv::Mat depth_double;
  cv_depth_image->image.convertTo(depth_double, CV_64F);
  depth_double = depth_double.mul(depth_multiplier_correction_);

  cv::Mat zero_addition = (depth_double == 0); //if true value is set to 255, so divide by 255 later
  zero_addition.convertTo(zero_addition, CV_64F);

  depth_double += (zero_addition * unknown_depth_distance_ / 255.0);

  cv::Mat not_border = (depth_multiplier_correction_ != BORDER_VALUE) / 255.0;
  not_border.convertTo(not_border, CV_64F);
  depth_double = depth_double.mul(not_border);

  depth_double.convertTo(cv_depth_image->image, CV_16U);
  pub_calibrated_depth_raw_.publish(cv_depth_image->toImageMsg());
}

void DepthAdjuster::relay_camera_info(const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  pub_camera_info_relay_.publish(info_msg);
}
}

PLUGINLIB_EXPORT_CLASS(depth_calibration::DepthAdjuster, nodelet::Nodelet)
