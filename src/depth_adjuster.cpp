#include <depth_calibration/depth_adjuster.h>
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
    ROS_WARN("[Depth adjuster] Depth calibration disabled by launch file");

  private_nh.param("unknown_depth_distance", unknown_depth_distance_, 0.0);
  unknown_depth_distance_ = unknown_depth_distance_ * 1000; //convert to depth value, which are in mm

  private_nh.param("border_percentage_top", border_percentage_top_, 0.0);
  private_nh.param("border_percentage_bottom", border_percentage_bottom_, 0.0);
  private_nh.param("border_percentage_left", border_percentage_left_, 0.0);
  private_nh.param("border_percentage_right", border_percentage_right_, 0.0);
}

void DepthAdjuster::load_calibration(std::string file_path)
{
  cv::FileStorage file_storage(file_path, cv::FileStorage::READ);

  if (!file_storage.isOpened())
  {
    ROS_WARN("[Depth adjuster] Cannot load depth calibration file: No file at %s", file_path.c_str());
    return;
  }

  file_storage["depth_multiplier"] >> depth_multiplier_correction_;
  file_storage.release();

  if (depth_multiplier_correction_.empty())
    ROS_WARN("[Depth adjuster] Depth calibration couldn't be loaded: Couldn't find depth_multiplier in file");
  else
    ROS_INFO("[Depth adjuster] Depth calibration file loaded successfully");
}

void DepthAdjuster::apply_calibration_cb(const sensor_msgs::ImageConstPtr& depth_msg)
{
  if (depth_multiplier_correction_.empty())
  {
    //just relay to make sure stuff still works
    pub_calibrated_depth_raw_.publish(depth_msg);
    return;
  }

  if (depth_msg->width != depth_multiplier_correction_.cols || depth_msg->height != depth_multiplier_correction_.rows)
  {
    ROS_ERROR_STREAM("[Depth adjuster] Calibration file has different resolution than camera depth image");
    ROS_ERROR_STREAM(
        "[Depth adjuster] Calibration multiplier: " << depth_multiplier_correction_.cols << "x" << depth_multiplier_correction_.rows << ", depth image: " << depth_msg->width << "x" << depth_msg->height);
    ROS_WARN_STREAM("[Depth adjuster] Skipping depth adjustments.");
    depth_multiplier_correction_.release();
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
    ROS_WARN_THROTTLE(1.0, "[Depth adjuster] cv_bridge exception with cv_depth_image: %s", e.what());
    return;
  }

  cv::Mat depth_double = cv_depth_image->image.clone();
  depth_double.convertTo(depth_double, CV_64F);
  depth_double = (depth_double).mul(depth_multiplier_correction_);

  cv::Mat zero_addition = (depth_double == 0); //if true value is set to 255, so divide by 255 later
  zero_addition.convertTo(zero_addition, CV_64F);

  depth_double += (zero_addition * unknown_depth_distance_ / 255.0);

  if (border_percentage_top_ != 0.0 || border_percentage_bottom_ != 0.0 || border_percentage_left_ != 0.0
      || border_percentage_right_ != 0.0)
  {
    remove_borders(depth_double);
  }

  depth_double.convertTo(cv_depth_image->image, CV_16U);
  pub_calibrated_depth_raw_.publish(cv_depth_image->toImageMsg());
}

void DepthAdjuster::remove_borders(cv::Mat& image)
{
  int border_height_top = image.cols * border_percentage_top_;
  int border_height_bottom = image.cols * border_percentage_bottom_;
  int border_width_left = image.rows * border_percentage_left_;
  int border_width_right = image.rows * border_percentage_right_;
  cv::Mat borders(image.size(), CV_64F, 1.0);

  image(cv::Rect(0, 0, image.cols, border_height_top)).setTo(0); //top
  image(cv::Rect(0, image.rows - border_height_bottom, image.cols, border_height_bottom)).setTo(0); //bottom

  image(cv::Rect(0, 0, border_width_left, image.rows)).setTo(0); //left
  image(cv::Rect(image.cols - border_width_right, 0, border_width_right, image.rows)).setTo(0); //right
}

void DepthAdjuster::relay_camera_info(const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  pub_camera_info_relay_.publish(info_msg);
}
}

PLUGINLIB_EXPORT_CLASS(depth_calibration::DepthAdjuster, nodelet::Nodelet)
