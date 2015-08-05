#include <depth_calibration/depth_adjuster.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <pluginlib/class_list_macros.h>
#include <deque>

namespace depth_calibration
{

static double m_to_depth = 1000;

void DepthAdjuster::onInit()
{
  ros::NodeHandle& nh = getNodeHandle();

  sub_depth_raw_ = nh.subscribe<sensor_msgs::Image>("input_depth_raw", 1, &DepthAdjuster::apply_calibration_cb, this);
  pub_calibrated_depth_raw_ = nh.advertise<sensor_msgs::Image>("output_depth_raw", 1);
  pub_inlier_ = nh.advertise<sensor_msgs::Image>("inlier", 1);

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

  private_nh.param("max_distance", max_distance_, 0.0);
  max_distance_ = max_distance_ * m_to_depth;

  private_nh.param("unknown_depth_distance", unknown_depth_distance_, 0.0);
  unknown_depth_distance_ = unknown_depth_distance_ * m_to_depth;

  private_nh.param("is_occluded_percentage", is_occluded_percentage_, 1.0);
  private_nh.param("occluded_distance", occluded_distance_, 0.0);

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
  if (!depth_multiplier_correction_.empty()
      && (depth_msg->width != depth_multiplier_correction_.cols
          || depth_msg->height != depth_multiplier_correction_.rows))
  {
    ROS_ERROR_STREAM("[Depth adjuster] Calibration file has different resolution than camera depth image");
    ROS_ERROR_STREAM(
        "[Depth adjuster] Calibration multiplier: " << depth_multiplier_correction_.cols << "x" << depth_multiplier_correction_.rows << ", depth image: " << depth_msg->width << "x" << depth_msg->height);
    ROS_WARN_STREAM("[Depth adjuster] Skipping depth calibration adjustments.");
    depth_multiplier_correction_.release();
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
  if (!depth_multiplier_correction_.empty())
  {
    depth_double = (depth_double).mul(depth_multiplier_correction_);
  }

//  removeOutliers(depth_double);

  cv::Mat zero_addition = (depth_double == 0); //if true value is set to 255, so divide by 255 later
  int unknown_distances_count = cv::countNonZero(zero_addition);

  if (unknown_distances_count <= depth_double.total() * is_occluded_percentage_)
  {
    zero_addition.convertTo(zero_addition, CV_64F);
    depth_double += (zero_addition * unknown_depth_distance_ / 255.0);
  }
  else
  {
    ROS_WARN_THROTTLE(5.0, "3D sensors seems to be occluded");
    zero_addition.convertTo(zero_addition, CV_64F);
    depth_double += (zero_addition * occluded_distance_ * m_to_depth / 255.0);
  }

  if (border_percentage_top_ != 0.0 || border_percentage_bottom_ != 0.0 || border_percentage_left_ != 0.0
      || border_percentage_right_ != 0.0)
  {
    remove_borders(depth_double);
  }

  if (max_distance_ != 0.0)
  {
    depth_double.convertTo(depth_double, CV_32F); ///TODO change type?
    cv::threshold(depth_double, depth_double, max_distance_, 0.0, cv::THRESH_TRUNC);
  }

  depth_double.convertTo(cv_depth_image->image, CV_16U);

  pub_calibrated_depth_raw_.publish(cv_depth_image->toImageMsg());
}

void DepthAdjuster::removeOutliers(cv::Mat& image)
{
  ///TODO border treatment
  cv::Mat number_of_close_points = cv::Mat::zeros(image.size(), image.type());
  double threshold = 0.2 * m_to_depth;

  int boder_size = 1;
  int kernel_width = 2 * boder_size + 1;

  cv::Point2i current;
  double min = 0;
  double max = 0;

//  unsigned int double_size = sizeof(double);
//  uchar* it = image.data + (image.cols * boder_size + boder_size) * double_size;
////  double* it_end = image.end<double>() - image.cols * boder_size - boder_size;
//  uchar* it_counter = number_of_close_points.data
//      + (number_of_close_points.cols * boder_size + boder_size) * double_size;
//  uchar* neighbour;
//
//  std::vector<uchar*> offsets;
//
//  for (int y = 0; y < kernel_width; ++y)
//  {
//    for (int x = 0; x < kernel_width; ++x)
//    {
//        offsets.push_back(image.data + (y * image.cols + x) * double_size);
//    }
//  }
//
//  int max_count = image.total() - (image.cols * boder_size + boder_size) * 2;
//
//  for (int k = 0; k < max_count; ++k)
//  {
////    ROS_INFO_STREAM("k: " << k);
//    min = *it - threshold;
//    max = *it + threshold;
//    for (int i = 0; i < offsets.size(); ++i)
//    {
////      ROS_INFO_STREAM("i: " << i);
//      neighbour = offsets[i];
//      if (*neighbour > min && *neighbour < max)
//      {
//        ++(*it_counter);
//      }
//      offsets[i] += double_size;
//    }
//    it += double_size;
//    it_counter += double_size;
//  }

  std::deque<const double*> rows;
  for (int y = 0; y < kernel_width - 1; ++y)
  {
    rows.push_back(image.ptr<double>(y));
  }

  int count = 0;
  double value = 0;
  double* counter;
  for (int row = boder_size; row < image.rows - boder_size; ++row)
  {
    rows.push_back(image.ptr<double>(row + boder_size));
    counter = number_of_close_points.ptr<double>(row);
    for (int col = boder_size; col < image.cols - boder_size; ++col)
    {
      value = rows[boder_size + 1][col];
      min = value - threshold;
      max = value + threshold;
      count = 0;

      for (int i = 0; i < kernel_width; ++i)
      {
//        value = rows[i][col - 2];
//        if (value > min && value < max)
//        {
//          ++count;
//        }
        value = rows[i][col - 1];
        if (value > min && value < max)
        {
          ++count;
        }
        value = rows[i][col];
        if (value > min && value < max)
        {
          ++count;
        }
        value = rows[i][col + 1];
        if (value > min && value < max)
        {
          ++count;
        }
//        value = rows[i][col + 2];
//        if (value > min && value < max)
//        {
//          ++counter[col];
//        }
      }

      counter[col] = count;
    }
    rows.pop_front();
  }

//  double value = 0;
//
//  for (int y = boder_size; y < image.rows - boder_size; y++)
//  {
//    const double* row = image.ptr<double>(y);
//    double* count_row = number_of_close_points.ptr<double>(y);
//
//    for (int x = boder_size; x < image.cols - boder_size; x++)
//    {
//      cv::Mat roi = image(cv::Rect(cv::Point(x - boder_size, y - boder_size), cv::Size(kernel_width, kernel_width)));
//      value = row[x];
//
//      min = value - threshold;
//      max = value + threshold;
//
//      count_row[x] = cv::sum((roi > min & roi < max) / 255)[0];
//    }
//  }
  int limit = 4;//includes itself
  cv::Mat inlier = (number_of_close_points > limit) / 255;

  inlier.convertTo(inlier, CV_32F);
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "32FC1", inlier).toImageMsg();
  pub_inlier_.publish(msg);

//  inlier.convertTo(inlier, CV_64F);
//  image = image.mul(inlier);
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
