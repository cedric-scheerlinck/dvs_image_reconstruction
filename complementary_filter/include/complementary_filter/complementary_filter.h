#pragma once

#include <string>
#include <stdio.h>

// boost
#include <boost/thread.hpp>
#include <boost/thread/thread_time.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

// dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <complementary_filter/complementary_filterConfig.h>

#include <cv_bridge/cv_bridge.h>

// messages
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

// google logging
#include <glog/logging.h>
#include <gflags/gflags.h>

namespace complementary_filter
{

class Complementary_filter
{
public:
  Complementary_filter(ros::NodeHandle & nh, ros::NodeHandle nh_private);
  void eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg);
  void set_parameters();
  virtual ~Complementary_filter();

private:
  ros::NodeHandle nh_;

  void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
  void reconfigureCallback(complementary_filter::complementary_filterConfig &config, uint32_t level);

  void initialise_image_states(const uint32_t& rows, const uint32_t& columns);
  void update_log_intensity_state(const double& ts, const int& x, const int& y,
                                  const bool& polarity);
  void update_log_intensity_state_global(const double& ts);
  void recalibrate_cutoff_frequency_array();
  void recalibrate_contrast_thresholds();
  void update_xy_cutoff_frequency(const int& row,const int& col, const double& lower_bound, const double& upper_bound);
  void update_xy_contrast_threshold(const uint32_t& row, const uint32_t& col, const double& log_aps_change,
                                    const double& event_count_on_max, const double& event_count_off_max);
  void minMaxLocRobust(const cv::Mat& image, double* robust_min, double* robust_max,
                                             const double& percentage_pixels_to_discard);
  void publish_intensity_estimate(const ros::Time& timestamp);
  void publish_cutoff_frequency_array(const ros::Time& timestamp);
  void print_if_negative(const cv::Mat array);
  bool log_aps_pixel_within_allowed_range(const uint32_t& row, const uint32_t& col,
                                          const double& min, const double& max);
  void reset_all();

  boost::shared_ptr<dynamic_reconfigure::Server<complementary_filter::complementary_filterConfig> > server_;
  dynamic_reconfigure::Server<complementary_filter::complementary_filterConfig>::CallbackType dynamic_reconfigure_callback_;

  ros::Subscriber event_sub_;

  image_transport::Subscriber image_raw_sub_;
  image_transport::Publisher intensity_estimate_pub_;
  image_transport::Publisher cutoff_frequency_array_pub_;

  // double
  cv::Mat log_intensity_state_;
  cv::Mat log_intensity_aps_frame_last_;
  cv::Mat log_intensity_aps_frame_previous_;
  cv::Mat ts_map_;
  cv::Mat event_count_on_array_;
  cv::Mat event_count_off_array_;
  cv::Mat contrast_threshold_on_array_;
  cv::Mat contrast_threshold_off_array_;
  cv::Mat cutoff_frequency_array_;

  bool log_intensity_state_initialised_;
  bool adaptive_contrast_threshold_;
  bool adaptive_cutoff_frequency_;
  bool save_images_;

  std::string save_dir_;

  int spatial_smoothing_method_;

  double cutoff_frequency_user_defined_; /** rad/s */
  double contrast_threshold_on_user_defined_;
  double contrast_threshold_off_user_defined_;
  double contrast_threshold_on_adaptive_;
  double contrast_threshold_off_adaptive_;
  double t_next_update_log_intensity_state_global_;
  double t_next_recalibrate_contrast_thresholds_;
  double global_log_intensity_state_update_frequency_;
  double contrast_threshold_recalibration_frequency_;
//  double dynamic_range_extension_;
  double intensity_min_;
  double intensity_max_;
  double spatial_filter_sigma_;

};

} // namespace
