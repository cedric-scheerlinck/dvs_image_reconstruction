#pragma once

#include <complementary_filter/complementary_filterConfig.h>
#include <dvs_msgs/EventArray.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/mat.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <string>

namespace complementary_filter
{

class Complementary_filter
{
public:
  Complementary_filter(ros::NodeHandle & nh, ros::NodeHandle nh_private);
  void eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg);
  void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
  void offlineEventsCallback(const dvs_msgs::EventArray::ConstPtr& msg);
  void load_images(const sensor_msgs::Image::ConstPtr& msg);

  virtual ~Complementary_filter();

private:
  ros::NodeHandle nh_;

  void reconfigureCallback(complementary_filter::complementary_filterConfig &config, uint32_t level);

  void initialise_image_states(const uint32_t& rows, const uint32_t& columns);
  void update_log_intensity_state(const double& ts, const int& x, const int& y,
                                  const bool& polarity);
  void update_log_intensity_state_global(const double& ts);
  void recalibrate_cutoff_frequency_array();
  void recalibrate_contrast_thresholds(const ros::Time& timestamp);
  void update_xy_cutoff_frequency(const int& row,const int& col, const double& lower_bound, const double& upper_bound);
  void update_xy_contrast_threshold(const uint32_t& row, const uint32_t& col, const double& log_aps_change,
                                    const double& event_count_on_max, const double& event_count_off_max, cv::Mat& used_pix);
  void minMaxLocRobust(const cv::Mat& image, double* robust_min, double* robust_max,
                                             const double& percentage_pixels_to_discard);
  void publish_intensity_estimate(const ros::Time& timestamp);
  void publish_cutoff_frequency_array(const ros::Time& timestamp);
  void print_if_negative(const cv::Mat array);

  boost::shared_ptr<dynamic_reconfigure::Server<complementary_filter::complementary_filterConfig> > server_;
  dynamic_reconfigure::Server<complementary_filter::complementary_filterConfig>::CallbackType dynamic_reconfigure_callback_;

  image_transport::Publisher intensity_estimate_pub_;
  image_transport::Publisher cutoff_frequency_array_pub_;
  image_transport::Publisher used_for_contrast_threshold_recalibration_pub_;

  std::vector<ros::Time> image_timestamps_; // only to be used in offline mode
  std::vector<cv::Mat> images_;

  // double
  cv::Mat log_intensity_state_;
  cv::Mat log_intensity_aps_frame_last_;
  cv::Mat log_intensity_aps_frame_previous_;
  cv::Mat ts_array_;
  cv::Mat event_count_on_array_;
  cv::Mat event_count_off_array_;
  cv::Mat contrast_threshold_on_array_;
  cv::Mat contrast_threshold_off_array_;
  cv::Mat cutoff_frequency_array_;


  bool initialised_;
  bool adaptive_contrast_threshold_;
  bool adaptive_cutoff_frequency_;
  bool save_images_;
  bool recalibrate_contrast_thresholds_initialised_;

  std::string save_dir_;

  int spatial_smoothing_method_;

  double cutoff_frequency_user_defined_; /** rad/s */
  double contrast_threshold_on_user_defined_;
  double contrast_threshold_off_user_defined_;
  double contrast_threshold_on_adaptive_;
  double contrast_threshold_off_adaptive_;
  double t_next_publish_;
  double publish_framerate_;
  double contrast_threshold_recalibration_frequency_;
  double intensity_min_;
  double intensity_max_;
  double spatial_filter_sigma_;

};

} // namespace
