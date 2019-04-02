#pragma once

#include <string>
#include <stdio.h>

// boost
#include <boost/thread.hpp>
#include <boost/thread/thread_time.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

// dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <pure_event_reconstruction/pure_event_reconstructionConfig.h>

// messages
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

// google logging
#include <glog/logging.h>
#include <gflags/gflags.h>

namespace pure_event_reconstruction
{

class High_pass_filter
{
public:
  High_pass_filter(ros::NodeHandle & nh, ros::NodeHandle nh_private);
  void eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg);
  void set_parameters();
  virtual ~High_pass_filter();

private:
  ros::NodeHandle nh_;
  void reconfigureCallback(pure_event_reconstruction::pure_event_reconstructionConfig &config, uint32_t level);
  void initialise_image_states(const uint32_t& rows, const uint32_t& columns);
  void process_event_msg(const dvs_msgs::EventArray::ConstPtr& msg);

  void update_state_local(const double& delta_t,
                          const int& x,
                          const int& y,
                          const bool& polarity);
  void update_state_global(cv::Mat& delta_t_array);


  void update_log_intensity_state(const double& delta_t,
                                  const int& x,
                                  const int& y,
                                  const bool& polarity);
  void update_log_intensity_state_global(cv::Mat& delta_t_array);
  void update_bias_state(const double& delta_t,
                         const int& x,
                         const int& y);
  void update_bias_state_global(cv::Mat& delta_t_array);
  void update_leaky_event_count(const double& ts,
                                const int& x,
                                const int& y,
                                const bool& polarity);
  void recalibrate_contrast_thresholds(const double& ts);
  void publish_intensity_estimate(const ros::Time& ts);
  void publish_bias_state(const ros::Time& ts);
  void convert_log_intensity_state_to_display_image(cv::Mat& display_image, const double& ts);
  void minMaxLocRobust(const cv::Mat& image,
                       double* lower_bound,
                       double* upper_bound,
                       const double& percentage_pixels_to_discard);

  // dynamic reconfigure
  boost::shared_ptr<dynamic_reconfigure::Server<pure_event_reconstruction::pure_event_reconstructionConfig>>
  server_;
  dynamic_reconfigure::Server<pure_event_reconstruction::pure_event_reconstructionConfig>::CallbackType
  dynamic_reconfigure_callback_;

  // publishers
  image_transport::Publisher intensity_estimate_pub_;
  image_transport::Publisher bias_pub_;

  // internal image states
  cv::Mat log_intensity_state_;
  cv::Mat leaky_event_count_on_; // used to calibrate contrast threshold
  cv::Mat leaky_event_count_off_;
  cv::Mat bias_state_;

  // last update time-stamp map (similar to surface of active events)
  cv::Mat ts_array_; // for log_intensity_state_
  cv::Mat ts_array_on_; // for leaky_event_count_on_
  cv::Mat ts_array_off_; // for leaky_event_count_off_

  bool initialised_;
  bool adaptive_contrast_threshold_;
  bool adaptive_dynamic_range_;
  bool save_images_;
  bool color_image_;
  bool second_order_;

  std::string save_dir_;

  int spatial_smoothing_method_;

  // cutoff frequencies
  double cutoff_frequency_global_; /** rad/s */
  double cutoff_frequency_per_event_component_;
  double event_count_cutoff_frequency_;
  double cutoff_frequency_bias_;

  // contrast thresholds
  double contrast_threshold_on_user_defined_;
  double contrast_threshold_off_user_defined_;
  double contrast_threshold_on_adaptive_; // held constant by convention
  double contrast_threshold_off_adaptive_;

  // update-frequency parameters
  double publish_framerate_;
  double t_next_publish_;
  double t_next_recalibrate_contrast_thresholds_;
  double t_next_log_intensity_update_;

  // display image parameters
  double intensity_min_user_defined_;
  double intensity_max_user_defined_;
  double spatial_filter_sigma_;

//  double event_count_total_;

};

} // namespace
