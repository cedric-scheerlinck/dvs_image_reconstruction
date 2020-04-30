#include "complementary_filter/complementary_filter.h"

#include <cv_bridge/cv_bridge.h>
#include <glog/logging.h>
#include <opencv2/opencv.hpp>
#include <fastguidedfilter.h>

#include "complementary_filter/utils.h"

enum {NONE, GAUSSIAN, BILATERAL, GUIDED};


namespace complementary_filter
{

Complementary_filter::Complementary_filter(ros::NodeHandle & nh, ros::NodeHandle nh_private)
{

  // publisher queue size
  const int IMAGE_PUB_QUEUE_SIZE = 1;

  std::string working_dir;
  std::string save_dir;

  // read parameters from launch file
  nh_private.getParam("publish_framerate", publish_framerate_);
  nh_private.getParam("contrast_threshold_recalibration_frequency", contrast_threshold_recalibration_frequency_);
  nh_private.getParam("save_dir", save_dir);
  nh_private.getParam("working_dir", working_dir);

  VLOG(1) << "Found parameter publish_framerate " << publish_framerate_;
  VLOG(1) << "Found parameter contrast_threshold_recalibration_frequency " << contrast_threshold_recalibration_frequency_;

  if (save_dir.empty())
  {
    save_images_ = false;
  }
  else
  {
    save_images_ = true;
    save_dir_ = complementary_filter::utils::create_fullpath(working_dir, save_dir);
    if (save_dir_.back() != '/')
    {
      save_dir_.append("/");
    }

    VLOG(1) << "Saving images to " << save_dir_ ;
  }

  // setup publishers
  image_transport::ImageTransport it_(nh_);
  intensity_estimate_pub_ = it_.advertise("complementary_filter/intensity_estimate", IMAGE_PUB_QUEUE_SIZE);
  cutoff_frequency_array_pub_ = it_.advertise("complementary_filter/cutoff_frequency", IMAGE_PUB_QUEUE_SIZE);
  used_for_contrast_threshold_recalibration_pub_ = it_.advertise("complementary_filter/used_for_ct_calib", IMAGE_PUB_QUEUE_SIZE);
  //guide
  guide_pub_ = it_.advertise("complementary_filter/guide", IMAGE_PUB_QUEUE_SIZE);

  // bool
  initialised_ = false;
  recalibrate_contrast_thresholds_initialised_ = false;

  // dynamic reconfigure
  dynamic_reconfigure_callback_ = boost::bind(&Complementary_filter::reconfigureCallback, this, _1, _2);
  server_.reset(new dynamic_reconfigure::Server<complementary_filter::complementary_filterConfig>(nh_private));
  server_->setCallback(dynamic_reconfigure_callback_);
}

Complementary_filter::~Complementary_filter()
{
  intensity_estimate_pub_.shutdown();
  cutoff_frequency_array_pub_.shutdown();
  used_for_contrast_threshold_recalibration_pub_.shutdown();
}

void Complementary_filter::eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg)
{
  // initialisation only to be performed once at the beginning
  if (!initialised_)
  {
    initialise_image_states(msg->height, msg->width);
  }

  if (msg->events.size() > 0)
  {
//     count events per pixels with polarity
    for (int i = 0; i < msg->events.size(); ++i)
    {
      const double ts = msg->events[i].ts.toSec();
      const int x = msg->events[i].x;
      const int y = msg->events[i].y;
      const bool polarity = msg->events[i].polarity;

      update_log_intensity_state(ts, x, y, polarity);

      if (adaptive_contrast_threshold_)
      {
        // accumulate all events - the arrays will be cleared by recalibrate_contrast_thresholds(timestamp)
        if (polarity)
        {
          event_count_on_array_.at<double>(y, x)++;
        }
        else
        {
          event_count_off_array_.at<double>(y, x)++;
        }
      }

      if (publish_framerate_ > 0 && ts > t_next_publish_)
      {
        update_log_intensity_state_global(ts);
        publish_intensity_estimate(msg->events[i].ts);
        t_next_publish_ = ts + 1 / publish_framerate_;
      }
    }

    const double ts = msg->events.back().ts.toSec();

    if (publish_framerate_ < 0)
    {
      update_log_intensity_state_global(ts);
      publish_intensity_estimate(msg->events.back().ts);
    }
  }
}

void Complementary_filter::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  // make sure everything has been initialised
  if (!initialised_)
  {
    return;
  }
  cv::Mat last_image;
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv_ptr->image.convertTo(last_image, CV_64FC1, 1 / 255.0, 1);  // [1, 2]

  // put logarithm of APS frame into class member variable
  if (last_image.size() == log_intensity_aps_frame_last_.size())
  {
    cv::log(last_image, log_intensity_aps_frame_last_);  // [0, 0.69]
  }

  if (adaptive_contrast_threshold_)
  {
    recalibrate_contrast_thresholds(msg->header.stamp);
  }

  if (adaptive_cutoff_frequency_)
  {
    recalibrate_cutoff_frequency_array();
    publish_cutoff_frequency_array(msg->header.stamp);
  }
}

void Complementary_filter::offlineEventsCallback(const dvs_msgs::EventArray::ConstPtr& msg)
{
  // smart wrapper for eventsCallback that guarantees correct event/image ordering.

  static int image_idx = 0;
  static std::vector<dvs_msgs::Event> events;
  for(auto e : msg->events)
  {
    if (image_idx < image_timestamps_.size() - 1)
    {
      const double image_ts = image_timestamps_[image_idx].toSec();
      if (e.ts.toSec() > image_ts)
      {
        // package events and call eventsCallback
        dvs_msgs::EventArray event_array_msg;
        event_array_msg.events = events;
        event_array_msg.width = msg->width;
        event_array_msg.height = msg->height;
        event_array_msg.header.stamp = events.back().ts;
        dvs_msgs::EventArrayConstPtr event_array_pointer = boost::make_shared<dvs_msgs::EventArray>(event_array_msg);
        eventsCallback(event_array_pointer);
        events.clear();
        // update and set new image
        update_log_intensity_state_global(image_ts);
        if (images_[image_idx].size() == log_intensity_aps_frame_last_.size())
        {
          cv::log(images_[image_idx], log_intensity_aps_frame_last_);
        }
        if (adaptive_contrast_threshold_)
        {
          recalibrate_contrast_thresholds(image_timestamps_[image_idx]);
        }

        if (adaptive_cutoff_frequency_)
        {
          recalibrate_cutoff_frequency_array();
          publish_cutoff_frequency_array(image_timestamps_[image_idx]);
        }
        image_idx++;
      }
      events.push_back(e);
    }
  }
}

void Complementary_filter::initialise_image_states(const uint32_t& rows, const uint32_t& columns)
{
  const double init_on = contrast_threshold_on_user_defined_;  // set by dynamic reconfigure
  const double init_off = contrast_threshold_off_user_defined_;
  // double
  log_intensity_state_ = cv::Mat::zeros(rows, columns, CV_64FC1);
  log_intensity_aps_frame_last_ = cv::Mat::ones(rows, columns, CV_64FC1) * 0.405; // latest frame (log(1.5))
  log_intensity_aps_frame_previous_ = cv::Mat::zeros(rows, columns, CV_64FC1); // one before latest frame
  ts_array_ = cv::Mat::zeros(rows, columns, CV_64FC1); // similar to surface of active events
  event_count_on_array_ = cv::Mat::zeros(rows, columns, CV_64FC1);
  event_count_off_array_ = cv::Mat::zeros(rows, columns, CV_64FC1);
  contrast_threshold_on_array_ = cv::Mat::ones(rows, columns, CV_64FC1) * init_on;
  contrast_threshold_off_array_ = cv::Mat::ones(rows, columns, CV_64FC1) * init_off;
  cutoff_frequency_array_ = cv::Mat::ones(rows, columns, CV_64FC1) * cutoff_frequency_user_defined_;
  // guide
  guide_ = cv::Mat::zeros(rows, columns, CV_64FC1);

  contrast_threshold_on_adaptive_ = init_on;
  contrast_threshold_off_adaptive_ = init_off;

  t_next_publish_ = 0;
  recalibrate_contrast_thresholds_initialised_ = false;

  initialised_ = true;

  VLOG(3) << "Initialised!";
}

void Complementary_filter::update_log_intensity_state(const double& ts, const int& x, const int& y,
    const bool& polarity)
{
  const double delta_t = (ts - ts_array_.at<double>(y, x));
  if (delta_t < 0)
  {
    LOG(WARNING) << "Warning: non-monotonic timestamp detected, resetting...";
    initialise_image_states(log_intensity_state_.rows, log_intensity_state_.cols);
    return;
  }
  double contrast_threshold;
  if (adaptive_contrast_threshold_)
  {
    contrast_threshold = (polarity) ? contrast_threshold_on_adaptive_ : contrast_threshold_off_adaptive_;
//      c = (polarity) ? contrast_threshold_on_array_.at<double>(y, x) : contrast_threshold_off_array_.at<double>(y, x);
  }
  else
  {
    contrast_threshold = (polarity) ? contrast_threshold_on_user_defined_ : contrast_threshold_off_user_defined_;
  }

  const double cutoff_frequency = (adaptive_cutoff_frequency_) ?
      cutoff_frequency_array_.at<double>(y, x) : cutoff_frequency_user_defined_;
  const double beta = std::exp(-cutoff_frequency * delta_t);

  // standard complementary filter update
  log_intensity_state_.at<double>(y, x) = beta * log_intensity_state_.at<double>(y, x)
      + (1 - beta) * log_intensity_aps_frame_last_.at<double>(y, x) + contrast_threshold;

  // guide
  guide_.at<double>(y, x) = std::exp(-guide_fade_ * delta_t) * guide_.at<double>(y, x) + contrast_threshold;

  ts_array_.at<double>(y, x) = ts;
}

void Complementary_filter::update_log_intensity_state_global(const double& ts)
{
  cv::Mat beta;
  cv::Mat delta_t = ts - ts_array_;
//	print_if_negative(delta_t);
  double min;
  double max;
  cv::minMaxLoc(delta_t, &min, &max);
  if (min < 0)
  {
    LOG(WARNING) << "Warning: non-monotonic timestamp detected, resetting...";
    initialise_image_states(log_intensity_state_.rows, log_intensity_state_.cols);
    return;
  }
  if (adaptive_cutoff_frequency_)
  {
    cv::exp(-cutoff_frequency_array_.mul(delta_t), beta);
  }
  else
  {
    cv::exp(-cutoff_frequency_user_defined_ * (delta_t), beta);
  }
  log_intensity_state_ = log_intensity_state_.mul(beta)
      + log_intensity_aps_frame_last_.mul(1 - beta);

  // guide
  cv::Mat guide_beta;
  cv::exp(-guide_fade_ * (delta_t), guide_beta);
  guide_ = guide_.mul(guide_beta);

  ts_array_.setTo(ts);
}

void Complementary_filter::recalibrate_cutoff_frequency_array()
{
  constexpr double log_aps_margin = 0.05;
  constexpr double log_intensity_lower = std::log(1 + log_aps_margin);
  constexpr double log_intensity_upper = std::log(2 - log_aps_margin);

  cv::Mat pixel_compromised =
      (log_intensity_aps_frame_last_ < log_intensity_lower)
    | (log_intensity_aps_frame_last_ > log_intensity_upper);

  cutoff_frequency_array_.setTo(cutoff_frequency_user_defined_);
  for (int row = 0; row < cutoff_frequency_array_.rows; row++)
  {
    for (int col = 0; col < cutoff_frequency_array_.cols; col++)
    {
      if (pixel_compromised.at<uint8_t>(row, col))
      {
        update_xy_cutoff_frequency(row, col, log_intensity_lower, log_intensity_upper);
      }
    }
  }
}

void Complementary_filter::update_xy_cutoff_frequency(const int& row, const int& col,
                                                      const double& lower_bound, const double& upper_bound)
{
  constexpr double min_fraction = 0.1;
  constexpr double max_log_aps_intensity = std::log(2);
  constexpr double min_log_aps_intensity = std::log(1);
  const double log_aps_value = log_intensity_aps_frame_last_.at<double>(row, col);
  double aps_saturation_severity;
  if (log_aps_value <= lower_bound)
  {
    aps_saturation_severity = (min_log_aps_intensity - log_aps_value)/(min_log_aps_intensity - lower_bound);
  }
  else if (log_aps_value >= upper_bound)
  {
    aps_saturation_severity = (max_log_aps_intensity - log_aps_value)/(max_log_aps_intensity - upper_bound);
  }
  cutoff_frequency_array_.at<double>(row, col) =
      (min_fraction + aps_saturation_severity*(1 - min_fraction) )*cutoff_frequency_user_defined_;
}

void Complementary_filter::recalibrate_contrast_thresholds(const ros::Time& timestamp)
{
  static double t_last = 0; // time of last update
  const double ts = timestamp.toSec();
  static cv::Mat log_frame_previous;
  if (!recalibrate_contrast_thresholds_initialised_)
  {
    t_last = 0;
    event_count_on_array_.setTo(0);
    event_count_off_array_.setTo(0);
    log_intensity_aps_frame_last_.copyTo(log_frame_previous);
    recalibrate_contrast_thresholds_initialised_ = true;
    return;
  }

  if ( (contrast_threshold_recalibration_frequency_ > 0)
      && (ts > t_last + 1.0 / contrast_threshold_recalibration_frequency_) )
  {
    // full log intensity APS range [log(1), log(2)]
    constexpr double LOG_INTENSITY_MIN = std::log(1 + 0.15);
    constexpr double LOG_INTENSTIY_MAX = std::log(2 - 0.15);
    constexpr double PERCENTAGE_PIXELS_TO_DISCARD = 1;

    double unused;
    double event_count_on_max;
    double event_count_off_max;

    cv::Mat difference_image = log_intensity_aps_frame_last_ - log_frame_previous;
    cv::Mat valid_pixels = (log_intensity_aps_frame_last_ < LOG_INTENSTIY_MAX)
                         & (log_intensity_aps_frame_last_ > LOG_INTENSITY_MIN)
                         & (log_frame_previous < LOG_INTENSTIY_MAX)
                         & (log_frame_previous > LOG_INTENSITY_MIN);

    cv::Mat used_pix = cv::Mat::zeros(log_intensity_aps_frame_last_.size(), CV_8UC1);

    minMaxLocRobust(event_count_on_array_, &unused, &event_count_on_max, PERCENTAGE_PIXELS_TO_DISCARD);
    minMaxLocRobust(event_count_off_array_, &unused, &event_count_off_max, PERCENTAGE_PIXELS_TO_DISCARD);

    for (int row = 0; row < difference_image.rows; row++)
    {
      for (int col = 0; col < difference_image.cols; col++)
      {
        if (valid_pixels.at<uint8_t>(row, col))
        {
          const double log_aps_change = difference_image.at<double>(row, col);
          update_xy_contrast_threshold(row, col, log_aps_change, event_count_on_max, event_count_off_max, used_pix);
        }
      }
    }

    cv_bridge::CvImage cv_image;
    cv_image.encoding = "mono8";
    cv_image.header.stamp = timestamp;
    cv_image.image = used_pix*255;
    used_for_contrast_threshold_recalibration_pub_.publish(cv_image.toImageMsg());

    contrast_threshold_on_adaptive_ = cv::mean(contrast_threshold_on_array_)[0];
    contrast_threshold_off_adaptive_ = cv::mean(contrast_threshold_off_array_)[0];

    VLOG_IF_EVERY_N(3, adaptive_contrast_threshold_, 5) << "\nAdaptive contrast thresholds OFF, ON: "
        << contrast_threshold_off_adaptive_ << "\t" << contrast_threshold_on_adaptive_;

    event_count_on_array_.setTo(0);
    event_count_off_array_.setTo(0);

    log_intensity_aps_frame_last_.copyTo(log_frame_previous);

    t_last = ts;
  }
}

void Complementary_filter::update_xy_contrast_threshold(const uint32_t& row, const uint32_t& col,
    const double& log_aps_change, const double& event_count_on_max, const double& event_count_off_max, cv::Mat& used_pix)
{

  constexpr double LOG_APS_CHANGE_MIN = 0.03; // ~ fraction of total intensity range
  constexpr double DVS_CHANGE_MIN = 2; // number of events
  constexpr double ON_OFF_RATIO_MIN = 2;
  constexpr double OFF_ON_RATIO_MIN = 2;
  constexpr double ITERATIONS_TO_REACH_95_PERCENT = 10;
  // contrast threshold low pass (exponentially weighted moving
  // average; EWMA) parameter BETA = (1 - 0.95)^(1/N), where N is
  // the number of iterations required to reach 95% of a constant signal.
  constexpr double BETA = std::pow(1 - 0.95, 1.0 / ITERATIONS_TO_REACH_95_PERCENT);

  const double event_count_on = event_count_on_array_.at<double>(row, col);
  const double event_count_off = event_count_off_array_.at<double>(row, col);
  const double on_off_ratio = (event_count_on + 1e-6) / (event_count_off + 1e-6);
  const double dvs_change = event_count_on - event_count_off;

  // positive (ON) change
  if (log_aps_change >= LOG_APS_CHANGE_MIN
      && event_count_on < event_count_on_max
      && dvs_change >= DVS_CHANGE_MIN
      && on_off_ratio > ON_OFF_RATIO_MIN)
  {
    const double measured_contrast_threshold = log_aps_change / dvs_change;
    contrast_threshold_on_array_.at<double>(row, col) =
        BETA * contrast_threshold_on_array_.at<double>(row, col) + (1 - BETA) * measured_contrast_threshold;
    used_pix.at<uint8_t>(row, col) = 1;
  }
  else if (log_aps_change <= -LOG_APS_CHANGE_MIN // negative (OFF) change
        && event_count_off < event_count_off_max
        && dvs_change <= -DVS_CHANGE_MIN
        && 1.0 / on_off_ratio > OFF_ON_RATIO_MIN)
  {
    const double measured_contrast_threshold = -log_aps_change / dvs_change;
    contrast_threshold_off_array_.at<double>(row, col) =
        BETA * contrast_threshold_off_array_.at<double>(row, col) + (1 - BETA) * measured_contrast_threshold;
    used_pix.at<uint8_t>(row, col) = 1;
  }

}

void Complementary_filter::publish_intensity_estimate(const ros::Time& timestamp)
{
  const double display_range = intensity_max_ - intensity_min_;
  cv::Mat display_image;
  cv_bridge::CvImage cv_image;

  cv::exp(log_intensity_state_, display_image); //[1, 2]
  display_image -= 1; // [0, 1]
  display_image -= intensity_min_;
  display_image.convertTo(display_image, CV_8UC1, 255.0/display_range); // [0, 255]

  // guide
  cv::Mat display_guide;
  display_guide = guide_ + 0.5; // [0, 1]
  display_guide.convertTo(display_guide, CV_8UC1, 255.0); // [0, 255]

  if (color_image_)
  {
    cv::Mat color_display_image;
    cv::cvtColor(display_image, color_display_image, CV_BayerBG2BGR);
    display_image = color_display_image;
    cv_image.encoding = "bgr8";
  }
  else
  {
    cv_image.encoding = "mono8";
  }

  cv::Mat filtered_display_image;
  switch (spatial_smoothing_method_) {
    case NONE: filtered_display_image = display_image;
               break;
    case GAUSSIAN: cv::GaussianBlur(display_image, filtered_display_image, cv::Size(5, 5), spatial_filter_sigma_, spatial_filter_sigma_);
                   break;
    case BILATERAL: cv::bilateralFilter(display_image, filtered_display_image, 5, spatial_filter_sigma_, spatial_filter_sigma_);
                    break;
    case GUIDED: if (switch_guide_) {
                     filtered_display_image = fastGuidedFilter(display_guide, display_image, 8, spatial_filter_sigma_, 2);
                 } else {
                     filtered_display_image = fastGuidedFilter(display_image, display_guide, 8, spatial_filter_sigma_, 2);
                 }
                 break;
  }

  cv_image.image = filtered_display_image;
  cv_image.header.stamp = timestamp;
  intensity_estimate_pub_.publish(cv_image.toImageMsg());
  //guide
  cv_image.image = display_guide;
  guide_pub_.publish(cv_image.toImageMsg());

  if (save_images_)
  {
    static int image_counter = 0;
    std::string save_path = save_dir_ + "image" + std::to_string(image_counter) + ".png";
    cv::imwrite(save_path, display_image);
    image_counter++;
  }
}

void Complementary_filter::publish_cutoff_frequency_array(const ros::Time& timestamp)
{
  cv::Mat display_image;

  const double display_range = cutoff_frequency_user_defined_;

  cutoff_frequency_array_.convertTo(display_image, CV_8UC1, 255.0/display_range);

  cv_bridge::CvImage cv_image;
  cv_image.encoding = "mono8";
  cv_image.header.stamp = timestamp;
  cv_image.image = display_image;
  cutoff_frequency_array_pub_.publish(cv_image.toImageMsg());
}

void Complementary_filter::print_if_negative(const cv::Mat array)
{
  double min;
  double max;
  cv::minMaxLoc(array, &min, &max);
  if (min < 0)
  {
    VLOG(3) << "minimum value less than zero: " << min;
  }
}

void Complementary_filter::load_images(const sensor_msgs::Image::ConstPtr& msg)
{

  cv::Mat image;
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv_ptr->image.convertTo(image, CV_64FC1, 1 / 255.0, 1);
  images_.push_back(image);
  image_timestamps_.push_back(msg->header.stamp);

}

void Complementary_filter::minMaxLocRobust(const cv::Mat& image, double* robust_min, double* robust_max,
                                           const double& percentage_pixels_to_discard)
{
  CHECK_NOTNULL(robust_max);
  CHECK_NOTNULL(robust_min);
  cv::Mat image_as_row;
  cv::Mat image_as_row_sorted;
  const int single_row_idx_min = (0.5*percentage_pixels_to_discard/100)*image.total();
  const int single_row_idx_max = (1 - 0.5*percentage_pixels_to_discard/100)*image.total();
  image_as_row = image.reshape(0, 1);
  cv::sort(image_as_row, image_as_row_sorted, CV_SORT_EVERY_ROW + CV_SORT_ASCENDING);
  image_as_row_sorted.convertTo(image_as_row_sorted, CV_64FC1);
  *robust_min = image_as_row_sorted.at<double>(single_row_idx_min);
  *robust_max = image_as_row_sorted.at<double>(single_row_idx_max);
}

void Complementary_filter::reconfigureCallback
    (complementary_filter::complementary_filterConfig &config, uint32_t level)
{
  cutoff_frequency_user_defined_ = config.Cutoff_frequency*2*M_PI;
  intensity_min_ = config.Intensity_min;
  intensity_max_ = config.Intensity_max;
  contrast_threshold_on_user_defined_ = config.Contrast_threshold_ON;
  contrast_threshold_off_user_defined_ = config.Contrast_threshold_OFF;
  adaptive_contrast_threshold_ = config.Auto_detect_contrast_thresholds;
  adaptive_cutoff_frequency_ = config.High_dynamic_range_mode;
  spatial_smoothing_method_ = config.Spatial_filter;
  switch (spatial_smoothing_method_) {
    case NONE: break;
    case GAUSSIAN: spatial_filter_sigma_ = config.Spatial_filter_sigma;
                   break;
    case BILATERAL: spatial_filter_sigma_ = config.Spatial_filter_sigma*20;
                   break;
    case GUIDED: spatial_filter_sigma_ = config.Spatial_filter_sigma*config.Spatial_filter_sigma*650.25;  // 0.1^2*255^2
                 break;
  }
  color_image_ = config.Color_display;
  // guide
  guide_fade_ = config.Guide_fade*2*M_PI;
  switch_guide_ = config.Switch_guide;
}

} // namespace
