#include "../include/complementary_filter/complementary_filter.h"

#include <std_msgs/Float32.h>
#include <glog/logging.h>

enum {GAUSSIAN, BILATERAL};


namespace complementary_filter
{

Complementary_filter::Complementary_filter(ros::NodeHandle & nh, ros::NodeHandle nh_private)
{
  // subscriber queue size
  const int EVENT_SUB_QUEUE_SIZE = 1000;
  const int IMAGE_SUB_QUEUE_SIZE = 100;
  // publisher queue size
  const int INTENSITY_ESTIMATE_PUB_QUEUE_SIZE = 1;
  const int CUTOFF_FREQUENCY_PUB_QUEUE_SIZE = 1;

  // read parameters from launch file
  nh_private.param<double>("global_log_intensity_state_update_frequency", global_log_intensity_state_update_frequency_, 30.0);
  nh_private.param<double>("contrast_threshold_recalibration_frequency", contrast_threshold_recalibration_frequency_, 10.0);

  VLOG(1) << "Found parameter global_log_intensity_state_update_frequency " << global_log_intensity_state_update_frequency_;
  VLOG(1) << "Found parameter contrast_threshold_recalibration_frequency " << contrast_threshold_recalibration_frequency_;

  // setup subscribers and publishers
  event_sub_ = nh_.subscribe("events", EVENT_SUB_QUEUE_SIZE, &Complementary_filter::eventsCallback, this);
  image_transport::ImageTransport it_(nh_);
  image_raw_sub_ = it_.subscribe("image_raw", IMAGE_SUB_QUEUE_SIZE, &Complementary_filter::imageCallback, this);
  intensity_estimate_pub_ = it_.advertise("complementary_filter/intensity_estimate", INTENSITY_ESTIMATE_PUB_QUEUE_SIZE);
  cutoff_frequency_array_pub_ = it_.advertise("complementary_filter/cutoff_frequency", CUTOFF_FREQUENCY_PUB_QUEUE_SIZE);

  // bool
  log_intensity_state_initialised_ = false;

  // double
  t_next_update_log_intensity_state_global_ = 0.0;
  t_next_recalibrate_contrast_thresholds_ = 0.0;

  // dynamic reconfigure
  dynamic_reconfigure_callback_ = boost::bind(&Complementary_filter::reconfigureCallback, this, _1, _2);
  server_.reset(new dynamic_reconfigure::Server<complementary_filter::complementary_filterConfig>(nh_private));
  server_->setCallback(dynamic_reconfigure_callback_);
}

Complementary_filter::~Complementary_filter()
{
  intensity_estimate_pub_.shutdown();
}

void Complementary_filter::eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg)
{
  // initialisation only to be performed once at the beginning
  if (!log_intensity_state_initialised_)
  {
    initialise_image_states(msg->height, msg->width);
    log_intensity_state_initialised_ = true;
  }
  if (msg->events.size() > 0 && intensity_estimate_pub_.getNumSubscribers() >= 1)
  {
//     count events per pixels with polarity
    for (int i = 0; i < msg->events.size(); ++i)
    {
      const double ts = msg->events[i].ts.toSec();
      const int x = msg->events[i].x;
      const int y = msg->events[i].y;
      const bool polarity = msg->events[i].polarity;

      update_log_intensity_state(ts, x, y, polarity);

      if (polarity)
      {
        event_count_on_array_.at<double>(y, x)++;
      } else
      {
        event_count_off_array_.at<double>(y, x)++;
      }
    }

    const double ts = msg->events.back().ts.toSec();

// perform some action every fixed time-interval
    if (ts > t_next_update_log_intensity_state_global_)
    {
      update_log_intensity_state_global(ts);
      t_next_update_log_intensity_state_global_ = ts + 1.0 / global_log_intensity_state_update_frequency_;
      publish_intensity_estimate(msg->events.back().ts);
    }

  }
}

void Complementary_filter::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  // make sure everything has been initialised
  if (log_intensity_state_initialised_)
  {
    const double ts = msg->header.stamp.toSec();
    double minVal;
    double maxVal;
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

    cv_ptr->image.convertTo(last_image, CV_64FC1, 1 / 255.0, 1);
    cv::minMaxLoc(last_image, &minVal, &maxVal);
    VLOG_EVERY_N(4, 10) << "image_raw natural range [" << minVal << ", " << maxVal << "] | expected [1, 2]";

    // put logarithm of APS frame into class member variable
    cv::log(last_image, log_intensity_aps_frame_last_);

    if ((ts > t_next_recalibrate_contrast_thresholds_))
    {
      recalibrate_contrast_thresholds();
      t_next_recalibrate_contrast_thresholds_ = ts + 1.0 / contrast_threshold_recalibration_frequency_;
    }

    recalibrate_cutoff_frequency_array();
    publish_cutoff_frequency_array(msg->header.stamp);
  }
}

void Complementary_filter::initialise_image_states(const uint32_t& rows, const uint32_t& columns)
{
  const double init_on = contrast_threshold_on_user_defined_;  // set by dynamic reconfigure
  const double init_off = contrast_threshold_off_user_defined_;
  // double
  log_intensity_state_ = cv::Mat::zeros(rows, columns, CV_64FC1);
  log_intensity_aps_frame_last_ = cv::Mat::zeros(rows, columns, CV_64FC1);
  log_intensity_aps_frame_previous_ = cv::Mat::zeros(rows, columns, CV_64FC1);
  ts_map_ = cv::Mat::zeros(rows, columns, CV_64FC1); // surface of active events
  event_count_on_array_ = cv::Mat::zeros(rows, columns, CV_64FC1);
  event_count_off_array_ = cv::Mat::zeros(rows, columns, CV_64FC1);

  contrast_threshold_on_array_ = cv::Mat::ones(rows, columns, CV_64FC1) * init_on;
  contrast_threshold_off_array_ = cv::Mat::ones(rows, columns, CV_64FC1) * init_off;
  cutoff_frequency_array_ = cv::Mat::ones(rows, columns, CV_64FC1) * cutoff_frequency_user_defined_;

  contrast_threshold_on_adaptive_ = init_on;
  contrast_threshold_off_adaptive_ = init_off;
}

void Complementary_filter::update_log_intensity_state(const double& ts, const int& x, const int& y,
    const bool& polarity)
{
  const double delta_t = (ts - ts_map_.at<double>(y, x));
  if (delta_t < 0)
  {
		reset_all();
  } else
  {
    double contrast_threshold;
    if (adaptive_contrast_threshold_)
    {
      contrast_threshold = (polarity) ? contrast_threshold_on_adaptive_ : contrast_threshold_off_adaptive_;
//      c = (polarity) ? contrast_threshold_on_array_.at<double>(y, x) : contrast_threshold_off_array_.at<double>(y, x);
    } else
    {
      contrast_threshold = (polarity) ? contrast_threshold_on_user_defined_ : contrast_threshold_off_user_defined_;
    }

    const double cutoff_frequency = (adaptive_cutoff_frequency_) ?
        cutoff_frequency_array_.at<double>(y, x) : cutoff_frequency_user_defined_;
    const double beta = std::exp(-cutoff_frequency * delta_t);

    // standard complementary filter update
    log_intensity_state_.at<double>(y, x) = beta * log_intensity_state_.at<double>(y, x)
        + (1 - beta) * log_intensity_aps_frame_last_.at<double>(y, x) + contrast_threshold;

    ts_map_.at<double>(y, x) = ts;
  }
}

void Complementary_filter::update_log_intensity_state_global(const double& ts)
{
  cv::Mat beta;
  cv::Mat delta_t = ts - ts_map_;
//	print_if_negative(delta_t);
  double min;
  double max;
  cv::minMaxLoc(delta_t, &min, &max);
  if (min < 0)
  {
    reset_all();
    VLOG(2) << "negative delta_t detected during update_log_intensity_state(const double& ts)" << std::endl;
  } else
  {
    if (adaptive_cutoff_frequency_)
    {
      cv::exp(-cutoff_frequency_array_.mul(delta_t), beta);
    } else
    {
      cv::exp(-cutoff_frequency_user_defined_ * (delta_t), beta);
    }
    log_intensity_state_ = log_intensity_state_.mul(beta)
        + log_intensity_aps_frame_last_.mul(1 - beta);
    ts_map_.setTo(ts);
  }
}

void Complementary_filter::recalibrate_cutoff_frequency_array()
{
  constexpr double log_aps_margin = 0.05;
  constexpr double log_intensity_lower = std::log(1 + log_aps_margin);
  constexpr double log_intensity_upper = std::log(2 - log_aps_margin);

  cutoff_frequency_array_.setTo(cutoff_frequency_user_defined_);
  for (int row = 0; row < cutoff_frequency_array_.rows; row++)
  {
    for (int col = 0; col < cutoff_frequency_array_.cols; col++)
    {
      if (!log_aps_pixel_within_allowed_range(row, col, log_intensity_lower, log_intensity_upper))
      {
        update_xy_cutoff_frequency(row, col, log_intensity_lower, log_intensity_upper);
//        VLOG_EVERY_N(3, 10000) << "log APS pixel outside range " << log_intensity_aps_frame_last_.at<double>(row, col);
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
  } else if (log_aps_value >= upper_bound)
  {
    aps_saturation_severity = (max_log_aps_intensity - log_aps_value)/(max_log_aps_intensity - upper_bound);
  }
  cutoff_frequency_array_.at<double>(row, col) =
      (min_fraction + aps_saturation_severity*(1 - min_fraction) )*cutoff_frequency_user_defined_;
}

void Complementary_filter::recalibrate_contrast_thresholds()
{
  constexpr double PERCENTAGE_PIXELS_TO_DISCARD = 1;

  double unused;
  double event_count_on_max;
  double event_count_off_max;

  cv::Mat difference_image = log_intensity_aps_frame_last_ - log_intensity_aps_frame_previous_;

  minMaxLocRobust(event_count_on_array_, &unused, &event_count_on_max, PERCENTAGE_PIXELS_TO_DISCARD);
  minMaxLocRobust(event_count_off_array_, &unused, &event_count_off_max, PERCENTAGE_PIXELS_TO_DISCARD);

  for (int row = 0; row < difference_image.rows; row++)
  {
    for (int col = 0; col < difference_image.cols; col++)
    {
      const double log_aps_change = difference_image.at<double>(row, col);
      update_xy_contrast_threshold(row, col, log_aps_change, event_count_on_max, event_count_off_max);
    }
  }

  contrast_threshold_on_adaptive_ = cv::mean(contrast_threshold_on_array_)[0];
  contrast_threshold_off_adaptive_ = cv::mean(contrast_threshold_off_array_)[0];

	VLOG_IF_EVERY_N(3, adaptive_contrast_threshold_, 5) << "\nAdaptive contrast thresholds OFF, ON: "
	    << contrast_threshold_off_adaptive_ << "\t" << contrast_threshold_on_adaptive_;

	double min_on, max_on;
	double min_off, max_off;
	cv::minMaxLoc(event_count_on_array_, &min_on, &max_on);
	cv::minMaxLoc(event_count_off_array_, &min_off, &max_off);
	const double mean_on = cv::mean(event_count_on_array_)[0];
	const double mean_off = cv::mean(event_count_off_array_)[0];

//	VLOG_IF_EVERY_N(3, adaptive_contrast_threshold_, 5) << "ON, OFF, [min, mean, max]:\n[" << min_on
//	    << ", " << mean_on << ", " << max_on << "]\t[" << min_off << ", " << mean_off << ", " << max_off << "]";

  event_count_on_array_.setTo(0);
  event_count_off_array_.setTo(0);

  log_intensity_aps_frame_last_.copyTo(log_intensity_aps_frame_previous_);
}

void Complementary_filter::update_xy_contrast_threshold(const uint32_t& row, const uint32_t& col,
    const double& log_aps_change, const double& event_count_on_max, const double& event_count_off_max)
{
  // full log intensity APS range [log(1), log(2)]
  constexpr double LOG_INTENSITY_MIN = std::log(1 + 0.15);
  constexpr double LOG_INTENSTIY_MAX = std::log(2 - 0.15);
  constexpr double LOG_APS_CHANGE_MIN = 0.03; // ~ fraction of total intensity range
  constexpr double DVS_CHANGE_MIN = 2; // number of events
  constexpr double ON_OFF_RATIO_MIN = 2;
  constexpr double OFF_ON_RATIO_MIN = 2;
  constexpr double ITERATIONS_TO_REACH_95_PERCENT = 10;
  // contrast threshold low pass (exponentially weighted moving
  // average; EWMA) parameter BETA = (1 - 0.95)^(1/N), where N is
  // the number of iterations required to reach 95% of a constant signal.
  constexpr double BETA = std::pow(1 - 0.95, 1.0 / ITERATIONS_TO_REACH_95_PERCENT);

  if (log_aps_pixel_within_allowed_range(row, col, LOG_INTENSITY_MIN, LOG_INTENSTIY_MAX))
  {
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
    } else if // negative (OFF) change
        (log_aps_change <= -LOG_APS_CHANGE_MIN
        && event_count_off < event_count_off_max
        && dvs_change <= -DVS_CHANGE_MIN
        && 1.0 / on_off_ratio > OFF_ON_RATIO_MIN)
    {
      const double measured_contrast_threshold = -log_aps_change / dvs_change;
      contrast_threshold_off_array_.at<double>(row, col) =
          BETA * contrast_threshold_off_array_.at<double>(row, col) + (1 - BETA) * measured_contrast_threshold;
    }
  }
}

void Complementary_filter::publish_intensity_estimate(const ros::Time& timestamp)
{
  cv::Mat display_image;

  cv::exp(log_intensity_state_, display_image); //[1, 2]

  display_image -= 1; // [0, 1]

  display_image -= intensity_min_;

  const double display_range = intensity_max_ - intensity_min_;

  display_image.convertTo(display_image, CV_8UC1, 255.0/display_range);

  cv::Mat filtered_display_image;
  if (spatial_filter_sigma_ > 0)
  {
    if (spatial_smoothing_method_ == GAUSSIAN)
    {
      cv::GaussianBlur(display_image, filtered_display_image, cv::Size(5, 5), spatial_filter_sigma_, spatial_filter_sigma_);
    } else if (spatial_smoothing_method_ == BILATERAL)
    {
      const double bilateral_sigma = spatial_filter_sigma_*20;
      cv::bilateralFilter(display_image, filtered_display_image, 5, bilateral_sigma, bilateral_sigma);
    }
  } else
  {
    filtered_display_image = display_image;
  }

  cv_bridge::CvImage cv_image;
  cv_image.encoding = "mono8";
  cv_image.header.stamp = timestamp;
  cv_image.image = filtered_display_image;
  intensity_estimate_pub_.publish(cv_image.toImageMsg());
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

bool Complementary_filter::log_aps_pixel_within_allowed_range
    (const uint32_t& row, const uint32_t& col, const double& min, const double& max)
{
  const bool returnValue = (
         log_intensity_aps_frame_last_.at<double>(row, col) < max
      && log_intensity_aps_frame_last_.at<double>(row, col) > min
      && log_intensity_aps_frame_previous_.at<double>(row, col) < max
      && log_intensity_aps_frame_previous_.at<double>(row, col) > min);

  return returnValue;
}

void Complementary_filter::reset_all()
{
  ts_map_.setTo(0);
  log_intensity_state_.setTo(0);
  log_intensity_aps_frame_last_.setTo(0);
  log_intensity_aps_frame_previous_.setTo(0);
  event_count_on_array_.setTo(0);
  event_count_off_array_.setTo(0);
  t_next_recalibrate_contrast_thresholds_ = 0;
  t_next_update_log_intensity_state_global_ = 0;
  VLOG(3) << "full reset";
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
  spatial_filter_sigma_ = config.Spatial_filter_sigma;
  spatial_smoothing_method_ = int(config.Bilateral_filter);
}

} // namespace
