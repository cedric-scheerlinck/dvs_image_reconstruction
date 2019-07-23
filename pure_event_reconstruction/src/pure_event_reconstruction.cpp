#include "pure_event_reconstruction/pure_event_reconstruction.h"
#include <std_msgs/Float32.h>
#include <glog/logging.h>
#include "pure_event_reconstruction/utils.h"
//#include "matrix_exponential/r8lib.h"
//#include "matrix_exponential/matrix_exponential.h"
#include <math.h>

enum {GAUSSIAN, BILATERAL};

namespace pure_event_reconstruction
{

High_pass_filter::High_pass_filter(ros::NodeHandle & nh, ros::NodeHandle nh_private)
{
  constexpr int IMAGE_PUB_QUEUE_SIZE = 1;
  constexpr double EVENT_RETENTION_DURATION = 30;  // seconds. Used for calibrating contrast thresholds.

  std::string working_dir;
  std::string save_dir;

  nh_private.getParam("publish_framerate", publish_framerate_);
  nh_private.getParam("save_dir", save_dir);
  nh_private.getParam("working_dir", working_dir);

  VLOG(1) << "Found parameter publish_framerate " << publish_framerate_;

  if (save_dir.empty())
  {
    save_images_ = false;
  }
  else
  {
    save_images_ = true;
    save_dir_ = pure_event_reconstruction::utils::fullpath(working_dir, save_dir);
    if (save_dir_.back() != '/')
    {
      save_dir_.append("/");
    }

    VLOG(1) << "Saving images to " << save_dir_ ;
  }

  // setup publishers
  image_transport::ImageTransport it_(nh_);
  image_state_pub_ = it_.advertise("pure_event_reconstruction/intensity_estimate", IMAGE_PUB_QUEUE_SIZE);
  bias_pub_ = it_.advertise("pure_event_reconstruction/bias", IMAGE_PUB_QUEUE_SIZE);
  second_order_pub_ = it_.advertise("pure_event_reconstruction/second_order", IMAGE_PUB_QUEUE_SIZE);
  // flags and counters
  initialised_ = false;

  // low-pass parameter to reach 95% of a constant signal in EVENT_RETENTION_DURATION seconds.
  event_count_cutoff_frequency_ = -std::log(1 - 0.95)/EVENT_RETENTION_DURATION;  // rad/s.

  contrast_threshold_on_adaptive_ = 0.1; // fixed by convention
  contrast_threshold_off_adaptive_  = -0.1;

  t_next_publish_ = 0.0;
  t_next_recalibrate_contrast_thresholds_ = 0.0;
  t_next_log_intensity_update_ = 0.0;
//  event_count_total_ = 0.0;

  // dynamic reconfigure
  dynamic_reconfigure_callback_ = boost::bind(&High_pass_filter::reconfigureCallback, this, _1, _2);
  server_.reset(new dynamic_reconfigure::Server<pure_event_reconstruction::pure_event_reconstructionConfig>(nh_private));
  server_->setCallback(dynamic_reconfigure_callback_);
}

High_pass_filter::~High_pass_filter()
{
  image_state_pub_.shutdown();
  bias_pub_.shutdown();
  second_order_pub_.shutdown();
}

void High_pass_filter::eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg)
{
  // initialisation only to be performed once at the beginning
  if (!initialised_)
  {
    initialise_image_states(msg->height, msg->width);
  }

  if (msg->events.size() > 0)
  {
    process_event_msg(msg);

    const ros::Time ros_ts = msg->events.back().ts;
    const double ts = ros_ts.toSec();
//
//    if (adaptive_contrast_threshold_ && (ts > t_next_recalibrate_contrast_thresholds_ ))
//    {
//      constexpr double contrast_threshold_recalibration_frequency = 20.0;  // Hz
//      recalibrate_contrast_thresholds(ts);
//      t_next_recalibrate_contrast_thresholds_ = ts + 1 / contrast_threshold_recalibration_frequency;
//    }
//
    if (publish_framerate_ < 0)
    {
      cv::Mat delta_t_array;
      delta_t_array = ts - ts_array_;
//      update_bias_state_global(delta_t_array);
      double min;
      cv::minMaxLoc(delta_t_array, &min, nullptr);
      if (min < 0)
      {
        LOG(WARNING) << "Warning: non-monotonic timestamp detected, resetting...";
        initialise_image_states(second_order_state_.rows, second_order_state_.cols);
        return;
      }
      update_log_intensity_state_global(delta_t_array);
      update_state_global_cedric(delta_t_array);
      publish_log_image(log_image_state_, ros_ts, image_state_pub_, "first_order");
      publish_log_image(second_order_state_, ros_ts, second_order_pub_, "second_order");
      publish_raw_image(bias_state_, ros_ts, bias_pub_, "bias");
      ts_array_.setTo(ts);
    }
  }

//  LOG_EVERY_N(INFO, 100) << event_count_total_;
}

void High_pass_filter::process_event_msg(const dvs_msgs::EventArray::ConstPtr& msg)
{
  cv::Mat delta_t_array;

  // count events per pixels with polarity
  for (int i = 0; i < msg->events.size(); ++i)
  {
    const int x = msg->events[i].x;
    const int y = msg->events[i].y;
    if (x < msg->width && x > 0 && y < msg->height && y > 0) // preserve border
    {
      const ros::Time ros_ts = msg->events[i].ts;
      const double ts = ros_ts.toSec();
      const bool polarity = msg->events[i].polarity;
      const double delta_t = (ts - ts_array_.at<double>(y, x));
      if (delta_t < 0)
      {
        LOG(WARNING) << "Warning: non-monotonic timestamp detected, resetting...";
        initialise_image_states(second_order_state_.rows, second_order_state_.cols);
        return;
      }

      // pixel-wise update
      if (adaptive_contrast_threshold_)
      {
        update_leaky_event_count(ts, x, y, polarity);
      }

//      if (second_order_)
//      {
//        update_bias_state(delta_t, x, y);
//      }
//
      update_log_intensity_state(delta_t, x, y, polarity);
//      update_state_local(delta_t, x, y, polarity);
      update_state_local_cedric(delta_t, x, y, polarity);

      ts_array_.at<double>(y, x) = ts; // reset timestamp map at pixel
  
      if (publish_framerate_ > 0 && ts > t_next_publish_)
      {
        cv::Mat delta_t_array;
        delta_t_array = ts - ts_array_;
        double min;
        cv::minMaxLoc(delta_t_array, &min, nullptr);
        if (min < 0)
        {
          LOG(WARNING) << "Warning: non-monotonic timestamp detected, resetting...";
          initialise_image_states(second_order_state_.rows, second_order_state_.cols);
          return;
        }
        update_log_intensity_state_global(delta_t_array);
        update_state_global_cedric(delta_t_array);
        publish_log_image(log_image_state_, ros_ts, image_state_pub_, "first_order");
        publish_log_image(second_order_state_, ros_ts, second_order_pub_, "second_order");
        publish_raw_image(bias_state_, ros_ts, bias_pub_, "bias");
        ts_array_.setTo(ts);
        t_next_publish_ = ts + 1 / publish_framerate_;
      }

      // global update
//      if (publish_framerate_ > 0 && ts > t_next_publish_)
//      {
//        delta_t_array = ts - ts_array_;
//        double min;
//        cv::minMaxLoc(delta_t, &min, nullptr);
//        if (min < 0)
//        {
//          LOG(WARNING) << "Warning: non-monotonic timestamp detected, resetting...";
//          initialise_image_states(log_intensity_state_.rows,
//                                  log_intensity_state_.cols);
//          return;
//        }
//
//        update_bias_state_global(delta_t_array);
//        publish_bias_state(msg->events[i].ts);
//
//        update_log_intensity_state_global(delta_t_array);
//        publish_intensity_estimate(msg->events[i].ts);
//        t_next_publish_ = ts + 1 / publish_framerate_;
//        ts_array_.setTo(ts);
//      }
    }
  }
}

void High_pass_filter::initialise_image_states(const uint32_t& rows, const uint32_t& columns)
{
  log_image_state_ = cv::Mat::zeros(rows, columns, CV_64FC1);
  second_order_state_ = cv::Mat::zeros(rows, columns, CV_64FC1);
  bias_state_ = cv::Mat::zeros(rows, columns, CV_64FC1);
  leaky_event_count_on_ = cv::Mat::zeros(rows, columns, CV_64FC1);
  leaky_event_count_off_ = cv::Mat::zeros(rows, columns, CV_64FC1);
  ts_array_ = cv::Mat::zeros(rows, columns, CV_64FC1);
  ts_array_on_ = cv::Mat::zeros(rows, columns, CV_64FC1);
  ts_array_off_ = cv::Mat::zeros(rows, columns, CV_64FC1);

  t_next_publish_ = 0.0;
  t_next_recalibrate_contrast_thresholds_ = 0.0;
  t_next_log_intensity_update_ = 0.0;

  initialised_ = true;

  VLOG(2) << "Initialised!";
}

void High_pass_filter::update_state_local_cedric(const double& delta_t,
                                                 const int& x,
                                                 const int& y,
                                                 const bool& polarity)
{
  // compute exp(D*delta_t)
  double exp_D[2];  // diagonal 2x2 matrix
  double exp_A[4];  // column-major

  exp_D[0] = exp(D_[0]*delta_t);
  exp_D[1] = exp(D_[1]*delta_t);

  undiagonalise(exp_D, exp_A);  // undiagonalise based on U_ and U_inv, i.e. compute A = UDU'.

//  matmul2by2(stage1, U_inv_, exp_A);

  double log_i_state = exp_A[0] * second_order_state_.at<double>(y, x)
                       + exp_A[2] * bias_state_.at<double>(y, x);

  bias_state_.at<double>(y, x) = exp_A[1] * second_order_state_.at<double>(y, x)
                                 + exp_A[3] * bias_state_.at<double>(y, x);

  const double contrast_threshold = (polarity) ?
      contrast_threshold_on_user_defined_ : contrast_threshold_off_user_defined_;

//  second_order_state_.at<double>(y, x) = log_i_state + contrast_threshold;

  second_order_state_.at<double>(y, x) = exp(-cutoff_frequency_per_event_component_)
                                         * log_i_state + contrast_threshold;
}

void High_pass_filter::undiagonalise(double D[], double result[])
{
  // compute UDU' in two stages.
  // stage 1 UD
  // column-major
  double UD[4];
  UD[0] = U_[0]*D[0];
  UD[1] = U_[1]*D[0];
  UD[2] = U_[2]*D[1];
  UD[3] = U_[3]*D[1];
  // stage 2 (UD)U'
  result[0] = UD[0]*U_inv_[0] + UD[2]*U_inv_[1];
  result[1] = UD[1]*U_inv_[0] + UD[3]*U_inv_[1];
  result[2] = UD[0]*U_inv_[2] + UD[2]*U_inv_[3];
  result[3] = UD[1]*U_inv_[2] + UD[3]*U_inv_[3];
}


void High_pass_filter::matmul2by2(double a[], double b[], double result[])
{

}

void High_pass_filter::update_state_global_cedric(cv::Mat& delta_t_array)
{
  cv::Mat exp_D[2];  // diagonal 2x2 matrix at every cv::Mat pixel.
  cv::Mat exp_A[4];  // column-major
  cv::exp(delta_t_array*D_[0], exp_D[0]);
  cv::exp(delta_t_array*D_[1], exp_D[1]);

  undiagonalise_mat(exp_D, exp_A);  // undiagonalise based on U_ and U_inv, i.e. compute A = UDU'.

  cv::Mat bias_state = exp_A[1].mul(second_order_state_)
                       + exp_A[3].mul(bias_state_);

  second_order_state_ = exp_A[0].mul(second_order_state_)
                         + exp_A[2].mul(bias_state_);

  bias_state_ = bias_state;  // could just update bias state first and use updated version since it changes slow
}

void High_pass_filter::undiagonalise_mat(cv::Mat D[], cv::Mat result[])
{
  // compute UDU' in two stages.
  // stage 1 UD
  // column-major
  cv::Mat UD[4];
  UD[0] = U_[0]*D[0];  // scalar * mat
  UD[1] = U_[1]*D[0];
  UD[2] = U_[2]*D[1];
  UD[3] = U_[3]*D[1];
  // stage 2 (UD)U'
  result[0] = UD[0]*U_inv_[0] + UD[2]*U_inv_[1];  // mat * scalar + mat * scalar
  result[1] = UD[1]*U_inv_[0] + UD[3]*U_inv_[1];
  result[2] = UD[0]*U_inv_[2] + UD[2]*U_inv_[3];
  result[3] = UD[1]*U_inv_[2] + UD[3]*U_inv_[3];
}

//void High_pass_filter::matmul2by2_array(cv::Mat a[], double b[], cv::Mat result[])
//{
//  // column-major
//  result[0] = a[0]*b[0] + a[2]*b[1];
//  result[1] = a[1]*b[0] + a[3]*b[1];
//  result[2] = a[0]*b[2] + a[2]*b[3];
//  result[3] = a[1]*b[2] + a[3]*b[3];
//}

void High_pass_filter::update_log_intensity_state(const double& delta_t,
                                                  const int& x,
                                                  const int& y,
                                                  const bool& polarity)
{
  double contrast_threshold;
  if (adaptive_contrast_threshold_)
  {
    contrast_threshold = (polarity) ?
        contrast_threshold_on_adaptive_ : contrast_threshold_off_adaptive_;
  }
  else
  {
    contrast_threshold = (polarity) ?
        contrast_threshold_on_user_defined_ : contrast_threshold_off_user_defined_;
  }

//  const double decay_factor = std::exp(
//      - cutoff_frequency_global_ * delta_t
//      - cutoff_frequency_per_event_component_);
//  second_order_state_.at<double>(y, x) = decay_factor
//                                          * second_order_state_.at<double>(y, x)
//                                          + (1 - decay_factor)
//                                          * (- bias_state_.at<double>(y, x)
//                                             / cutoff_frequency_global_)
//                                          + contrast_threshold;
  const double decay_factor = std::exp(
        - cutoff_frequency_global_ * delta_t
        - cutoff_frequency_per_event_component_);
    log_image_state_.at<double>(y, x) = decay_factor
                                        * log_image_state_.at<double>(y, x)
                                        + contrast_threshold;
}

void High_pass_filter::update_log_intensity_state_global(cv::Mat& delta_t_array)
{
  cv::Mat beta;
  cv::exp(-cutoff_frequency_global_ * delta_t_array, beta);
  log_image_state_ = log_image_state_.mul(beta);
}

void High_pass_filter::update_bias_state(const double& delta_t,
                                         const int& x,
                                         const int& y)
{
    bias_state_.at<double>(y, x) = bias_state_.at<double>(y, x)
                                   + cutoff_frequency_bias_
                                   * second_order_state_.at<double>(y, x)
                                   * delta_t;

    //  ts_array_.at<double>(y, x) = ts; // FIX THIS
}

void High_pass_filter::update_bias_state_global(cv::Mat& delta_t_array)
{
  bias_state_ = bias_state_
                + cutoff_frequency_bias_
                * second_order_state_.mul(delta_t_array);

//  ts_array_.setTo(ts); // FIX THIS
}

void High_pass_filter::update_leaky_event_count(const double& ts,
                                                const int& x,
                                                const int& y,
                                                const bool& polarity)
{
  if (polarity)
  {
    // positive ON event
    const double delta_t = (ts - ts_array_on_.at<double>(y, x));
    if (delta_t >= 0)
    {
      leaky_event_count_on_.at<double>(y, x) = std::exp(
          - event_count_cutoff_frequency_ * delta_t)
          * leaky_event_count_on_.at<double>(y, x) + 1;
      ts_array_on_.at<double>(y, x) = ts;
    }
  }
  else
  {
    // negative OFF event
    const double delta_t = (ts - ts_array_off_.at<double>(y, x));
    if (delta_t >= 0)
    {
      leaky_event_count_off_.at<double>(y, x) = std::exp(-event_count_cutoff_frequency_ * delta_t)
          * leaky_event_count_off_.at<double>(y, x) + 1;
      ts_array_off_.at<double>(y, x) = ts;
    }
  }
}

void High_pass_filter::recalibrate_contrast_thresholds(const double& ts)
{
  constexpr double EVENT_DENSITY_MIN = 5e6;
  //first do global update
  cv::Mat decay_factor_on;
  cv::Mat decay_factor_off;
  cv::exp(-event_count_cutoff_frequency_ * (ts - ts_array_on_), decay_factor_on);
  cv::exp(-event_count_cutoff_frequency_ * (ts - ts_array_off_), decay_factor_off);

  leaky_event_count_on_ = leaky_event_count_on_.mul(decay_factor_on);
  leaky_event_count_off_ = leaky_event_count_off_.mul(decay_factor_off);

  ts_array_on_.setTo(ts);
  ts_array_off_.setTo(ts);

  const double sum_on = cv::sum(leaky_event_count_on_)[0];
  const double sum_off = cv::sum(leaky_event_count_off_)[0];
  if (sum_on + sum_off > EVENT_DENSITY_MIN)
  {
  contrast_threshold_off_adaptive_ = -sum_on / (sum_off + 1e-10)
                                     * contrast_threshold_on_adaptive_; // re-calibrate contrast thresholds
  }
  VLOG_IF_EVERY_N(3, adaptive_contrast_threshold_, 10) << "contrast threshold [ON, OFF] = ["
      << contrast_threshold_off_adaptive_*10 << ",\t1]*" <<  contrast_threshold_on_adaptive_
      << "\nEvent density: " << sum_on + sum_off;

}

void High_pass_filter::publish_log_image(cv::Mat& image,
                                         const ros::Time& timestamp,
                                         const image_transport::Publisher& publisher,
                                         std::string save_dir="")
{
  cv::Mat display_image = process_log_image(image, timestamp.toSec());
  cv_bridge::CvImage cv_image;

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

  if (spatial_filter_sigma_ > 0)
  {
   cv::Mat filtered_display_image;
   if (spatial_smoothing_method_ == GAUSSIAN)
   {
     cv::GaussianBlur(display_image, filtered_display_image, cv::Size(5, 5), spatial_filter_sigma_, spatial_filter_sigma_);
   }
   else if (spatial_smoothing_method_ == BILATERAL)
   {
     const double bilateral_sigma = spatial_filter_sigma_*25;
     cv::bilateralFilter(display_image, filtered_display_image, 5, bilateral_sigma, bilateral_sigma);
   }
   display_image = filtered_display_image; // data is not copied
  }

  cv_image.image = display_image;
  cv_image.header.stamp = timestamp;
  publisher.publish(cv_image.toImageMsg());

  if (save_images_)
  {
   static int image_counter = 0;
   char image_counter_c_string[10];
   std::sprintf(image_counter_c_string, "%05d", image_counter);
   std::string image_counter_str(image_counter_c_string);
   std::string save_path = save_dir_ + save_dir + "/image" + image_counter_str + ".png";
   cv::imwrite(save_path, display_image);
   image_counter++;
  }
}

void High_pass_filter::publish_intensity_estimate(const ros::Time& timestamp)
{
  cv::Mat display_image;
  cv_bridge::CvImage cv_image;

  convert_log_intensity_state_to_display_image(display_image, timestamp.toSec());

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

  if (spatial_filter_sigma_ > 0)
  {
    cv::Mat filtered_display_image;
    if (spatial_smoothing_method_ == GAUSSIAN)
    {
      cv::GaussianBlur(display_image, filtered_display_image, cv::Size(5, 5), spatial_filter_sigma_, spatial_filter_sigma_);
    }
    else if (spatial_smoothing_method_ == BILATERAL)
    {
      const double bilateral_sigma = spatial_filter_sigma_*25;
      cv::bilateralFilter(display_image, filtered_display_image, 5, bilateral_sigma, bilateral_sigma);
    }
    display_image = filtered_display_image; // data is not copied
  }

  cv_image.image = display_image;
  cv_image.header.stamp = timestamp;
  image_state_pub_.publish(cv_image.toImageMsg());

  if (save_images_)
  {
    static int image_counter = 0;
    std::string save_path = save_dir_ + "image" + std::to_string(image_counter) + ".png";
    cv::imwrite(save_path, display_image);
    image_counter++;
  }
}

void High_pass_filter::publish_raw_image(cv::Mat& image,
                                         const ros::Time& timestamp,
                                         const image_transport::Publisher& publisher,
                                         std::string save_dir="")
{
  cv::Mat display_image;
  cv_bridge::CvImage cv_image;

//  cv::normalize(image, display_image, 0, 255, cv::NORM_MINMAX, CV_8UC1);

  image.convertTo(display_image, CV_8UC1, 150, 75);

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

  cv_image.image = display_image;
  cv_image.header.stamp = timestamp;
  publisher.publish(cv_image.toImageMsg());

  if (save_images_)
  {
   static int image_counter = 0;
   char image_counter_c_string[10];
  std::sprintf(image_counter_c_string, "%05d", image_counter);
  std::string image_counter_str(image_counter_c_string);
   std::string save_path = save_dir_ + save_dir + "/image" + image_counter_str + ".png";
   cv::imwrite(save_path, display_image);
   image_counter++;
  }
}

cv::Mat High_pass_filter::process_log_image(cv::Mat& log_image, const double& ts)
{
  constexpr double LOG_INTENSITY_OFFSET = std::log(1.5);  // chosen because standard APS frames range from [1, 2].
  constexpr double FADE_DURATION = 2; // seconds. Time taken for dynamic range bounds to "take effect".
  // used for low-pass parameter to reach 95% of constant signal in FADE_DURATION seconds.
  constexpr double ALPHA = -std::log(1 - 0.95)/FADE_DURATION;  // rad/s.
  constexpr double EXPECTED_MEAN = 0.5;

  static double t_last = 0.0;
  static double intensity_lower_bound = intensity_min_user_defined_;
  static double intensity_upper_bound = intensity_max_user_defined_;

  const double delta_t = ts - t_last;
  const double beta = std::exp(-delta_t*ALPHA);  // low-pass parameter

  cv::Mat image;
  cv::Mat image_out;

  log_image.copyTo(image);  // ~[-0.5, 0.5]
  image += LOG_INTENSITY_OFFSET;  // [-0.1, 0.9]
  cv::exp(image, image);  // ~[1, 2]
  image -= 1;  // [0, 1]

  if (delta_t >= 0)
  {
    if (adaptive_dynamic_range_)
    {
      constexpr int DO_EVERY_N_TIMES = 10;
      static int count = DO_EVERY_N_TIMES;
      static double robust_min, robust_max;
      if (count++ >= DO_EVERY_N_TIMES)
      {
        minMaxLocRobust(image, robust_min, robust_max);
        count = 0;
  //      cv::minMaxLoc(image, &robust_min, &robust_max);  // test speed
      }
      constexpr double MAX_INTENSITY_LOWER_BOUND = EXPECTED_MEAN - 0.2;
      constexpr double MIN_INTENSITY_UPPER_BOUND = EXPECTED_MEAN + 0.2;
      constexpr double EXTEND_RANGE = 0.05;  // extend dynamic range for visual appeal.
      intensity_lower_bound = std::min(beta*intensity_lower_bound + (1 - beta)
          * (robust_min - EXTEND_RANGE), MAX_INTENSITY_LOWER_BOUND);

      intensity_upper_bound = std::max(beta*intensity_upper_bound + (1 - beta)
          * (robust_max + EXTEND_RANGE), MIN_INTENSITY_UPPER_BOUND);
    }
    else
    {
      intensity_lower_bound = intensity_min_user_defined_;
      intensity_upper_bound = intensity_max_user_defined_;
    }
  }
  const double intensity_range = intensity_upper_bound - intensity_lower_bound;
  image -= intensity_lower_bound;
  image.convertTo(image_out, CV_8UC1, 255.0/intensity_range);
  t_last = ts;
  return image_out;
}

void High_pass_filter::convert_log_intensity_state_to_display_image(cv::Mat& image_out, const double& ts)
{
  constexpr double LOG_INTENSITY_OFFSET = std::log(1.5);  // chosen because standard APS frames range from [1, 2].
  constexpr double FADE_DURATION = 2; // seconds. Time taken for dynamic range bounds to "take effect".
  // used for low-pass parameter to reach 95% of constant signal in FADE_DURATION seconds.
  constexpr double ALPHA = -std::log(1 - 0.95)/FADE_DURATION;  // rad/s.
  constexpr double EXPECTED_MEAN = 0.5;

  static double t_last = 0.0;
  static double intensity_lower_bound = intensity_min_user_defined_;
  static double intensity_upper_bound = intensity_max_user_defined_;

  const double delta_t = ts - t_last;
  const double beta = std::exp(-delta_t*ALPHA);  // low-pass parameter

  cv::Mat image;

  second_order_state_.copyTo(image);  // ~[-0.5, 0.5]
  image += LOG_INTENSITY_OFFSET;  // [-0.1, 0.9]
  cv::exp(image, image);  // ~[1, 2]
  image -= 1;  // [0, 1]

  if (delta_t >= 0)
  {
    if (adaptive_dynamic_range_)
    {
      constexpr int DO_EVERY_N_TIMES = 10;
      static int count = DO_EVERY_N_TIMES;
      static double robust_min, robust_max;
      if (count++ >= DO_EVERY_N_TIMES)
      {
        minMaxLocRobust(image, robust_min, robust_max);
        count = 0;
  //      cv::minMaxLoc(image, &robust_min, &robust_max);  // test speed
      }
      constexpr double MAX_INTENSITY_LOWER_BOUND = EXPECTED_MEAN - 0.2;
      constexpr double MIN_INTENSITY_UPPER_BOUND = EXPECTED_MEAN + 0.2;
      constexpr double EXTEND_RANGE = 0.05;  // extend dynamic range for visual appeal.
      intensity_lower_bound = std::min(beta*intensity_lower_bound + (1 - beta)
          * (robust_min - EXTEND_RANGE), MAX_INTENSITY_LOWER_BOUND);

      intensity_upper_bound = std::max(beta*intensity_upper_bound + (1 - beta)
          * (robust_max + EXTEND_RANGE), MIN_INTENSITY_UPPER_BOUND);
    }
    else
    {
      intensity_lower_bound = intensity_min_user_defined_;
      intensity_upper_bound = intensity_max_user_defined_;
    }
  }
  const double intensity_range = intensity_upper_bound - intensity_lower_bound;
  image -= intensity_lower_bound;
  image.convertTo(image_out, CV_8UC1, 255.0/intensity_range);
  t_last = ts;
}

void High_pass_filter::minMaxLocRobust(const cv::Mat& image,
                                       double& robust_min,
                                       double& robust_max)
{
  constexpr double PERCENTAGE_PIXELS_TO_DISCARD = 0.5;
  cv::Mat image_as_row;
  cv::Mat image_as_row_sorted;
  const int single_row_idx_min = (0.5*PERCENTAGE_PIXELS_TO_DISCARD/100)*image.total();
  const int single_row_idx_max = (1 - 0.5*PERCENTAGE_PIXELS_TO_DISCARD/100)*image.total();
  image_as_row = image.reshape(0, 1);
  cv::sort(image_as_row, image_as_row_sorted, CV_SORT_EVERY_ROW + CV_SORT_ASCENDING);
  image_as_row_sorted.convertTo(image_as_row_sorted, CV_64FC1);
  robust_min = image_as_row_sorted.at<double>(single_row_idx_min);
  robust_max = image_as_row_sorted.at<double>(single_row_idx_max);
}

void High_pass_filter::reconfigureCallback(pure_event_reconstruction::pure_event_reconstructionConfig &config, uint32_t level)
{
  static double a = 0;
  static double b = 0;
  static double c = 0;
  cutoff_frequency_global_ = config.Cutoff_frequency*2*M_PI;
  cutoff_frequency_per_event_component_ = config.Cutoff_frequency_per_event_component;
  cutoff_frequency_bias_ = config.Cutoff_frequency_bias*2*M_PI;
  contrast_threshold_on_user_defined_ = config.Contrast_threshold_ON;
  contrast_threshold_off_user_defined_ = config.Contrast_threshold_OFF;
  intensity_min_user_defined_ = config.Intensity_min;
  intensity_max_user_defined_ = config.Intensity_max;
  adaptive_contrast_threshold_ = config.Auto_detect_contrast_thresholds;
  spatial_filter_sigma_ = config.Spatial_filter_sigma;
  spatial_smoothing_method_ = int(config.Bilateral_filter);
  adaptive_dynamic_range_ = config.Auto_adjust_dynamic_range;
  color_image_ = config.Color_display;
  compute_second_order_ = config.Second_order;
  bias_gain_ = config.Bias_gain;
  bool reset = config.Reset;
  if (reset)
  {
    initialise_image_states(second_order_state_.rows, second_order_state_.cols);
  }

  if ( (a != cutoff_frequency_global_) || (b != cutoff_frequency_bias_) || (c != bias_gain_) )
  {
    a = cutoff_frequency_global_;
    b = cutoff_frequency_bias_;
    c = bias_gain_;
    // check that eigenvalues of A are real
    if (4*b*c > a*a)
    {
      VLOG(1) << "Eigenvalues of A not real. Please change gains (cutoff_frequency(s) for intensity and bias).";
      return;
    }
    // (p)re-compute eigenvectors/values of A (A = UDU').
    const double root = sqrt(a*a - 4*b*c);
    D_[0] = (-a - root)/2;  // diagonal matrix, D_11 = D_[0], D_22 = D_[1]
    D_[1] = (-a + root)/2;

    U_[0] = D_[0]/b;  // column-major
    U_[2] = D_[1]/b;

    U_inv_[0] = -b/root;  // column-major
    U_inv_[1] = b/root;
    U_inv_[2] = (1 - a/root)/2;
    U_inv_[3] = (1 + a/root)/2;

    VLOG(3) << "Re-computed eigenvalue decomposition.";
  }
}


} // namespace
