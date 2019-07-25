#include <dvs_msgs/EventArray.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>
#include <rosbag/message_instance.h>
#include <sensor_msgs/Image.h>
#include <string>

#include "complementary_filter/bag_player.h"
#include "complementary_filter/complementary_filter.h"
#include "complementary_filter/utils.h"

int main(int argc, char* argv[])
{
  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  ros::init(argc, argv, "reconstruction_node");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  complementary_filter::Complementary_filter complementary_filter(nh, nh_private);

  std::string bag_path;
  nh_private.getParam("bag_path", bag_path);

  bool realtime = bag_path.empty(); // used to determine whether to use realtime or offline mode

  if (realtime)
  {
    VLOG(1) << "Running in real-time mode";

    // subscriber queue size
     constexpr int EVENT_SUB_QUEUE_SIZE = 1000;
     constexpr int IMAGE_SUB_QUEUE_SIZE = 100;

    ros::Subscriber event_sub = nh.subscribe(
        "events", EVENT_SUB_QUEUE_SIZE, &complementary_filter::Complementary_filter::eventsCallback,
        &complementary_filter);
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber image_raw_sub = it.subscribe(
        "image_raw", IMAGE_SUB_QUEUE_SIZE, &complementary_filter::Complementary_filter::imageCallback,
        &complementary_filter);

    ros::spin();
  }
  else if (!realtime)
  {
    std::string working_dir;
    nh_private.getParam("working_dir", working_dir);

    bag_path = complementary_filter::utils::create_fullpath(working_dir, bag_path);

    VLOG(1) << "Path to rosbag: " << bag_path;

    // if you want to specify the exact topic you want to read from, replace these lines
    std::string event_topic_name = complementary_filter::utils::find_topic_by_type(bag_path, "dvs_msgs/EventArray");
    std::string image_topic_name = complementary_filter::utils::find_topic_by_type(bag_path, "sensor_msgs/Image");

    VLOG(1) << "Reading events from: " << event_topic_name;
    VLOG(1) << "Reading images from: " << image_topic_name;


    // put all images and timestamps into vectors
    VLOG(1) << "Loading images...";

    rpg_common_ros::BagPlayer image_player(bag_path);
    image_player.attachCallbackToTopic(image_topic_name,
        [&](const rosbag::MessageInstance& msg)
        {
          sensor_msgs::Image::ConstPtr image = msg.instantiate<sensor_msgs::Image>();
          CHECK(image);
          complementary_filter.load_images(image);
        }
    );
    // go through the whole bag once just to load images into vectors
    image_player.play();
    VLOG(1) << "...done!";

    // attach relevant callbacks to topics
    rpg_common_ros::BagPlayer player(bag_path);
    player.attachCallbackToTopic(event_topic_name,
        [&](const rosbag::MessageInstance& msg)
        {
          dvs_msgs::EventArray::ConstPtr events = msg.instantiate<dvs_msgs::EventArray>();
          CHECK(events);
          complementary_filter.offlineEventsCallback(events);
        }
    );

    VLOG(1) << "Playing bag...";
    // all the work is done here
    std::clock_t start;
    double duration;
    start = std::clock();

    player.play();
    
    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;

    VLOG(1) << "...done!";
    VLOG(1) << "Duration (s): " << duration;

  }

  ros::shutdown();
  return 0;
}
