#include <dvs_msgs/EventArray.h>
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
    ros::spin();
  }
  else if (!realtime)
  {
    std::string working_dir;
    nh_private.getParam("working_dir", working_dir);

    bag_path = complementary_filter::utils::create_fullpath(working_dir, bag_path);

    VLOG(1) << "Path to rosbag: " << bag_path;

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
    player.play();
    VLOG(1) << "...done!";

  }

  ros::shutdown();
  return 0;
}
