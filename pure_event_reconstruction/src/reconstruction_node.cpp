#include <dvs_msgs/EventArray.h>
#include <glog/logging.h>
#include <ros/ros.h>
#include <rosbag/message_instance.h>
#include <string>

#include "pure_event_reconstruction/bag_player.h"
#include "pure_event_reconstruction/pure_event_reconstruction.h"
#include "pure_event_reconstruction/utils.h"

int main(int argc, char* argv[])
{
  // Initialize Google's flags and logging libraries.
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  ros::init(argc, argv, "reconstruction_node");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  pure_event_reconstruction::High_pass_filter high_pass_filter(nh, nh_private);

  std::string bag_path;
  nh_private.getParam("bag_path", bag_path);

  bool realtime = bag_path.empty(); // used to determine whether to use realtime or offline mode

  if (realtime)
  {
    VLOG(1) << "Running in real-time mode";
    // subscriber queue size
    constexpr int EVENT_SUB_QUEUE_SIZE = 1000;
    ros::Subscriber event_sub = nh.subscribe(
        "events", EVENT_SUB_QUEUE_SIZE, &pure_event_reconstruction::High_pass_filter::eventsCallback,
        &high_pass_filter);

    ros::spin();
  }
  else if (!realtime)
  {
    std::string working_dir;
    nh_private.getParam("working_dir", working_dir);

    bag_path = pure_event_reconstruction::utils::fullpath(working_dir, bag_path);

    VLOG(1) << "Path to rosbag: " << bag_path;

    std::string event_topic_name = pure_event_reconstruction::utils::find_event_topic(bag_path);

    VLOG(1) << "Reading events from: " << event_topic_name;

    // attach relevant callbacks to topics
    rpg_common_ros::BagPlayer player(bag_path);
    player.attachCallbackToTopic(event_topic_name,
        [&](const rosbag::MessageInstance& msg)
        {
          dvs_msgs::EventArray::ConstPtr events = msg.instantiate<dvs_msgs::EventArray>();
          CHECK(events);
          high_pass_filter.eventsCallback(events);
        }
    );

    player.play();
    VLOG(1) << "...done!";

  }

  ros::shutdown();
  return 0;
}
