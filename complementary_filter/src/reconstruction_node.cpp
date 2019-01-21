#include "complementary_filter/complementary_filter.h"

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
  std::string working_dir;
  nh_private.getParam("bag_path", bag_path);
  nh_private.getParam("working_dir", working_dir);
//
//  bool realtime = bag_path.empty();
//
//  if (realtime)
//  {
//    VLOG(1) << "Running in real-time mode";
//
//  ros::spin();
//  } else if (!realtime)
//  {
//    bag_path = pure_event_reconstruction::utils::fullpath(wd, bag_path);
//
//    VLOG(1) << "Path to rosbag: " << bag_path;
//
//    std::string event_topic_name = pure_event_reconstruction::utils::find_event_topic(bag_path);
//
//    VLOG(1) << "Reading events from: " << event_topic_name;
//
//    // attach relevant callbacks to topics
//    rpg_common_ros::BagPlayer player(bag_path);
//    player.attachCallbackToTopic(event_topic_name,
//        [&](const rosbag::MessageInstance& msg)
//        {
//          dvs_msgs::EventArray::ConstPtr events = msg.instantiate<dvs_msgs::EventArray>();
//          CHECK(events);
//          high_pass_filter.eventsCallback(events);
//        }
//    );
//
//    player.play();
//    VLOG(1) << "...done!";
//
//  }

//  ros::shutdown();
//  return 0;
}
