#include <dvs_msgs/EventArray.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/message_instance.h>
#include <rosbag/view.h>
//#include <string>
#include <ctime>


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

    std::clock_t start;
    double duration;
    start = std::clock();

    player.play();

    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;

    player.closeBag();

    // compute stats
    rosbag::Bag bag;
    bag.open(bag_path, rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back(event_topic_name);
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    int num_events = 0;
    bool first_msg = true;
    double start_time = 0;
    double end_time = 0;
    for (const rosbag::MessageInstance& msg : view)
    {
      dvs_msgs::EventArray::ConstPtr events = msg.instantiate<dvs_msgs::EventArray>();
      CHECK(events);
      num_events += events->events.size();
      if (first_msg)
      {
        start_time = events->events.front().ts.toSec();
        first_msg = false;
      }
      end_time = std::max(events->events.back().ts.toSec(), end_time);
    }

    VLOG(1) << "Processing time (s): " << duration;
    VLOG(1) << "Real-time factor: " << (end_time - start_time)/duration;
    VLOG(1) << "Events/second: " << num_events/duration/1e6 << "M";
    VLOG(1) << "...done!";
  }

  ros::shutdown();
  return 0;
}
