#include "pure_event_reconstruction/bag_player.h"

#include <iostream>

#include <glog/logging.h>
#include <rosbag/view.h>

namespace rpg_common_ros {

BagPlayer::BagPlayer(const std::string& file_name)
{
  bag_.open(file_name, rosbag::bagmode::Read);
}

void BagPlayer::attachCallbackToTopic(
    const std::string& topic,
    const std::function<void(const rosbag::MessageInstance&)>& callback)
{
  CHECK(subscriptions_.emplace(topic, callback).second);
}

void BagPlayer::play()
{
  std::vector<std::string> topics;
  for (const CallBackMap::value_type& topic_data : subscriptions_)
  {
    topics.emplace_back(topic_data.first);
    std::cerr << topic_data.first << std::endl;
  }

  rosbag::View view(bag_, rosbag::TopicQuery(topics));
  for (const rosbag::MessageInstance& message : view)
  {
    const std::string& topic = message.getTopic();
    subscriptions_[topic](message);
  }
}

}  // namespace rpg_common_ros
