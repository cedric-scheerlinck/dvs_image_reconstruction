#pragma once

#include <functional>
#include <string>
#include <unordered_map>

#include <rosbag/bag.h>

namespace rpg_common_ros {

class BagPlayer
{
public:
  explicit BagPlayer(const std::string& file_name);

  void attachCallbackToTopic(
      const std::string& topic,
      const std::function<void(
          const rosbag::MessageInstance&)>& callback);

  void play();

private:
  rosbag::Bag bag_;

  typedef std::unordered_map<std::string,
      std::function<void(const rosbag::MessageInstance&)>> CallBackMap;
  CallBackMap subscriptions_;
};

}  // namespace rpg_common_ros
namespace rpg_ros = rpg_common_ros;
