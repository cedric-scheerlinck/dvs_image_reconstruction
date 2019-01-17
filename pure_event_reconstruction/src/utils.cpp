#include "pure_event_reconstruction/utils.h"
#include <string>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <glog/logging.h>

namespace pure_event_reconstruction {
namespace utils {

std::string find_event_topic(const std::string bag_path)
{
  // determine topics that contain messages of type "dvs_msgs/EventArray"

  std::string event_topic_name="";
  rosbag::Bag input_bag;
  try
  {
    input_bag.open(bag_path, rosbag::bagmode::Read);
  } catch(rosbag::BagIOException e)
  {
    std::cerr << "Error: could not open rosbag: " << bag_path << std::endl;
    return "";
  }
  rosbag::View view(input_bag);

  for(const rosbag::MessageInstance& m : view)
  {
    if(m.getDataType() == "dvs_msgs/EventArray")
    {
      VLOG(1) << "Detected a topic with messages of type: dvs_msgs/EventArray";
      event_topic_name = m.getTopic();
      break;
    }
  }
  input_bag.close();
  return event_topic_name;
}

} // namespace utils
} // namespace pure_event_reconstruction
