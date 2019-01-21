#include "complementary_filter/utils.h"

#include <rosbag/bag.h>
#include <rosbag/view.h>

namespace complementary_filter {
namespace utils {

std::string find_topic_by_type(const std::string bag_path, const std::string message_type)
{
  // returns the topic of the first message of type message_type in the bag specified in bag_path.

  std::string topic_name="";
  rosbag::Bag input_bag;
  try
  {
    input_bag.open(bag_path, rosbag::bagmode::Read);
  } catch(rosbag::BagIOException& e)
  {
    std::cerr << "Error: could not open rosbag: " << bag_path << std::endl;
    return "";
  }
  rosbag::View view(input_bag);

  for(const rosbag::MessageInstance& m : view)
  {
    if(m.getDataType() == message_type)
    {
      topic_name = m.getTopic();
      break;
    }
  }
  input_bag.close();
  return topic_name;
}


std::string create_fullpath(const std::string wd, const std::string path)
{
  // checks if path seems like a full path, if not, prepend wd in a smart way

  switch(path.front())
  {
    case '/' : return path;
    case '~' :
    {
      const int total_slashes = std::count(wd.begin(), wd.end(), '/');
      if (total_slashes < 2)
      {

        std::cerr << "Cannot resolve home directory (~) from current working directory.\n"
            "Please specify relative or full path." << std::endl;
        // throw exception
        return "";
      }
      else if (total_slashes == 2)
      {
        return wd + path.substr(1);
      }
      else
      {
        int slashes = 0;
        int pos = 0;
        while (slashes != 3)
        {
          if (wd.at(pos) == '/')
          {
            slashes++;
          }
          pos++; // pos will be one larger but that's okay because substr takes in len.
        }
        const int len = pos - 1; // take the slash from path.
        return wd.substr(0, len) + path.substr(1);
      }
    }
    case '$' :
    {
      std::cerr << "Error: cannot resolve environment variables starting with $.\n"
          "Please use relative or full path." << std::endl;
      return ""; // throw exception
    }
  }

  return wd + "/" + path;

}


} // namespace utils
} // namespace pure_event_reconstruction
