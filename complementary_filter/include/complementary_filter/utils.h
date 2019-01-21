#pragma once

#include <string>

namespace complementary_filter {
namespace utils {

std::string find_topic_by_type(const std::string bag_path, const std::string message_type);
std::string create_fullpath(const std::string wd, const std::string path);
void repackage_bag(const std::string bag_path);

} // namespace utils
} // namespace pure_event_reconstruction
