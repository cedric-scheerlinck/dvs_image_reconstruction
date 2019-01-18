#pragma once

#include <string>

namespace pure_event_reconstruction {
namespace utils {

std::string find_event_topic(const std::string bag_path);
std::string fullpath(const std::string wd, const std::string path);

} // namespace utils
} // namespace pure_event_reconstruction
