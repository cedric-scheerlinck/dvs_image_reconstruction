#include "../include/complementary_filter/complementary_filter.h"

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

  ros::spin();

  return 0;
}
