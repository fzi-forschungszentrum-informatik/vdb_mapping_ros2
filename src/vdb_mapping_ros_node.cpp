#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "vdb_mapping/OccupancyVDBMapping.h"
//#include "vdb_mapping_ros2/VDBMappingROS2.h"


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  //rclcpp::spin(std::make_shared<)
  std::cout << "well hello there" << std::endl;
  rclcpp::shutdown();
  return 0;
}
