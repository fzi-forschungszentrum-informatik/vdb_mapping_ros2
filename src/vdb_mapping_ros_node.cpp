#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "vdb_mapping/OccupancyVDBMapping.h"
#include "vdb_mapping_ros2/VDBMappingROS2.h"


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  std::cout << "well hello there" << std::endl;
  VDBMappingROS2<vdb_mapping::OccupancyVDBMapping> vdb_mapping;
  rclcpp::spin(std::make_shared<VDBMappingROS2<vdb_mapping::OccupancyVDBMapping>>());
  std::cout << "dafuq1" << std::endl;
  //vdb_mapping.resetMap();
  std::cout << "dafuq2" << std::endl;

  rclcpp::shutdown();
  return 0;
}
