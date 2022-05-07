cmake_minimum_required(VERSION 3.8)
project(vdb_mapping_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vdb_mapping REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)


set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  tf2
  tf2_ros
  vdb_mapping
  std_srvs
)

add_executable(vdb_mapping_ros_node src/vdb_mapping_ros_node.cpp)

target_link_libraries(vdb_mapping_ros_node vdb_mapping::vdb_mapping)
ament_target_dependencies(vdb_mapping_ros_node ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(TARGETS
  vdb_mapping_ros_node
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
