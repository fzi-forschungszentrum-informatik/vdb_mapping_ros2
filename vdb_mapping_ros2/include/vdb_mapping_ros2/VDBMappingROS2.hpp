// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2022 FZI Forschungszentrum Informatik
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// -- END LICENSE BLOCK ------------------------------------------------
//----------------------------------------------------------------------
/*!\file
 *
 * \author  Marvin Große Besselmann grosse@fzi.de
 * \date    2022-05-07
 *
 */
//----------------------------------------------------------------------
#ifndef VDB_MAPPING_ROS2_VDBMAPPINGROS2_HPP_INCLUDED
#define VDB_MAPPING_ROS2_VDBMAPPINGROS2_HPP_INCLUDED

#include <rclcpp/rclcpp.hpp>

#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <vdb_mapping_interfaces/srv/get_map_section.hpp>
#include <vdb_mapping_interfaces/srv/load_map.hpp>
#include <vdb_mapping_interfaces/srv/trigger_map_section_update.hpp>
#include <visualization_msgs/msg/marker.hpp>

#define BOOST_BIND_NO_PLACEHOLDERS
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <vdb_mapping_ros2/VDBMappingTools.hpp>

struct RemoteSource
{
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_update_sub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_overwrite_sub;
  rclcpp::Client<vdb_mapping_interfaces::srv::GetMapSection>::SharedPtr get_map_section_client;
  bool apply_remote_updates;
  bool apply_remote_overwrites;
};

template <typename VDBMappingT>
class VDBMappingROS2 : public rclcpp::Node
{
public:
  /*!
   * \brief Creates a new VDBMappingROS instance
   */
  VDBMappingROS2();
  virtual ~VDBMappingROS2(){};

  /*!
   * \brief Resets the current map
   */
  void resetMap();
  /*!
   * \brief Saves the current map
   */
  bool saveMap(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
               const std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  /*!
   * \brief Load stored map
   */
  bool loadMap(const std::shared_ptr<vdb_mapping_interfaces::srv::LoadMap::Request> req,
               const std::shared_ptr<vdb_mapping_interfaces::srv::LoadMap::Response> res);
  /*!
   * \brief Sensor callback for scan aligned Pointclouds
   * In contrast to the normal sensor callback here an additional sensor frame has to be specified
   * as origin of the raycasting
   *
   * \param msg PointCloud message
   */
  void alignedCloudCallback(const std::shared_ptr<sensor_msgs::msg::PointCloud2> cloud_msg);
  /*!
   * \brief Sensor callback for raw pointclouds. All data will be transformed into the map frame.
   *
   * \param msg
   */
  void sensorCloudCallback(const std::shared_ptr<sensor_msgs::msg::PointCloud2> cloud_msg);
  /*!
   * \brief Integrating the transformed pointcloud and sensor origins into the core mapping library
   *
   *
   * \param cloud Point cloud transformed into map coordinates
   * \param tf Sensor transform in map coordinates
   */
  void insertPointCloud(const typename VDBMappingT::PointCloudT::Ptr cloud,
                        const geometry_msgs::msg::TransformStamped transform);
  /*!
   * \brief Publishes a marker array and pointcloud representation of the map
   */
  void publishMap() const;
  /*!
   * \brief Listens to map updates and creats a map from these
   *
   * \param update_msg Single map update from a remote mapping instance
   */
  void mapUpdateCallback(const std::shared_ptr<std_msgs::msg::String> update_msg);

  /*!
   * \brief Listens to map overwrites and creates a map from these
   *
   * \param update_msg Single map overwrite from a remote mapping instance
   */
  void mapOverwriteCallback(const std::shared_ptr<std_msgs::msg::String> update_msg);

  /*!
   * \brief Returns the map
   *
   * \returns VDB map
   */
  VDBMappingT& getMap();

  /*!
   * \brief Returns the map
   *
   * \returns VDB map
   */
  const VDBMappingT& getMap() const;

  /*!
   * \brief Callback for map reset service call
   *
   * \param res result of the map reset
   * \returns result of map reset
   */
  bool resetMapCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                        const std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  /*!
   * \brief Callback for requesting parts of the map
   *
   * \param req Coordinates and reference of the map section
   * \param res Result of section request, which includes the returned map
   *
   * \returns Result of section request
   */
  bool getMapSectionCallback(
    const std::shared_ptr<vdb_mapping_interfaces::srv::GetMapSection::Request> req,
    const std::shared_ptr<vdb_mapping_interfaces::srv::GetMapSection::Response> res);

  /*!
   * \brief Callback for triggering a map section request on a remote source
   *
   * \param req Coordinates, reference frame and remote source identifier of the map section
   * \param res Result of triggering section request
   *
   * \returns Result of triggering section request
   */
  bool triggerMapSectionUpdateCallback(
    const std::shared_ptr<vdb_mapping_interfaces::srv::TriggerMapSectionUpdate::Request> req,
    const std::shared_ptr<vdb_mapping_interfaces::srv::TriggerMapSectionUpdate::Response> res);

  /*!
   * \brief Creates a compressed Bitstream as ROS2 msg from an input grid
   *
   * \param update Update grid
   *
   * \returns bitstream
   */
  std_msgs::msg::String gridToMsg(const typename VDBMappingT::UpdateGridT::Ptr update) const;

  /*!
   * \brief Creates a compressed Bitstream as string from an input grid
   *
   * \param update Update grid
   *
   * \returns bitstream
   */
  std::string gridToStr(const typename VDBMappingT::UpdateGridT::Ptr update) const;

  /*!
   * \brief Unpacks an update grid from a compressed bitstream
   *
   * \param msg Compressed Bitstream
   *
   * \returns Update Grid
   */
  typename VDBMappingT::UpdateGridT::Ptr
  msgToGrid(const std::shared_ptr<std_msgs::msg::String> msg) const;

  /*!
   * \brief Unpacks an update grid from a ROS2 msg
   *
   * \param msg Compressed Bitstream as ROS2 msg
   *
   * \returns Update Grid
   */
  typename VDBMappingT::UpdateGridT::Ptr strToGrid(const std::string& msg) const;

private:
  /*!
   * \brief Subscriber for raw pointclouds
   */
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_sensor_cloud_sub;
  /*!
   * \brief Subscriber for scan aligned pointclouds
   */
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_aligned_cloud_sub;
  /*!
   * \brief Publisher for the marker array
   */
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_visualization_marker_pub;
  /*!
   * \brief Publisher for the point cloud
   */
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pointcloud_pub;
  /*!
   * \brief Publisher for map updates
   */
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_map_update_pub;
  /*!
   * \brief Publisher for map overwrites
   */
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_map_overwrite_pub;
  /*!
   * \brief Saves map in specified path from parameter server
   */
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_save_map_service;
  /*!
   * \brief Loads a map from specified path from service
   */
  rclcpp::Service<vdb_mapping_interfaces::srv::LoadMap>::SharedPtr m_load_map_service;
  /*!
   * \brief Service for reset map
   */
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_reset_map_service;
  /*!
   * \brief Service for dynamic reconfigure of parameters
   */
  /*!
   * \brief Service for map section requests
   */
  rclcpp::Service<vdb_mapping_interfaces::srv::GetMapSection>::SharedPtr m_get_map_section_service;
  /*!
   * \brief Service for triggering the map section request on a remote source
   */
  rclcpp::Service<vdb_mapping_interfaces::srv::TriggerMapSectionUpdate>::SharedPtr
    m_trigger_map_section_update_service;
  /*!
   * \brief Transformation buffer
   */
  std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
  /*!
   * \brief Transformation listener
   */
  std::shared_ptr<tf2_ros::TransformListener> m_tf_listener{nullptr};
  /*!
   * \brief Grid cell resolution
   */
  double m_resolution;
  /*!
   * \brief Sensor frame used for raycasting of scan aligned pointclouds
   */
  std::string m_sensor_frame;
  /*!
   * \brief Map Frame
   */
  std::string m_map_frame;
  /*!
   * \brief Map pointer
   */
  std::unique_ptr<VDBMappingT> m_vdb_map;
  /*!
   * \brief Map configuration
   */
  vdb_mapping::Config m_config;
  /*!
   * \brief Specifies whether a pointcloud should be published or not
   */
  bool m_publish_pointcloud;
  /*!
   * \brief Specifies whether the map should be published as markers or not
   */
  bool m_publish_vis_marker;
  /*!
   * \brief Specifies whether the mapping publishes map updates for remote use
   */
  bool m_publish_updates;
  /*!
   * \brief Specifies whether the mapping publishes map overwrites for remote use
   */
  bool m_publish_overwrites;
  /*!
   * \brief Specifies whether the mapping applies raw sensor data
   */
  bool m_apply_raw_sensor_data;
  /*!
   * \brief Specifies whether the data reduction is enabled
   */
  bool m_reduce_data;
  /*!
   * \brief Map of remote mapping source connections
   */
  std::map<std::string, RemoteSource> m_remote_sources;
};


#include "VDBMappingROS2_impl.hpp"
#endif /* VDB_MAPPING_ROS22_VDBMAPPINGROS2_HPP_INCLUDED */
