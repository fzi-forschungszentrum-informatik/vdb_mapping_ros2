// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2021 FZI Forschungszentrum Informatik
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
#ifndef VDB_MAPPING_ROS2_VDBMAPPINGROS2_H_INCLUDED
#define VDB_MAPPING_ROS2_VDBMAPPINGROS2_H_INCLUDED

#include <rclcpp/rclcpp.hpp>

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <std_srvs/srv/trigger.hpp>

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
  // bool saveMap(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  /*!
   * \brief Load stored map
   */
  // bool loadMap(vdb_mapping_msgs::LoadMap::Request& req, vdb_mapping_msgs::LoadMap::Response&
  // res);
  /*!
   * \brief Sensor callback for scan aligned Pointclouds
   * In contrast to the normal sensor callback here an additional sensor frame has to be specified
   * as origin of the raycasting
   *
   * \param msg PointCloud message
   */
  // void alignedCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
  /*!
   * \brief Sensor callback for raw pointclouds. All data will be transformed into the map frame.
   *
   * \param msg
   */
  // void sensorCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
  /*!
   * \brief Integrating the transformed pointcloud and sensor origins into the core mapping library
   *
   *
   * \param cloud Point cloud transformed into map coordinates
   * \param tf Sensor transform in map coordinates
   */
  // void insertPointCloud(const typename VDBMappingT::PointCloudT::Ptr cloud,
  // const geometry_msgs::TransformStamped transform);
  /*!
   * \brief Publishes a marker array and pointcloud representation of the map
   */
  // void publishMap() const;
  /*!
   * \brief Publishes a grid update as compressed serialized string
   *
   * \param update Update grid
   */
  // void publishUpdate(const typename VDBMappingT::UpdateGridT::Ptr update) const;
  /*!
   * \brief Listens to map updates and creats a map from these
   *
   * \param update_msg Single map update from a remote mapping instance
   */
  // void mapUpdateCallback(const std_msgs::String::ConstPtr& update_msg);
  /*!
   * \brief Returns a pointer to the map
   *
   * \returns VDB grid pointer
   */
  // const typename VDBMappingT::GridT::Ptr getMap();
  /*!
   * \brief Callback for map reset service call
   *
   * \param res result of the map reset
   * \returns result of map reset
   */
  bool mapResetCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                        const std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  /*!
   * \brief Callback for dynamic reconfigure of parameters
   *
   * \param config new configuration
   */
  // void dynamicReconfigureCallback(vdb_mapping_ros::VDBMappingROSConfig& config, uint32_t);

private:
  /*!
   * \brief Public node handle
   */
  // ros::NodeHandle m_nh;
  /*!
   * \brief Private node handle
   */
  // ros::NodeHandle m_priv_nh;
  /*!
   * \brief Subscriber for raw pointclouds
   */
  // ros::Subscriber m_sensor_cloud_sub;
  /*!
   * \brief Subscriber for scan aligned pointclouds
   */
  // ros::Subscriber m_aligned_cloud_sub;
  /*!
   * \brief Subscriber for map updates
   */
  // ros::Subscriber m_map_update_sub;
  /*!
   * \brief Publisher for the marker array
   */
  // ros::Publisher m_visualization_marker_pub;
  /*!
   * \brief Publisher for the point cloud
   */
  // ros::Publisher m_pointcloud_pub;
  /*!
   * \brief Publisher map updates
   */
  // ros::Publisher m_map_update_pub;
  /*!
   * \brief Saves map in specified path from parameter server
   */
  // ros::ServiceServer m_save_map_service_server;
  /*!
   * \brief Loads a map from specified path from service
   */
  // ros::ServiceServer m_load_map_service_server;
  /*!
   * \brief Service for reset map
   */
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_map_reset_service;
  /*!
   * \brief Service for dynamic reconfigure of parameters
   */
  // dynamic_reconfigure::Server<vdb_mapping_ros::VDBMappingROSConfig>
  // m_dynamic_reconfigure_service;
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
   * \brief Specifies if the node runs in normal or remote mapping mode
   */
  bool m_remote_mode;
};


#include "VDBMappingROS2.hpp"
#endif /* VDB_MAPPING_ROS22_VDBMAPPINGROS2_H_INCLUDED */
