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
 * \date    2020-05-07
 *
 */
//----------------------------------------------------------------------

template <typename VDBMappingT>
VDBMappingROS2<VDBMappingT>::VDBMappingROS2()
  : Node("vdb_mapping_ros2")
{
  std::cout << "hi im a constructor" << std::endl;
  m_tf_buffer   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

  this->declare_parameter<double>("resolution", 0.1);
  this->get_parameter("resolution", m_resolution);
  m_vdb_map = std::make_unique<VDBMappingT>(m_resolution);

  this->declare_parameter<double>("max_range", 10.0);
  this->get_parameter("max_range", m_config.max_range);
  this->declare_parameter<double>("prob_hit", 0.7);
  this->get_parameter("prob_hit", m_config.prob_hit);
  this->declare_parameter<double>("prob_miss", 0.4);
  this->get_parameter("prob_miss", m_config.prob_miss);
  this->declare_parameter<double>("prob_thres_min", 0.12);
  this->get_parameter("prob_thres_min", m_config.prob_thres_min);
  this->declare_parameter<double>("prob_thres_max", 0.97);
  this->get_parameter("prob_thres_max", m_config.prob_thres_max);
  this->declare_parameter<std::string>("map_directory_path", "");
  this->get_parameter("map_directory_path", m_config.map_directory_path);

  // Configuring the VDB map
  m_vdb_map->setConfig(m_config);

  this->declare_parameter<bool>("publish_pointcloud", true);
  this->get_parameter("publish_pointcloud", m_publish_pointcloud);
  this->declare_parameter<bool>("publish_vis_marker", true);
  this->get_parameter("publish_vis_marker", m_publish_vis_marker);
  this->declare_parameter<bool>("publish_updates", true);
  this->get_parameter("publish_updates", m_publish_updates);
  this->declare_parameter<bool>("remote_mode", false);
  this->get_parameter("remote_mode", m_remote_mode);


  this->declare_parameter<std::string>("sensor_frame", "");
  this->get_parameter("sensor_frame", m_sensor_frame);
  if (m_sensor_frame.empty())
  {
    RCLCPP_WARN(this->get_logger(), "No sensor frame specified");
  }
  this->declare_parameter<std::string>("map_frame", "");
  this->get_parameter("map_frame", m_map_frame);
  if (m_map_frame.empty())
  {
    RCLCPP_WARN(this->get_logger(), "No map frame specified");
  }

  std::string raw_points_topic, aligned_points_topic;
  this->declare_parameter<std::string>("raw_points", "");
  this->get_parameter("raw_points", raw_points_topic);
  this->declare_parameter<std::string>("aligned_points", "");
  this->get_parameter("aligned_points", aligned_points_topic);

}

template <typename VDBMappingT>
void VDBMappingROS2<VDBMappingT>::resetMap()
{
  RCLCPP_INFO(this->get_logger(), "Resetting Map");
  m_vdb_map->resetMap();

  // publishMap();
}
