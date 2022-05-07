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
  m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

}

template <typename VDBMappingT>
void VDBMappingROS2<VDBMappingT>::resetMap()
{
  RCLCPP_INFO(this->get_logger(), "Resetting Map");
  //m_vdb_map->resetMap();
  
  // publishMap();
}
