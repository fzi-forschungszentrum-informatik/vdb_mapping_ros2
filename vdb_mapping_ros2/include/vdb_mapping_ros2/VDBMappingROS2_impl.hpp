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

template <typename VDBMappingT>
VDBMappingROS2<VDBMappingT>::VDBMappingROS2()
  : Node("vdb_mapping_ros2")
{
  using namespace std::placeholders;

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
  this->declare_parameter<bool>("publish_overwrites", true);
  this->get_parameter("publish_overwrites", m_publish_overwrites);
  this->declare_parameter<bool>("apply_raw_sensor_data", true);
  this->get_parameter("apply_raw_sensor_data", m_apply_raw_sensor_data);
  this->declare_parameter<bool>("reduce_data", false);
  this->get_parameter("reduce_data", m_reduce_data);


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

  std::string raw_points_topic;
  std::string aligned_points_topic;
  this->declare_parameter<std::string>("raw_points", "");
  this->get_parameter("raw_points", raw_points_topic);
  this->declare_parameter<std::string>("aligned_points", "");
  this->get_parameter("aligned_points", aligned_points_topic);

  m_reset_map_service = this->create_service<std_srvs::srv::Trigger>(
    "~/reset_map", std::bind(&VDBMappingROS2::resetMapCallback, this, _1, _2));

  m_save_map_service = this->create_service<std_srvs::srv::Trigger>(
    "~/save_map", std::bind(&VDBMappingROS2::saveMap, this, _1, _2));

  m_load_map_service = this->create_service<vdb_mapping_interfaces::srv::LoadMap>(
    "~/load_map", std::bind(&VDBMappingROS2::loadMap, this, _1, _2));

  m_get_map_section_service = this->create_service<vdb_mapping_interfaces::srv::GetMapSection>(
    "~/get_map_section", std::bind(&VDBMappingROS2::getMapSectionCallback, this, _1, _2));

  m_trigger_map_section_update_service =
    this->create_service<vdb_mapping_interfaces::srv::TriggerMapSectionUpdate>(
      "~/trigger_map_section_update",
      std::bind(&VDBMappingROS2::triggerMapSectionUpdateCallback, this, _1, _2));

  m_pointcloud_pub =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("~/vdb_map_pointcloud", 1);
  m_visualization_marker_pub =
    this->create_publisher<visualization_msgs::msg::Marker>("~/vdb_map_visualization", 1);


  // Setting up remote sources
  std::vector<std::string> source_ids;
  this->declare_parameter<std::vector<std::string> >("remote_sources", std::vector<std::string>());
  this->get_parameter("remote_sources", source_ids);

  for (auto& source_id : source_ids)
  {
    std::string remote_namespace;
    this->declare_parameter<std::string>(source_id + ".namespace", "");
    this->get_parameter(source_id + ".namespace", remote_namespace);

    RemoteSource remote_source;
    this->declare_parameter<bool>(source_id + ".apply_remote_updates", false);
    this->get_parameter(source_id + ".apply_remote_updates", remote_source.apply_remote_updates);
    this->declare_parameter<bool>(source_id + ".apply_remote_overwrites", false);
    this->get_parameter(source_id + ".apply_remote_overwrites",
                        remote_source.apply_remote_overwrites);
    if (remote_source.apply_remote_updates)
    {
      remote_source.map_update_sub = this->create_subscription<std_msgs::msg::String>(
        remote_namespace + "/vdb_map_updates",
        1,
        std::bind(&VDBMappingROS2::mapUpdateCallback, this, _1));
    }
    if (remote_source.apply_remote_overwrites)
    {
      remote_source.map_update_sub = this->create_subscription<std_msgs::msg::String>(
        remote_namespace + "/vdb_map_overwrites",
        1,
        std::bind(&VDBMappingROS2::mapUpdateCallback, this, _1));
    }
    remote_source.get_map_section_client =
      this->create_client<vdb_mapping_interfaces::srv::GetMapSection>(remote_namespace +
                                                                      "/get_map_section");
    m_remote_sources.insert(std::make_pair(source_id, remote_source));
  }

  if (m_publish_updates)
  {
    m_map_update_pub = this->create_publisher<std_msgs::msg::String>("~/vdb_map_updates", 1);
  }
  if (m_publish_overwrites)
  {
    m_map_overwrite_pub = this->create_publisher<std_msgs::msg::String>("~/vdb_map_overwrites", 1);
  }
  if (m_apply_raw_sensor_data)
  {
    m_sensor_cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      raw_points_topic, 1, std::bind(&VDBMappingROS2::sensorCloudCallback, this, _1));
    m_aligned_cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      aligned_points_topic, 1, std::bind(&VDBMappingROS2::alignedCloudCallback, this, _1));
  }
}

template <typename VDBMappingT>
void VDBMappingROS2<VDBMappingT>::resetMap()
{
  RCLCPP_INFO(this->get_logger(), "Resetting Map");
  m_vdb_map->resetMap();
  publishMap();
}

template <typename VDBMappingT>
bool VDBMappingROS2<VDBMappingT>::resetMapCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
  const std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  (void)req;
  resetMap();
  res->success = true;
  res->message = "Reset map successful.";
  return true;
}

template <typename VDBMappingT>
bool VDBMappingROS2<VDBMappingT>::saveMap(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
  const std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  (void)req;
  RCLCPP_INFO(this->get_logger(), "Saving Map");
  res->success = m_vdb_map->saveMap();
  return res->success;
}

template <typename VDBMappingT>
bool VDBMappingROS2<VDBMappingT>::loadMap(
  const std::shared_ptr<vdb_mapping_interfaces::srv::LoadMap::Request> req,
  const std::shared_ptr<vdb_mapping_interfaces::srv::LoadMap::Response> res)
{
  RCLCPP_INFO(this->get_logger(), "Loading Map");
  bool success = m_vdb_map->loadMap(req->path);
  publishMap();
  res->success = success;
  return success;
}

template <typename VDBMappingT>
VDBMappingT& VDBMappingROS2<VDBMappingT>::getMap()
{
  return *m_vdb_map;
}

template <typename VDBMappingT>
const VDBMappingT& VDBMappingROS2<VDBMappingT>::getMap() const
{
  return *m_vdb_map;
}

template <typename VDBMappingT>
bool VDBMappingROS2<VDBMappingT>::getMapSectionCallback(
  const std::shared_ptr<vdb_mapping_interfaces::srv::GetMapSection::Request> req,
  const std::shared_ptr<vdb_mapping_interfaces::srv::GetMapSection::Response> res)
{
  geometry_msgs::msg::TransformStamped source_to_map_tf;
  try
  {
    source_to_map_tf = m_tf_buffer->lookupTransform(
      m_map_frame, req->header.frame_id, rclcpp::Time(0), rclcpp::Duration(1, 0));
  }
  catch (tf2::TransformException& ex)
  {
    RCLCPP_ERROR(this->get_logger(),
                 "Could not transform %s to %s: %s",
                 m_map_frame.c_str(),
                 m_sensor_frame.c_str(),
                 ex.what());
    res->success = false;
    return true;
  }
  res->map     = gridToStr(m_vdb_map->getMapSection(req->bounding_box.min_corner.x,
                                                req->bounding_box.min_corner.y,
                                                req->bounding_box.min_corner.z,
                                                req->bounding_box.max_corner.x,
                                                req->bounding_box.max_corner.y,
                                                req->bounding_box.max_corner.z,
                                                tf2::transformToEigen(source_to_map_tf).matrix()));
  res->success = true;

  return true;
}

template <typename VDBMappingT>
bool VDBMappingROS2<VDBMappingT>::triggerMapSectionUpdateCallback(
  const std::shared_ptr<vdb_mapping_interfaces::srv::TriggerMapSectionUpdate::Request> req,
  const std::shared_ptr<vdb_mapping_interfaces::srv::TriggerMapSectionUpdate::Response> res)
{
  auto remote_source = m_remote_sources.find(req->remote_source);
  if (remote_source == m_remote_sources.end())
  {
    std::stringstream ss;
    ss << "Key " << req->remote_source << " not found. Available sources are: ";
    for (auto& source : m_remote_sources)
    {
      ss << source.first << ", ";
    }
    RCLCPP_WARN(this->get_logger(), ss.str().c_str());
    res->success = false;
    return true;
  }

  auto request = std::make_shared<vdb_mapping_interfaces::srv::GetMapSection::Request>();

  request->header       = req->header;
  request->bounding_box = req->bounding_box;
  auto result           = remote_source->second.get_map_section_client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
      rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = result.get();
    if (response->success)
    {
      m_vdb_map->overwriteMap(strToGrid(response->map));
      publishMap();
    }
    res->success = response->success;
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to call servcie get_map_section");
    res->success = false;
  }

  return true;
}

template <typename VDBMappingT>
std_msgs::msg::String
VDBMappingROS2<VDBMappingT>::gridToMsg(const typename VDBMappingT::UpdateGridT::Ptr update) const
{
  std_msgs::msg::String msg;
  msg.data = gridToStr(update);
  return msg;
}

template <typename VDBMappingT>
std::string
VDBMappingROS2<VDBMappingT>::gridToStr(const typename VDBMappingT::UpdateGridT::Ptr update) const
{
  openvdb::GridPtrVec grids;
  grids.push_back(update);
  std::ostringstream oss(std::ios_base::binary);
  openvdb::io::Stream(oss).write(grids);
  return oss.str();
}

template <typename VDBMappingT>
typename VDBMappingT::UpdateGridT::Ptr
VDBMappingROS2<VDBMappingT>::msgToGrid(const std::shared_ptr<std_msgs::msg::String> msg) const
{
  return strToGrid(msg->data);
}

template <typename VDBMappingT>
typename VDBMappingT::UpdateGridT::Ptr
VDBMappingROS2<VDBMappingT>::strToGrid(const std::string& msg) const
{
  std::istringstream iss(msg);
  openvdb::io::Stream strm(iss);
  openvdb::GridPtrVecPtr grids;
  grids = strm.getGrids();
  // This cast might fail if different VDB versions are used.
  // Corresponding error messages are generated by VDB directly
  typename VDBMappingT::UpdateGridT::Ptr update_grid =
    openvdb::gridPtrCast<typename VDBMappingT::UpdateGridT>(grids->front());
  return update_grid;
}

template <typename VDBMappingT>
void VDBMappingROS2<VDBMappingT>::mapUpdateCallback(
  const std::shared_ptr<std_msgs::msg::String> update_msg)
{
  if (!m_reduce_data)
  {
    m_vdb_map->updateMap(msgToGrid(update_msg));
  }
  else
  {
    m_vdb_map->updateMap(m_vdb_map->raycastUpdateGrid(msgToGrid(update_msg)));
  }
  publishMap();
}

template <typename VDBMappingT>
void VDBMappingROS2<VDBMappingT>::mapOverwriteCallback(
  const std::shared_ptr<std_msgs::msg::String> update_msg)
{
  m_vdb_map->overwriteMap(msgToGrid(update_msg));
  publishMap();
}

template <typename VDBMappingT>
void VDBMappingROS2<VDBMappingT>::alignedCloudCallback(
  const std::shared_ptr<sensor_msgs::msg::PointCloud2> cloud_msg)
{
  typename VDBMappingT::PointCloudT::Ptr cloud(new typename VDBMappingT::PointCloudT);
  pcl::fromROSMsg(*cloud_msg, *cloud);
  geometry_msgs::msg::TransformStamped sensor_to_map_tf;
  try
  {
    // Get sensor origin transform in map coordinates
    sensor_to_map_tf =
      m_tf_buffer->lookupTransform(m_map_frame, m_sensor_frame, cloud_msg->header.stamp);
  }
  catch (tf2::TransformException& ex)
  {
    RCLCPP_ERROR(this->get_logger(),
                 "Could not transform %s to %s: %s",
                 m_map_frame.c_str(),
                 m_sensor_frame.c_str(),
                 ex.what());
    return;
  }
  // If aligned map is not already in correct map frame, transform it
  if (m_map_frame != cloud_msg->header.frame_id)
  {
    geometry_msgs::msg::TransformStamped map_to_map_tf;
    try
    {
      map_to_map_tf = m_tf_buffer->lookupTransform(
        m_map_frame, cloud_msg->header.frame_id, cloud_msg->header.stamp);
    }
    catch (tf2::TransformException& ex)
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Could not transform %s to %s: %s",
                   m_map_frame.c_str(),
                   cloud_msg->header.frame_id.c_str(),
                   ex.what());
      return;
    }
    pcl::transformPointCloud(*cloud, *cloud, tf2::transformToEigen(map_to_map_tf).matrix());
    cloud->header.frame_id = m_map_frame;
  }
  insertPointCloud(cloud, sensor_to_map_tf);
}
template <typename VDBMappingT>
void VDBMappingROS2<VDBMappingT>::sensorCloudCallback(
  const std::shared_ptr<sensor_msgs::msg::PointCloud2> cloud_msg)
{
  typename VDBMappingT::PointCloudT::Ptr cloud(new typename VDBMappingT::PointCloudT);
  pcl::fromROSMsg(*cloud_msg, *cloud);
  geometry_msgs::msg::TransformStamped sensor_to_map_tf;
  try
  {
    // Get sensor origin transform in map coordinates
    sensor_to_map_tf = m_tf_buffer->lookupTransform(
      m_map_frame, cloud_msg->header.frame_id, cloud_msg->header.stamp);
  }
  catch (tf2::TransformException& ex)
  {
    RCLCPP_ERROR(this->get_logger(),
                 "Could not transform %s to %s: %s",
                 m_map_frame.c_str(),
                 cloud_msg->header.frame_id.c_str(),
                 ex.what());
    return;
  }
  // Transform pointcloud into map reference system
  pcl::transformPointCloud(*cloud, *cloud, tf2::transformToEigen(sensor_to_map_tf).matrix());
  cloud->header.frame_id = m_map_frame;
  insertPointCloud(cloud, sensor_to_map_tf);
}


template <typename VDBMappingT>
void VDBMappingROS2<VDBMappingT>::insertPointCloud(
  const typename VDBMappingT::PointCloudT::Ptr cloud,
  const geometry_msgs::msg::TransformStamped transform)
{
  Eigen::Matrix<double, 3, 1> sensor_to_map_eigen = tf2::transformToEigen(transform).translation();
  typename VDBMappingT::UpdateGridT::Ptr update;
  typename VDBMappingT::UpdateGridT::Ptr overwrite;
  // Integrate data into vdb grid
  m_vdb_map->insertPointCloud(cloud, sensor_to_map_eigen, update, overwrite, m_reduce_data);
  if (m_publish_updates)
  {
    m_map_update_pub->publish(gridToMsg(update));
  }
  if (m_publish_overwrites)
  {
    m_map_overwrite_pub->publish(gridToMsg(update));
  }
  publishMap();
}

template <typename VDBMappingT>
void VDBMappingROS2<VDBMappingT>::publishMap() const
{
  if (!(m_publish_pointcloud || m_publish_vis_marker))
  {
    return;
  }
  typename VDBMappingT::GridT::Ptr grid = m_vdb_map->getGrid();
  bool publish_vis_marker;
  publish_vis_marker =
    (m_publish_vis_marker && this->count_subscribers("~/vdb_map_visualization") > 0);
  bool publish_pointcloud;
  publish_pointcloud =
    (m_publish_pointcloud && this->count_subscribers("~/vdb_map_pointcloud") > 0);

  visualization_msgs::msg::Marker visualization_marker_msg;
  sensor_msgs::msg::PointCloud2 cloud_msg;
  VDBMappingTools<VDBMappingT>::createMappingOutput(m_vdb_map->getGrid(),
                                                    m_map_frame,
                                                    visualization_marker_msg,
                                                    cloud_msg,
                                                    m_publish_vis_marker,
                                                    m_publish_pointcloud);
  if (publish_vis_marker)
  {
    visualization_marker_msg.header.stamp = this->now();
    m_visualization_marker_pub->publish(visualization_marker_msg);
  }
  if (publish_pointcloud)
  {
    cloud_msg.header.stamp = this->now();
    m_pointcloud_pub->publish(cloud_msg);
  }
}
