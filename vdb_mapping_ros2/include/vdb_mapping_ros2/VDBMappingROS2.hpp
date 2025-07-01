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
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <vdb_mapping/OccupancyVDBMapping.hpp>
#include <vdb_mapping/VDBMapping.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <vdb_mapping_interfaces/srv/add_artificial_areas.hpp>
#include <vdb_mapping_interfaces/srv/add_points_to_grid.hpp>
#include <vdb_mapping_interfaces/srv/batch_raytrace.hpp>
#include <vdb_mapping_interfaces/srv/get_map_section.hpp>
#include <vdb_mapping_interfaces/srv/get_occ_grid.hpp>
#include <vdb_mapping_interfaces/srv/load_map.hpp>
#include <vdb_mapping_interfaces/srv/load_map_from_pcd.hpp>
#include <vdb_mapping_interfaces/srv/raytrace.hpp>
#include <vdb_mapping_interfaces/srv/remove_points_from_grid.hpp>
#include <vdb_mapping_interfaces/srv/toggle_remote_source.hpp>
#include <vdb_mapping_interfaces/srv/trigger_map_section_update.hpp>
#include <visualization_msgs/msg/marker.hpp>

#define BOOST_BIND_NO_PLACEHOLDERS
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <vdb_mapping_ros2/VDBMappingTools.hpp>

struct RemoteSource
{
  rclcpp::Subscription<vdb_mapping_interfaces::msg::UpdateGrid>::SharedPtr map_section_sub;
  rclcpp::Subscription<vdb_mapping_interfaces::msg::UpdateGrid>::SharedPtr map_full_section_sub;
  rclcpp::Client<vdb_mapping_interfaces::srv::GetMapSection>::SharedPtr get_map_section_client;
  rclcpp::Client<vdb_mapping_interfaces::srv::GetMapSection>::SharedPtr get_map_full_section_client;
  bool apply_remote_sections;
  bool apply_remote_full_sections;
  bool active;
};

struct SensorSource
{
  std::string source_id;
  std::string topic;
  std::string sensor_origin_frame;
  double max_range;
  double max_rate;
  bool reliable;
};

template <typename VDBMappingT>
class VDBMappingROS2 : public rclcpp::Node
{
public:
  /*!
   * \brief Creates a new VDBMappingROS instance
   */
  explicit VDBMappingROS2(const rclcpp::NodeOptions& options)
    : Node("vdb_mapping_ros2", options)
  {
    using namespace std::placeholders;

    m_tf_buffer   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

    m_accumulation_cb_group =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_visualization_cb_group =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_remote_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    setUpVDBMap();
    setUpLocalSources();
    setUpRemoteSources();
    setUpVisualization();
    setUpServices();
    setUpPublishers();
    setUpMapServer();
  }


  virtual ~VDBMappingROS2(){};

  /*!
   * \brief Resets the current map
   */
  void resetMap()
  {
    RCLCPP_INFO(this->get_logger(), "Resetting Map");
    m_vdb_map->resetMap();
    publishMap();
  }
  /*!
   * \brief Saves the current map
   */
  bool saveMap(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
               const std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    (void)req;
    RCLCPP_INFO(this->get_logger(), "Saving Map");
    res->success = m_vdb_map->saveMap();
    return res->success;
  }
  /*!
   * \brief Saves the active values of the current map as PCD file
   */
  bool saveMapToPCD(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                    const std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    (void)req;
    RCLCPP_INFO(this->get_logger(), "Saving Map to PCD");
    res->success = m_vdb_map->saveMapToPCD();
    return res->success;
  }
  /*!
   * \brief Load stored map
   */
  bool loadMap(const std::shared_ptr<vdb_mapping_interfaces::srv::LoadMap::Request> req,
               const std::shared_ptr<vdb_mapping_interfaces::srv::LoadMap::Response> res)
  {
    RCLCPP_INFO(this->get_logger(), "Loading Map");
    bool success = m_vdb_map->loadMap(req->path);
    publishMap();
    res->success = success;
    return success;
  }

  /*!
   * \brief Load stored map
   */
  bool
  loadMapFromPCD(const std::shared_ptr<vdb_mapping_interfaces::srv::LoadMapFromPCD::Request> req,
                 const std::shared_ptr<vdb_mapping_interfaces::srv::LoadMapFromPCD::Response> res)
  {
    RCLCPP_INFO(this->get_logger(), "Loading Map from PCD file");
    bool success = m_vdb_map->loadMapFromPCD(req->path, req->set_background, req->clear_map);
    publishMap();
    res->success = success;
    return success;
  }
  /*!
   * \brief Sensor callback for Pointclouds
   *
   * If the sensor_origin_frame is not empty it will be used instead of the frame id
   * of the input cloud as origin of the raycasting
   *
   * \param cloud_msg PointCloud message
   * \param sensor_source Sensor source corresponding to the Pointcloud
   */
  void cloudCallback(const std::shared_ptr<sensor_msgs::msg::PointCloud2> cloud_msg,
                     const SensorSource& sensor_source)
  {
    typename VDBMappingT::PointCloudT::Ptr cloud(new typename VDBMappingT::PointCloudT);
    pcl::fromROSMsg(*cloud_msg, *cloud);
    geometry_msgs::msg::TransformStamped cloud_origin_tf;

    std::string sensor_frame = sensor_source.sensor_origin_frame.empty()
                                 ? cloud_msg->header.frame_id
                                 : sensor_source.sensor_origin_frame;

    // Get the origin of the sensor used as a starting point of the ray cast
    try
    {
      cloud_origin_tf =
        m_tf_buffer->lookupTransform(m_map_frame,
                                     sensor_frame,
                                     cloud_msg->header.stamp,
                                     rclcpp::Duration::from_seconds(m_tf_lookup_timeout));
    }
    catch (tf2::TransformException& ex)
    {
      RCLCPP_ERROR(this->get_logger(),
                   "MapToSensor: Could not transform %s to %s: %s",
                   m_map_frame.c_str(),
                   sensor_frame.c_str(),
                   ex.what());
      return;
    }
    // If aligned map is not already in correct map frame, transform it
    if (m_map_frame != cloud_msg->header.frame_id)
    {
      geometry_msgs::msg::TransformStamped origin_to_map_tf;
      try
      {
        origin_to_map_tf =
          m_tf_buffer->lookupTransform(m_map_frame,
                                       cloud_msg->header.frame_id,
                                       cloud_msg->header.stamp,
                                       rclcpp::Duration::from_seconds(m_tf_lookup_timeout));
      }
      catch (tf2::TransformException& ex)
      {
        RCLCPP_ERROR(this->get_logger(),
                     "MapToMessage: Could not transform %s to %s: %s",
                     m_map_frame.c_str(),
                     cloud_msg->header.frame_id.c_str(),
                     ex.what());
        return;
      }
      pcl::transformPointCloud(*cloud, *cloud, tf2::transformToEigen(origin_to_map_tf).matrix());
      cloud->header.frame_id = m_map_frame;
    }
    m_vdb_map->addDataToAccumulate(
      cloud, tf2::transformToEigen(cloud_origin_tf).translation(), sensor_source.source_id);
    if (!m_accumulate_updates)
    {
      m_vdb_map->integrateUpdate();
    }
  }
  /*!
   * \brief Integrating the transformed pointcloud and sensor origins into the core mapping library
   *
   *
   * \param cloud Point cloud transformed into map coordinates
   * \param tf Sensor transform in map coordinates
   */
  void insertPointCloud(const typename VDBMappingT::PointCloudT::Ptr cloud,
                        const geometry_msgs::msg::TransformStamped transform)
  {
    Eigen::Matrix<double, 3, 1> sensor_to_map_eigen =
      tf2::transformToEigen(transform).translation();
    // Integrate data into vdb grid
    m_vdb_map->insertPointCloud(cloud, sensor_to_map_eigen);
  }

  /*!
   * \brief Publishes a marker array and pointcloud representation of the map
   */
  void publishMap() const
  {
    if (!(m_publish_pointcloud || m_publish_vis_marker || m_publish_occupancy_grid))
    {
      return;
    }
    bool publish_vis_marker;
    publish_vis_marker =
      (m_publish_vis_marker && this->count_subscribers("~/vdb_map_visualization") > 0);
    bool publish_pointcloud;
    publish_pointcloud =
      (m_publish_pointcloud && this->count_subscribers("~/vdb_map_pointcloud") > 0);
    bool publish_occupancy_grid;
    publish_occupancy_grid =
      (m_publish_occupancy_grid && this->count_subscribers("~/vdb_map_occupancy") > 0);

    visualization_msgs::msg::Marker visualization_marker_msg;
    sensor_msgs::msg::PointCloud2 cloud_msg;
    nav_msgs::msg::OccupancyGrid occupancy_grid_msg;

    geometry_msgs::msg::TransformStamped map_to_robot_tf;
    try
    {
      map_to_robot_tf =
        m_tf_buffer->lookupTransform(m_map_frame, m_robot_frame, tf2::TimePointZero);
    }
    catch (tf2::TransformException& ex)
    {
      RCLCPP_ERROR(this->get_logger(),
                   "VisMapToRobot: Could not transform %s to %s: %s",
                   m_map_frame.c_str(),
                   m_robot_frame.c_str(),
                   ex.what());
      return;
    }


    std::shared_lock map_lock(*m_vdb_map->getMapMutex());
    VDBMappingTools<VDBMappingT>::createMappingOutput(
      m_vdb_map->getGrid(),
      m_map_frame,
      visualization_marker_msg,
      cloud_msg,
      occupancy_grid_msg,
      m_publish_vis_marker,
      m_publish_pointcloud,
      m_publish_occupancy_grid,
      map_to_robot_tf.transform.translation.z + m_lower_visualization_z_limit,
      map_to_robot_tf.transform.translation.z + m_upper_visualization_z_limit,
      m_resolution,
      m_two_dim_projection_threshold);
    map_lock.unlock();
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

    if (publish_occupancy_grid)
    {
      occupancy_grid_msg.header.stamp    = this->now();
      occupancy_grid_msg.header.frame_id = m_map_frame;
      occupancy_grid_msg.info.resolution = m_resolution;
      m_occupancy_grid_pub->publish(occupancy_grid_msg);
    }
  }

  void mapSectionCallback(const std::shared_ptr<vdb_mapping_interfaces::msg::UpdateGrid> update_msg,
                          const std::shared_ptr<RemoteSource> remote_source)
  {
    if (remote_source->active)
    {
      if (m_map_frame == update_msg->header.frame_id)
      {
        m_vdb_map->applyMapSectionUpdateGrid(
          m_vdb_map->template byteArrayToGrid<typename VDBMappingT::UpdateGridT>(update_msg->map),
          m_smooth_remote_sections,
          m_remote_section_smoothing_iterations);
      }
      else
      {
        geometry_msgs::msg::TransformStamped transform;
        try
        {
          transform =
            m_tf_buffer->lookupTransform(m_map_frame,
                                         update_msg->header.frame_id,
                                         update_msg->header.stamp,
                                         rclcpp::Duration::from_seconds(m_tf_lookup_timeout));
        }
        catch (tf2::TransformException& ex)
        {
          RCLCPP_ERROR(this->get_logger(),
                       "MapSection: Could not transform %s to %s: %s",
                       m_map_frame.c_str(),
                       update_msg->header.frame_id.c_str(),
                       ex.what());
          return;
        }
        m_vdb_map->transformAndApplyMapSectionUpdateGrid(
          m_vdb_map->template byteArrayToGrid<typename VDBMappingT::UpdateGridT>(update_msg->map),
          tf2::transformToEigen(transform).matrix(),
          m_smooth_remote_sections,
          m_remote_section_smoothing_iterations);
      }
    }
  }

  void
  mapFullSectionCallback(const std::shared_ptr<vdb_mapping_interfaces::msg::UpdateGrid> update_msg,
                         const std::shared_ptr<RemoteSource> remote_source)
  {
    if (remote_source->active)
    {
      if (m_map_frame == update_msg->header.frame_id)
      {
        m_vdb_map->applyMapSectionGrid(
          m_vdb_map->template byteArrayToGrid<typename VDBMappingT::GridT>(update_msg->map),
          m_smooth_remote_sections,
          m_remote_section_smoothing_iterations);
      }
      else
      {
        geometry_msgs::msg::TransformStamped transform;
        try
        {
          transform =
            m_tf_buffer->lookupTransform(m_map_frame,
                                         update_msg->header.frame_id,
                                         update_msg->header.stamp,
                                         rclcpp::Duration::from_seconds(m_tf_lookup_timeout));
        }
        catch (tf2::TransformException& ex)
        {
          RCLCPP_ERROR(this->get_logger(),
                       "MapFullSection: Could not transform %s to %s: %s",
                       m_map_frame.c_str(),
                       update_msg->header.frame_id.c_str(),
                       ex.what());
          return;
        }
        m_vdb_map->transformAndApplyMapSectionGrid(
          m_vdb_map->template byteArrayToGrid<typename VDBMappingT::GridT>(update_msg->map),
          tf2::transformToEigen(transform).matrix(),
          m_smooth_remote_sections,
          m_remote_section_smoothing_iterations);
      }
    }
  }

  /*!
   * \brief Get the map frame name
   *
   * \returns Map frame name
   */
  const std::string& getMapFrame() const { return m_map_frame; }

  /*!
   * \brief Returns the map
   *
   * \returns VDB map
   */
  std::shared_ptr<VDBMappingT> getMap() { return m_vdb_map; }

  /*!
   * \brief Returns the map
   *
   * \returns VDB map
   */
  const std::shared_ptr<VDBMappingT> getMap() const { return m_vdb_map; }

  /*!
   * \brief Callback for map reset service call
   *
   * \param res result of the map reset
   * \returns result of map reset
   */
  bool resetMapCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                        const std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    (void)req;
    resetMap();
    res->success = true;
    res->message = "Reset map successful.";
    return true;
  }

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
    const std::shared_ptr<vdb_mapping_interfaces::srv::GetMapSection::Response> res)
  {
    geometry_msgs::msg::TransformStamped source_to_map_tf;
    try
    {
      source_to_map_tf =
        m_tf_buffer->lookupTransform(m_map_frame,
                                     req->header.frame_id,
                                     rclcpp::Time(0),
                                     rclcpp::Duration::from_seconds(m_tf_lookup_timeout));
    }
    catch (tf2::TransformException& ex)
    {
      RCLCPP_ERROR(this->get_logger(),
                   "GetMapSection: Could not transform %s to %s: %s",
                   m_map_frame.c_str(),
                   req->header.frame_id.c_str(),
                   ex.what());
      res->success = false;
      return true;
    }
    res->section.map = m_vdb_map->template gridToByteArray<typename VDBMappingT::UpdateGridT>(
      m_vdb_map->getMapSectionUpdateGrid(
        Eigen::Matrix<double, 3, 1>(req->bounding_box.min_corner.x,
                                    req->bounding_box.min_corner.y,
                                    req->bounding_box.min_corner.z),
        Eigen::Matrix<double, 3, 1>(req->bounding_box.max_corner.x,
                                    req->bounding_box.max_corner.y,
                                    req->bounding_box.max_corner.z),
        tf2::transformToEigen(source_to_map_tf).matrix()));
    res->section.header.frame_id = m_map_frame;
    res->section.header.stamp    = this->now();
    res->success                 = true;

    return true;
  }

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
    auto result = remote_source->second->get_map_section_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      auto response = result.get();
      if (response->success)
      {
        m_vdb_map->updateMap(m_vdb_map->template byteArrayToGrid<typename VDBMappingT::UpdateGridT>(
          response->section.map));
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

  /*!
   * \brief Callback for triggering a map full section request on a remote source
   *
   * \param req Coordinates, reference frame and remote source identifier of the map section
   * \param res Result of triggering section request
   *
   * \returns Result of triggering section request
   */
  bool triggerMapFullSectionUpdateCallback(
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
    auto result = remote_source->second->get_map_full_section_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      auto response = result.get();
      if (response->success)
      {
        m_vdb_map->updateMap(m_vdb_map->template byteArrayToGrid<typename VDBMappingT::UpdateGridT>(
          response->section.map));
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

  /*!
   * \brief Callback for adding points directly into the grid
   *
   * \param req Pointcloud which should be added into the grid
   * \param res Result of adding points request
   *
   * \returns Result of adding points request
   */
  bool addPointsToGridCallback(
    const std::shared_ptr<vdb_mapping_interfaces::srv::AddPointsToGrid::Request> req,
    const std::shared_ptr<vdb_mapping_interfaces::srv::AddPointsToGrid::Response> res)
  {
    typename VDBMappingT::PointCloudT::Ptr cloud(new typename VDBMappingT::PointCloudT);
    pcl::fromROSMsg(req->points, *cloud);
    res->success = m_vdb_map->addPointsToGrid(cloud);
    return true;
  }

  /*!
   * \brief Callback for removing points directly from the grid
   *
   * \param req Pointcloud which should be removed from the grid
   * \param res Result of removing points request
   *
   * \returns Result of removing points request
   */
  bool removePointsFromGridCallback(
    const std::shared_ptr<vdb_mapping_interfaces::srv::RemovePointsFromGrid::Request> req,
    const std::shared_ptr<vdb_mapping_interfaces::srv::RemovePointsFromGrid::Response> res)
  {
    typename VDBMappingT::PointCloudT::Ptr cloud(new typename VDBMappingT::PointCloudT);
    pcl::fromROSMsg(req->points, *cloud);
    res->success = m_vdb_map->removePointsFromGrid(cloud);
    return true;
  }


  /*!
   * \brief Callback for raytrace service call
   *
   * \param req Origin and direction for raytracing
   * \param res Resulting point of the raytrace
   *
   * \returns result of raytrace service
   */
  bool raytraceCallback(const std::shared_ptr<vdb_mapping_interfaces::srv::Raytrace::Request> req,
                        const std::shared_ptr<vdb_mapping_interfaces::srv::Raytrace::Response> res)
  {
    auto batch_req = std::make_shared<vdb_mapping_interfaces::srv::BatchRaytrace::Request>();
    auto batch_res = std::make_shared<vdb_mapping_interfaces::srv::BatchRaytrace::Response>();

    batch_req->header = req->header;
    batch_req->rays.push_back(req->ray);
    batchRaytraceCallback(batch_req, batch_res);
    res->header    = batch_res->header;
    res->success   = batch_res->successes[0];
    res->end_point = batch_res->end_points[0];

    return true;
  }
  bool batchRaytraceCallback(
    const std::shared_ptr<vdb_mapping_interfaces::srv::BatchRaytrace::Request> req,
    const std::shared_ptr<vdb_mapping_interfaces::srv::BatchRaytrace::Response> res)
  {
    geometry_msgs::msg::TransformStamped reference_tf;
    res->header.frame_id = m_map_frame;
    res->header.stamp    = req->header.stamp;
    res->successes.resize(req->rays.size());
    res->end_points.resize(req->rays.size());
    try
    {
      reference_tf =
        m_tf_buffer->lookupTransform(m_map_frame,
                                     req->header.frame_id.c_str(),
                                     req->header.stamp,
                                     rclcpp::Duration::from_seconds(m_tf_lookup_timeout));
    }
    catch (tf2::TransformException& ex)
    {
      RCLCPP_ERROR_STREAM(this->get_logger(),
                          "BatchRaytrace: Transform to map frame failed: " << ex.what());
      for (size_t i = 0; i < req->rays.size(); i++)
      {
        res->successes[i]  = false;
        res->end_points[i] = geometry_msgs::msg::Point();
      }
      return true;
    }

    Eigen::Matrix<double, 4, 4> m = tf2::transformToEigen(reference_tf).matrix();

    std::vector<openvdb::Vec3d> ray_origins_world;
    std::vector<openvdb::Vec3d> ray_directions;
    std::vector<double> max_ray_lengths;
    std::vector<openvdb::Vec3d> end_points;

    for (size_t i = 0; i < req->rays.size(); i++)
    {
      Eigen::Matrix<double, 4, 1> origin, direction;
      origin << req->rays[i].origin.x, req->rays[i].origin.y, req->rays[i].origin.z, 1;
      direction << req->rays[i].direction.x, req->rays[i].direction.y, req->rays[i].direction.z, 0;

      origin    = m * origin;
      direction = m * direction;

      ray_origins_world.push_back(openvdb::Vec3d(origin.x(), origin.y(), origin.z()));
      ray_directions.push_back(openvdb::Vec3d(direction.x(), direction.y(), direction.z()));
      max_ray_lengths.push_back(req->rays[i].max_ray_length);
    }
    m_vdb_map->raytrace(
      ray_origins_world, ray_directions, max_ray_lengths, res->successes, end_points);

    for (size_t i = 0; i < end_points.size(); i++)
    {
      geometry_msgs::msg::Point p;
      p.x                = end_points[i].x();
      p.y                = end_points[i].y();
      p.z                = end_points[i].z();
      res->end_points[i] = p;
    }
    return true;
  }


  bool addArtificialAreasCallback(
    const std::shared_ptr<vdb_mapping_interfaces::srv::AddArtificialAreas::Request> req,
    const std::shared_ptr<vdb_mapping_interfaces::srv::AddArtificialAreas::Response> res)
  {
    std::vector<std::vector<Eigen::Matrix<double, 4, 1>>> artificial_areas;
    if (req->artificial_areas.size() > 0)
    {
      geometry_msgs::msg::TransformStamped source_to_map_tf;
      try
      {
        source_to_map_tf =
          m_tf_buffer->lookupTransform(m_map_frame,
                                       req->artificial_areas[0].header.frame_id,
                                       rclcpp::Time(0),
                                       rclcpp::Duration::from_seconds(m_tf_lookup_timeout));
      }
      catch (tf2::TransformException& ex)
      {
        RCLCPP_ERROR(this->get_logger(),
                     "ArtificialArea: Could not transform %s to %s: %s",
                     m_map_frame.c_str(),
                     req->artificial_areas[0].header.frame_id.c_str(),
                     ex.what());
        res->success = false;
        return true;
      }
      Eigen::Matrix<double, 4, 4> transform;
      transform = tf2::transformToEigen(source_to_map_tf).matrix();

      for (auto& artificial_area : req->artificial_areas)
      {
        std::vector<Eigen::Matrix<double, 4, 1>> area;
        for (auto& p : artificial_area.polygon.points)
        {
          area.push_back(transform * Eigen::Matrix<double, 4, 1>(p.x, p.y, p.z, 1.0));
        }
        artificial_areas.push_back(area);
      }
    }
    double m_artificial_negative_height = -0.5;
    double m_artificial_positive_height = 1.5;

    m_vdb_map->addArtificialAreas(
      artificial_areas, m_artificial_negative_height, m_artificial_positive_height);
    res->success = true;
    return true;
  }
  bool removeArtificialAreasCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                     const std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    (void)req;
    m_vdb_map->restoreMapIntegrity();
    res->success = true;
    return true;
  }

  bool toggleRemoteSource(
    const std::shared_ptr<vdb_mapping_interfaces::srv::ToggleRemoteSource::Request> req,
    const std::shared_ptr<vdb_mapping_interfaces::srv::ToggleRemoteSource::Response> res)
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
    remote_source->second->active = req->toggle;
    if (remote_source->second->active)
    {
      std::cout << "Remote source " << req->remote_source << " set to active" << std::endl;
    }
    else
    {
      std::cout << "Remote source " << req->remote_source << " set to inactive" << std::endl;
    }
    res->success = true;
    return true;
  }

  void visualizationTimerCallback() { publishMap(); }

  void sectionTimerCallback()
  {
    geometry_msgs::msg::TransformStamped map_to_robot_tf;
    try
    {
      // Get sensor origin transform in map coordinates
      map_to_robot_tf =
        m_tf_buffer->lookupTransform(m_map_frame,
                                     m_section_update_frame,
                                     rclcpp::Time(0),
                                     rclcpp::Duration::from_seconds(m_tf_lookup_timeout));
    }
    catch (tf2::TransformException& ex)
    {
      RCLCPP_ERROR(this->get_logger(),
                   "SectionTimer: Could not transform %s to %s: %s",
                   m_map_frame.c_str(),
                   m_section_update_frame.c_str(),
                   ex.what());
      return;
    }
    typename VDBMappingT::UpdateGridT::Ptr section = m_vdb_map->getMapSectionUpdateGrid(
      m_section_min_coord, m_section_max_coord, tf2::transformToEigen(map_to_robot_tf).matrix());
    vdb_mapping_interfaces::msg::UpdateGrid msg;
    msg.header.frame_id = m_map_frame;
    msg.header.stamp    = map_to_robot_tf.header.stamp;
    msg.map = m_vdb_map->template gridToByteArray<typename VDBMappingT::UpdateGridT>(section);
    m_map_section_pub->publish(msg);
  }
  void fullSectionTimerCallback()
  {
    geometry_msgs::msg::TransformStamped map_to_robot_tf;
    try
    {
      // Get sensor origin transform in map coordinates
      map_to_robot_tf =
        m_tf_buffer->lookupTransform(m_map_frame,
                                     m_section_update_frame,
                                     rclcpp::Time(0),
                                     rclcpp::Duration::from_seconds(m_tf_lookup_timeout));
    }
    catch (tf2::TransformException& ex)
    {
      RCLCPP_ERROR(this->get_logger(),
                   "FullSectionTimer: Could not transform %s to %s: %s",
                   m_map_frame.c_str(),
                   m_section_update_frame.c_str(),
                   ex.what());
      return;
    }

    typename VDBMappingT::GridT::Ptr section = m_vdb_map->getMapSectionGrid(
      m_section_min_coord, m_section_max_coord, tf2::transformToEigen(map_to_robot_tf).matrix());
    vdb_mapping_interfaces::msg::UpdateGrid msg;
    msg.header.frame_id = m_map_frame;
    msg.header.stamp    = map_to_robot_tf.header.stamp;
    msg.map             = m_vdb_map->template gridToByteArray<typename VDBMappingT::GridT>(section);
    m_map_section_pub->publish(msg);
  }

private:
  void setUpVDBMap()
  {
    this->declare_parameter<bool>("fast_mode", false);
    this->get_parameter("fast_mode", m_config.fast_mode);
    this->declare_parameter<double>("accumulation_period", 1);
    this->get_parameter("accumulation_period", m_config.accumulation_period);
    this->declare_parameter<double>("resolution", 0.05);
    this->get_parameter("resolution", m_resolution);
    m_vdb_map = std::make_shared<VDBMappingT>(m_resolution);

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
    this->declare_parameter<int>("two_dim_projection_threshold", 5);
    this->get_parameter("two_dim_projection_threshold", m_two_dim_projection_threshold);
    this->declare_parameter<double>("tf_lookup_timeout", 0.1);
    this->get_parameter("tf_lookup_timeout", m_tf_lookup_timeout);
    this->declare_parameter<bool>("smooth_remote_sections", false);
    this->get_parameter("smooth_remote_sections", m_smooth_remote_sections);
    this->declare_parameter<int>("remote_section_smoothing_iterations", 2);
    this->get_parameter("remote_section_smoothing_iterations",
                        m_remote_section_smoothing_iterations);

    // Configuring the VDB map
    m_vdb_map->setConfig(m_config);

    this->declare_parameter<std::string>("map_frame", "");
    this->get_parameter("map_frame", m_map_frame);
    if (m_map_frame.empty())
    {
      RCLCPP_WARN(this->get_logger(), "No map frame specified");
    }
    std::unique_lock map_lock(*m_vdb_map->getMapMutex());
    m_vdb_map->getGrid()->insertMeta("ros/map_frame", openvdb::StringMetadata(m_map_frame));
    map_lock.unlock();
    this->declare_parameter<std::string>("robot_frame", "");
    this->get_parameter("robot_frame", m_robot_frame);
    if (m_robot_frame.empty())
    {
      RCLCPP_WARN(this->get_logger(), "No robot frame specified");
    }
  }

  void setUpLocalSources()
  {
    this->declare_parameter<bool>("apply_raw_sensor_data", true);
    this->get_parameter("apply_raw_sensor_data", m_apply_raw_sensor_data);

    if (m_apply_raw_sensor_data)
    {
      std::vector<std::string> source_ids;
      this->declare_parameter<std::vector<std::string>>("sources", std::vector<std::string>());
      this->get_parameter("sources", source_ids);

      for (auto& source_id : source_ids)
      {
        SensorSource sensor_source;
        sensor_source.source_id = source_id;
        this->declare_parameter<std::string>(source_id + ".topic", "");
        this->get_parameter(source_id + ".topic", sensor_source.topic);
        this->declare_parameter<std::string>(source_id + ".sensor_origin_frame", "");
        this->get_parameter(source_id + ".sensor_origin_frame", sensor_source.sensor_origin_frame);
        this->declare_parameter<double>(source_id + ".max_range", 0);
        this->get_parameter(source_id + ".max_range", sensor_source.max_range);
        this->declare_parameter<double>(source_id + ".max_rate", 0);
        this->get_parameter(source_id + ".max_rate", sensor_source.max_rate);
        this->declare_parameter<bool>(source_id + ".reliable", false);
        this->get_parameter(source_id + ".reliable", sensor_source.reliable);
        RCLCPP_INFO_STREAM(this->get_logger(), "Setting up source: " << source_id);

        if (sensor_source.topic.empty())
        {
          RCLCPP_ERROR_STREAM(this->get_logger(),
                              "No input topic specified for source: " << source_id);
          continue;
        }
        RCLCPP_INFO_STREAM(this->get_logger(), "Topic: " << sensor_source.topic);
        if (sensor_source.sensor_origin_frame.empty())
        {
          RCLCPP_INFO(this->get_logger(), "Using frame id of topic as raycast origin");
        }
        else
        {
          RCLCPP_INFO_STREAM(this->get_logger(),
                             "Using " << sensor_source.sensor_origin_frame << " as raycast origin");
        }

        rclcpp::SubscriptionOptions opt;
        opt.callback_group = m_accumulation_cb_group;

        rclcpp::QoS qos_profile(1);
        if (sensor_source.reliable)
        {
          qos_profile = qos_profile.durability_volatile().reliable();
        }
        else
        {
          qos_profile = qos_profile.durability_volatile().best_effort();
        }

        m_cloud_subs.push_back(this->create_subscription<sensor_msgs::msg::PointCloud2>(
          sensor_source.topic,
          qos_profile,
          [&, sensor_source](const std::shared_ptr<sensor_msgs::msg::PointCloud2> cloud_msg) {
            cloudCallback(cloud_msg, sensor_source);
          },
          opt));
        m_vdb_map->addInputSource(
          sensor_source.source_id, sensor_source.max_range, sensor_source.max_rate);
      }
      this->declare_parameter<bool>("accumulate_updates", false);
      this->get_parameter("accumulate_updates", m_accumulate_updates);
    }
  }
  void setUpRemoteSources()
  {
    using namespace std::placeholders;
    std::vector<std::string> source_ids;
    // Setting up remote sources
    this->declare_parameter<std::vector<std::string>>("remote_sources", std::vector<std::string>());
    this->get_parameter("remote_sources", source_ids);

    for (auto& source_id : source_ids)
    {
      std::string remote_namespace;
      this->declare_parameter<std::string>(source_id + ".namespace", "");
      this->get_parameter(source_id + ".namespace", remote_namespace);


      auto remote_source = std::make_shared<RemoteSource>();
      this->declare_parameter<bool>(source_id + ".apply_remote_sections", false);
      this->get_parameter(source_id + ".apply_remote_sections",
                          remote_source->apply_remote_sections);
      this->declare_parameter<bool>(source_id + ".apply_remote_full_sections", false);
      this->get_parameter(source_id + ".apply_remote_full_sections",
                          remote_source->apply_remote_full_sections);

      this->declare_parameter<bool>(source_id + ".autostart", true);
      this->get_parameter(source_id + ".autostart", remote_source->active);

      if (remote_source->apply_remote_sections)
      {
        remote_source->map_section_sub =
          this->create_subscription<vdb_mapping_interfaces::msg::UpdateGrid>(
            remote_namespace + "/vdb_map_sections",
            rclcpp::QoS(10).durability_volatile().best_effort(),
            [&, remote_source](
              const std::shared_ptr<vdb_mapping_interfaces::msg::UpdateGrid> cloud_msg) {
              mapSectionCallback(cloud_msg, remote_source);
            });
      }
      if (remote_source->apply_remote_full_sections)
      {
        remote_source->map_full_section_sub =
          this->create_subscription<vdb_mapping_interfaces::msg::UpdateGrid>(
            remote_namespace + "/vdb_map_full_sections",
            rclcpp::QoS(10).durability_volatile().best_effort(),
            [&, remote_source](
              const std::shared_ptr<vdb_mapping_interfaces::msg::UpdateGrid> cloud_msg) {
              mapFullSectionCallback(cloud_msg, remote_source);
            });
      }
      remote_source->get_map_section_client =
        this->create_client<vdb_mapping_interfaces::srv::GetMapSection>(remote_namespace +
                                                                        "/get_map_section");
      remote_source->get_map_full_section_client =
        this->create_client<vdb_mapping_interfaces::srv::GetMapSection>(remote_namespace +
                                                                        "/get_map_full_section");
      m_remote_sources.insert(std::make_pair(source_id, remote_source));
    }
  }
  void setUpVisualization()
  {
    this->declare_parameter<double>("z_limit_min", 0);
    this->get_parameter("z_limit_min", m_lower_visualization_z_limit);
    this->declare_parameter<double>("z_limit_max", 0);
    this->get_parameter("z_limit_max", m_upper_visualization_z_limit);

    m_param_sub = std::make_shared<rclcpp::ParameterEventHandler>(this);

    auto min_z_cb = [this](const rclcpp::Parameter& p) {
      m_lower_visualization_z_limit = p.as_double();
    };
    auto max_z_cb = [this](const rclcpp::Parameter& p) {
      m_upper_visualization_z_limit = p.as_double();
    };

    m_z_min_param_handle = m_param_sub->add_parameter_callback("z_limit_min", min_z_cb);
    m_z_min_param_handle = m_param_sub->add_parameter_callback("z_limit_max", max_z_cb);

    double visualization_rate;
    this->declare_parameter<double>("visualization_rate", 1.0);
    this->get_parameter("visualization_rate", visualization_rate);
    if (visualization_rate > 0.0)
    {
      m_visualization_timer =
        this->create_wall_timer(std::chrono::milliseconds((int)(1000.0 / visualization_rate)),
                                std::bind(&VDBMappingROS2::visualizationTimerCallback, this),
                                m_visualization_cb_group);
    }
  }
  void setUpServices()
  {
    using namespace std::placeholders;
    m_reset_map_service = this->create_service<std_srvs::srv::Trigger>(
      "~/reset_map", std::bind(&VDBMappingROS2::resetMapCallback, this, _1, _2));

    m_save_map_service = this->create_service<std_srvs::srv::Trigger>(
      "~/save_map", std::bind(&VDBMappingROS2::saveMap, this, _1, _2));

    m_save_map_to_pcd_service = this->create_service<std_srvs::srv::Trigger>(
      "~/save_map_to_pcd", std::bind(&VDBMappingROS2::saveMapToPCD, this, _1, _2));

    m_load_map_service = this->create_service<vdb_mapping_interfaces::srv::LoadMap>(
      "~/load_map", std::bind(&VDBMappingROS2::loadMap, this, _1, _2));

    m_load_map_from_pcd_service = this->create_service<vdb_mapping_interfaces::srv::LoadMapFromPCD>(
      "~/load_map_from_pcd", std::bind(&VDBMappingROS2::loadMapFromPCD, this, _1, _2));

    m_get_map_section_service = this->create_service<vdb_mapping_interfaces::srv::GetMapSection>(
      "~/get_map_section", std::bind(&VDBMappingROS2::getMapSectionCallback, this, _1, _2));

    m_trigger_map_section_update_service =
      this->create_service<vdb_mapping_interfaces::srv::TriggerMapSectionUpdate>(
        "~/trigger_map_section_update",
        std::bind(&VDBMappingROS2::triggerMapSectionUpdateCallback, this, _1, _2));

    m_trigger_map_full_section_update_service =
      this->create_service<vdb_mapping_interfaces::srv::TriggerMapSectionUpdate>(
        "~/trigger_map_full_section_update",
        std::bind(&VDBMappingROS2::triggerMapFullSectionUpdateCallback, this, _1, _2));

    m_raytrace_service = this->create_service<vdb_mapping_interfaces::srv::Raytrace>(
      "~/raytrace", std::bind(&VDBMappingROS2::raytraceCallback, this, _1, _2));

    m_batch_raytrace_service = this->create_service<vdb_mapping_interfaces::srv::BatchRaytrace>(
      "~/batch_raytrace", std::bind(&VDBMappingROS2::batchRaytraceCallback, this, _1, _2));

    m_add_points_to_grid_service =
      this->create_service<vdb_mapping_interfaces::srv::AddPointsToGrid>(
        "~/add_points_to_grid", std::bind(&VDBMappingROS2::addPointsToGridCallback, this, _1, _2));

    m_remove_points_from_grid_service =
      this->create_service<vdb_mapping_interfaces::srv::RemovePointsFromGrid>(
        "~/remove_points_from_grid",
        std::bind(&VDBMappingROS2::removePointsFromGridCallback, this, _1, _2));

    m_add_artificial_areas_service =
      this->create_service<vdb_mapping_interfaces::srv::AddArtificialAreas>(
        "~/add_artificial_areas",
        std::bind(&VDBMappingROS2::addArtificialAreasCallback, this, _1, _2));

    m_remove_artificial_areas_service = this->create_service<std_srvs::srv::Trigger>(
      "~/remove_artificial_areas",
      std::bind(&VDBMappingROS2::removeArtificialAreasCallback, this, _1, _2));

    m_toggle_remote_source_service =
      this->create_service<vdb_mapping_interfaces::srv::ToggleRemoteSource>(
        "~/toggle_remote_source", std::bind(&VDBMappingROS2::toggleRemoteSource, this, _1, _2));
  }
  void setUpPublishers()
  {
    this->declare_parameter<bool>("publish_pointcloud", true);
    this->get_parameter("publish_pointcloud", m_publish_pointcloud);
    this->declare_parameter<bool>("publish_vis_marker", true);
    this->get_parameter("publish_vis_marker", m_publish_vis_marker);
    this->declare_parameter<bool>("publish_occupancy_grid", true);
    this->get_parameter("publish_occupancy_grid", m_publish_occupancy_grid);
    this->declare_parameter<bool>("publish_sections", false);
    this->get_parameter("publish_sections", m_publish_sections);
    this->declare_parameter<bool>("publish_full_sections", false);
    this->get_parameter("publish_full_sections", m_publish_full_sections);

    if (m_publish_pointcloud)
    {
      m_pointcloud_pub =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("~/vdb_map_pointcloud", 1);
    }
    if (m_publish_vis_marker)
    {
      m_visualization_marker_pub =
        this->create_publisher<visualization_msgs::msg::Marker>("~/vdb_map_visualization", 1);
    }
    if (m_publish_occupancy_grid)
    {
      m_occupancy_grid_pub =
        this->create_publisher<nav_msgs::msg::OccupancyGrid>("~/vdb_map_occupancy", 1);
    }

    if (m_publish_sections)
    {
      m_map_section_pub = this->create_publisher<vdb_mapping_interfaces::msg::UpdateGrid>(
        "~/vdb_map_sections", rclcpp::QoS(1).durability_volatile().best_effort());

      double section_update_rate;
      this->declare_parameter<double>("section_update.rate", 1);
      this->get_parameter("section_update.rate", section_update_rate);
      m_section_timer =
        this->create_wall_timer(std::chrono::milliseconds((int)(1000.0 / section_update_rate)),
                                std::bind(&VDBMappingROS2::sectionTimerCallback, this),
                                m_remote_cb_group);

      this->declare_parameter<double>("section_update.min_coord.x", -10);
      this->get_parameter("section_update.min_coord.x", m_section_min_coord.x());
      this->declare_parameter<double>("section_update.min_coord.y", -10);
      this->get_parameter("section_update.min_coord.y", m_section_min_coord.y());
      this->declare_parameter<double>("section_update.min_coord.z", -10);
      this->get_parameter("section_update.min_coord.z", m_section_min_coord.z());
      this->declare_parameter<double>("section_update.max_coord.x", 10);
      this->get_parameter("section_update.max_coord.x", m_section_max_coord.x());
      this->declare_parameter<double>("section_update.max_coord.y", 10);
      this->get_parameter("section_update.max_coord.y", m_section_max_coord.y());
      this->declare_parameter<double>("section_update.max_coord.z", 10);
      this->get_parameter("section_update.max_coord.z", m_section_max_coord.z());
      this->declare_parameter<std::string>("section_update.frame", m_robot_frame);
      this->get_parameter("section_update.frame", m_section_update_frame);
    }
    if (m_publish_full_sections)
    {
      m_map_section_pub = this->create_publisher<vdb_mapping_interfaces::msg::UpdateGrid>(
        "~/vdb_map_full_sections", rclcpp::QoS(1).durability_volatile().best_effort());

      double section_update_rate;
      this->declare_parameter<double>("section_update.rate", 1);
      this->get_parameter("section_update.rate", section_update_rate);
      m_full_section_timer =
        this->create_wall_timer(std::chrono::milliseconds((int)(1000.0 / section_update_rate)),
                                std::bind(&VDBMappingROS2::fullSectionTimerCallback, this),
                                m_remote_cb_group);

      this->declare_parameter<double>("section_update.min_coord.x", -10);
      this->get_parameter("section_update.min_coord.x", m_section_min_coord.x());
      this->declare_parameter<double>("section_update.min_coord.y", -10);
      this->get_parameter("section_update.min_coord.y", m_section_min_coord.y());
      this->declare_parameter<double>("section_update.min_coord.z", -10);
      this->get_parameter("section_update.min_coord.z", m_section_min_coord.z());
      this->declare_parameter<double>("section_update.max_coord.x", 10);
      this->get_parameter("section_update.max_coord.x", m_section_max_coord.x());
      this->declare_parameter<double>("section_update.max_coord.y", 10);
      this->get_parameter("section_update.max_coord.y", m_section_max_coord.y());
      this->declare_parameter<double>("section_update.max_coord.z", 10);
      this->get_parameter("section_update.max_coord.z", m_section_max_coord.z());
      this->declare_parameter<std::string>("section_update.frame", m_robot_frame);
      this->get_parameter("section_update.frame", m_section_update_frame);
    }
  }

  void setUpMapServer()
  {
    // Load initial map file
    std::string initial_map_file;
    bool set_background;
    bool clear_map;
    this->declare_parameter<std::string>("map_server.initial_map_file", "");
    this->get_parameter("map_server.initial_map_file", initial_map_file);
    this->declare_parameter<bool>("map_server.set_background", false);
    this->get_parameter("map_server.set_background", set_background);
    this->declare_parameter<bool>("map_server.clear_map", false);
    this->get_parameter("map_server.clear_map", clear_map);
    if (initial_map_file != "")
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "Loading intial Map " << initial_map_file);
      m_vdb_map->loadMapFromPCD(initial_map_file, set_background, clear_map);
      publishMap();
    }
  }


  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> m_cloud_subs;
  /*!
   * \brief Subscriber for raw pointclouds
   */
  // rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_sensor_cloud_sub;
  /*!
   * \brief Subscriber for scan aligned pointclouds
   */
  // rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_aligned_cloud_sub;
  /*!
   * \brief Publisher for the marker array
   */
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_visualization_marker_pub;
  /*!
   * \brief Publisher for the point cloud
   */
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pointcloud_pub;

  /*!
   * /brief Publisher for the OccupancyGrid.
   */
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr m_occupancy_grid_pub;
  /*!
   * \brief Publisher for map sections
   */
  rclcpp::Publisher<vdb_mapping_interfaces::msg::UpdateGrid>::SharedPtr m_map_section_pub;
  /*!
   * \brief Publisher for full map sections
   */
  rclcpp::Publisher<vdb_mapping_interfaces::msg::UpdateGrid>::SharedPtr m_map_full_section_pub;
  /*!
   * \brief Saves map in specified path from parameter server
   */
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_save_map_service;
  /*!
   * \brief Saves the active values of the map as PCD file in the specified path from the paramter
   * server
   */
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_save_map_to_pcd_service;
  /*!
   * \brief Loads a map from specified path from service
   */
  rclcpp::Service<vdb_mapping_interfaces::srv::LoadMap>::SharedPtr m_load_map_service;
  /*!
   * \brief Generates a map from a PCD file specified by the path from service
   */
  rclcpp::Service<vdb_mapping_interfaces::srv::LoadMapFromPCD>::SharedPtr
    m_load_map_from_pcd_service;
  /*!
   * \brief Service for reset map
   */
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_reset_map_service;
  /*!
   * \brief Service for dynamic reconfigure of parameters
   */
  // TODO
  /*!
   * \brief Service for raytracing
   */
  rclcpp::Service<vdb_mapping_interfaces::srv::Raytrace>::SharedPtr m_raytrace_service;
  /*!
   * \brief Service for batch raytracing
   */
  rclcpp::Service<vdb_mapping_interfaces::srv::BatchRaytrace>::SharedPtr m_batch_raytrace_service;
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
   * \brief Service for triggering the map section request on a remote source
   */
  rclcpp::Service<vdb_mapping_interfaces::srv::TriggerMapSectionUpdate>::SharedPtr
    m_trigger_map_full_section_update_service;
  /*!
   * \brief Service for adding points directly into the grid.
   */
  rclcpp::Service<vdb_mapping_interfaces::srv::AddPointsToGrid>::SharedPtr
    m_add_points_to_grid_service;
  /*!
   * \brief Service for removing points directly from the grid.
   */
  rclcpp::Service<vdb_mapping_interfaces::srv::RemovePointsFromGrid>::SharedPtr
    m_remove_points_from_grid_service;
  /*!
   * \brief Service for adding artificial areas to the grid.
   */
  rclcpp::Service<vdb_mapping_interfaces::srv::AddArtificialAreas>::SharedPtr
    m_add_artificial_areas_service;
  /*!
   * \brief Service for removing artificial areas from the grid.
   */
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_remove_artificial_areas_service;

  rclcpp::Service<vdb_mapping_interfaces::srv::ToggleRemoteSource>::SharedPtr
    m_toggle_remote_source_service;
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
   * \brief Map Frame
   */
  std::string m_map_frame;
  /*!
   * \brief Robot Frame
   */
  std::string m_robot_frame;
  /*!
   * \brief Map pointer
   */
  std::shared_ptr<VDBMappingT> m_vdb_map;
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
   * \brief Specifies whether the map should be published as a occupancy grid!
   */
  bool m_publish_occupancy_grid;
  /*!
   * \brief Specifies whether the mapping publishes map sections for remote use
   */
  bool m_publish_sections;
  /*!
   * \brief Specifies whether the mapping publishes map full sections for remote use
   */
  bool m_publish_full_sections;
  /*!
   * \brief Specifies whether the mapping applies raw sensor data
   */
  bool m_apply_raw_sensor_data;
  /*!
   * \brief Map of remote mapping source connections
   */
  std::map<std::string, std::shared_ptr<RemoteSource>> m_remote_sources;
  /*!
   * \brief Specifies whether the remote sections should be smoothed before integration
   */
  bool m_smooth_remote_sections;
  /*!
   * \brief Specifies the amount of smoothing applied to the remote sections
   */
  int m_remote_section_smoothing_iterations;
  /*!
   * \brief Timer for map visualization
   */
  rclcpp::TimerBase::SharedPtr m_visualization_timer;
  /*!
   * \brief Specifies whether the sensor data is accumulated before updating the map
   */
  bool m_accumulate_updates;
  /*!
   * \brief Timer for publishing map sections
   */
  rclcpp::TimerBase::SharedPtr m_section_timer;
  /*!
   * \brief Timer for publishing full map sections
   */
  rclcpp::TimerBase::SharedPtr m_full_section_timer;
  /*!
   * \brief Min Coordinate of the section update bounding box
   */
  Eigen::Matrix<double, 3, 1> m_section_min_coord;
  /*!
   * \brief Max Coordinate of the section update bounding box
   */
  Eigen::Matrix<double, 3, 1> m_section_max_coord;
  /*!
   * \brief Reference Frame for the section update
   */
  std::string m_section_update_frame;
  /*!
   * \brief Specifies the number of voxels which count as occupied for the occupancy grid
   */
  int m_two_dim_projection_threshold;

  /*!
   * \brief Specifies the timeout for tf lookups when inserting a scan in seconds
   */
  double m_tf_lookup_timeout;

  /*!
   * \brief Compression level used for creating the byte array message.
   */
  unsigned int m_compression_level = 1;
  std::shared_ptr<rclcpp::ParameterEventHandler> m_param_sub;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> m_z_min_param_handle;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> m_z_max_param_handle;
  double m_lower_visualization_z_limit;
  double m_upper_visualization_z_limit;

  rclcpp::CallbackGroup::SharedPtr m_accumulation_cb_group;
  rclcpp::CallbackGroup::SharedPtr m_visualization_cb_group;
  rclcpp::CallbackGroup::SharedPtr m_remote_cb_group;
};


#endif /* VDB_MAPPING_ROS22_VDBMAPPINGROS2_HPP_INCLUDED */
