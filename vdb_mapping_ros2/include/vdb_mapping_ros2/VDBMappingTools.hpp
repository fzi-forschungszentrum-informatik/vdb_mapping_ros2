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
 * \date    2022-05-09
 *
 */
//----------------------------------------------------------------------
#ifndef VDB_MAPPING_ROS2_VDBMAPPINGTOOLS_H_INCLUDED
#define VDB_MAPPING_ROS2_VDBMAPPINGTOOLS_H_INCLUDED
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <openvdb/openvdb.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>

/*!
 * \brief Collection of VDBMapping helper functions and tools
 */
template <typename VDBMappingT>
class VDBMappingTools
{
public:
  VDBMappingTools(){};
  virtual ~VDBMappingTools(){};
  /*!
   * \brief Creates output msgs for pointcloud and marker arrays
   *
   * \param grid Map grid
   * \param resolution Resolution of the grid
   * \param frame_id Frame ID of the grid
   * \param marker_msg Output Marker message
   * \param cloud_msg Output Pointcloud message
   * \param create_marker Flag specifying to create a marker message
   * \param create_pointcloud Flag specifying to create a pointcloud message
   */
  static void createMappingOutput(const typename VDBMappingT::GridT::Ptr grid,
                                  const std::string& frame_id,
                                  visualization_msgs::msg::Marker& marker_msg,
                                  sensor_msgs::msg::PointCloud2& cloud_msg,
                                  nav_msgs::msg::OccupancyGrid& occupancy_grid_msg,
                                  const bool create_marker,
                                  const bool create_pointcloud,
                                  const bool create_occupancy_grid,
                                  double lower_z_limit             = 0.0,
                                  double upper_z_limit             = 0.0,
                                  const float resolution           = 0.05,
                                  const int two_dim_proj_threshold = 5)
  {
    typename VDBMappingT::PointCloudT::Ptr cloud(new typename VDBMappingT::PointCloudT);
    openvdb::CoordBBox bbox = grid->evalActiveVoxelBoundingBox();
    double min_z, max_z;
    openvdb::Vec3d min_world_coord = grid->indexToWorld(bbox.getStart());
    openvdb::Vec3d max_world_coord = grid->indexToWorld(bbox.getEnd());
    min_z                          = min_world_coord.z();
    max_z                          = max_world_coord.z();

    if (lower_z_limit != upper_z_limit && lower_z_limit < upper_z_limit)
    {
      min_z = min_z < lower_z_limit ? lower_z_limit : min_z;
      max_z = max_z > upper_z_limit ? upper_z_limit : max_z;
    }

    std::vector<int> occ_voxel_projection_grid;
    int occ_x_offset = 0;
    int occ_y_offset = 0;
    if (create_occupancy_grid)
    {
      occupancy_grid_msg.info.height     = bbox.dim().y();
      occupancy_grid_msg.info.width      = bbox.dim().x();
      occupancy_grid_msg.info.resolution = resolution;
      occupancy_grid_msg.data.resize(occupancy_grid_msg.info.width * occupancy_grid_msg.info.height,
                                     0);
      occ_voxel_projection_grid.resize(
        occupancy_grid_msg.info.width * occupancy_grid_msg.info.height, 0);

      occ_x_offset = abs(bbox.min().x());
      occ_y_offset = abs(bbox.min().y());

      geometry_msgs::msg::Pose origin_pose;
      origin_pose.position.x = bbox.min().x() * resolution;
      origin_pose.position.y = bbox.min().y() * resolution;
      origin_pose.position.z = 0.00;

      occupancy_grid_msg.info.origin = origin_pose;
    }

    for (typename VDBMappingT::GridT::ValueOnCIter iter = grid->cbeginValueOn(); iter; ++iter)
    {
      if (create_occupancy_grid)
      {
        int vdb_index_to_occ_index = (iter.getCoord().y() + occ_y_offset) * bbox.dim().x() +
                                     (iter.getCoord().x() + occ_x_offset);
        occ_voxel_projection_grid[vdb_index_to_occ_index] += 1;
      }
      openvdb::Vec3d world_coord = grid->indexToWorld(iter.getCoord());

      if (world_coord.z() < min_z || world_coord.z() > max_z)
      {
        continue;
      }

      if (create_marker)
      {
        geometry_msgs::msg::Point cube_center;
        cube_center.x = world_coord.x();
        cube_center.y = world_coord.y();
        cube_center.z = world_coord.z();
        marker_msg.points.push_back(cube_center);
        // Calculate the relative height of each voxel.
        double h = (1.0 - ((world_coord.z() - min_z) / (max_z - min_z)));
        marker_msg.colors.push_back(heightColorCoding(h));
      }
      if (create_pointcloud)
      {
        cloud->points.push_back(
          typename VDBMappingT::PointT(world_coord.x(), world_coord.y(), world_coord.z()));
      }
    }
    if (create_marker)
    {
      double size                = grid->transform().voxelSize()[0];
      marker_msg.header.frame_id = frame_id;
      // marker_msg.header.stamp       = ros::Time::now();
      marker_msg.id                 = 0;
      marker_msg.type               = visualization_msgs::msg::Marker::CUBE_LIST;
      marker_msg.scale.x            = size;
      marker_msg.scale.y            = size;
      marker_msg.scale.z            = size;
      marker_msg.color.a            = 1.0;
      marker_msg.pose.orientation.w = 1.0;
      marker_msg.frame_locked       = true;
      if (marker_msg.points.size() > 0)
      {
        marker_msg.action = visualization_msgs::msg::Marker::ADD;
      }
      else
      {
        marker_msg.action = visualization_msgs::msg::Marker::DELETE;
      }
    }
    if (create_pointcloud)
    {
      cloud->width  = cloud->points.size();
      cloud->height = 1;
      pcl::toROSMsg(*cloud, cloud_msg);
      cloud_msg.header.frame_id = frame_id;
      // cloud_msg.header.stamp    = ros::Time::now();
    }

    if (create_occupancy_grid)
    {
      for (size_t i = 0; i < occ_voxel_projection_grid.size(); i++)
      {
        if (occ_voxel_projection_grid[i] > two_dim_proj_threshold)
        {
          occupancy_grid_msg.data[i] = 100;
        }
        else if (occ_voxel_projection_grid[i] == 0)
        {
          occupancy_grid_msg.data[i] = -1;
        }
        else
        {
          occupancy_grid_msg.data[i] = 0;
        }
      }
    }
  }
  /*!
   * \brief Calculates a height correlating color coding using HSV color space
   *
   * \param height Gridcell height relativ to the min and max height of the complete grid. Parameter
   * can take values between 0 and 1
   *
   * \returns RGBA color of the grid cell
   */
  static std_msgs::msg::ColorRGBA heightColorCoding(const double height)
  {
    // The factor of 0.8 is only for a nicer color range
    double h = height * 0.8;
    int i    = (int)(h * 6.0);
    double f = (h * 6.0) - i;
    double q = (1.0 - f);
    i %= 6;
    auto toMsg = [](double v1, double v2, double v3) {
      std_msgs::msg::ColorRGBA rgba;
      rgba.a = 1.0;
      rgba.r = v1;
      rgba.g = v2;
      rgba.b = v3;
      return rgba;
    };
    switch (i)
    {
      case 0:
        return toMsg(1.0, f, 0.0);
        break;
      case 1:
        return toMsg(q, 1.0, 0.0);
        break;
      case 2:
        return toMsg(0.0, 1.0, f);
        break;
      case 3:
        return toMsg(0.0, q, 1.0);
        break;
      case 4:
        return toMsg(f, 0.0, 1.0);
        break;
      case 5:
        return toMsg(1.0, 0.0, q);
        break;
      default:
        return toMsg(1.0, 0.5, 0.5);
        break;
    }
  }
};
#endif /* VDB_MAPPING_ROS2_VDBMAPPINGTOOLS_H_INCLUDED */
