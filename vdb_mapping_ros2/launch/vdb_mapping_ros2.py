from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    vdb_mapping = Node(
    package='vdb_mapping_ros2',
    # namespace='bla',
    executable='vdb_mapping_ros_node',
    name='vdb_map_blub',
    parameters=[
        {"aligned_points": "scan_matched_points2"},
        {"raw_points": "raw_points"},
        {"sensor_frame": "velodyne"},
        {"map_frame": "map"},
        {"max_range": 10.0},
        {"resolution": 0.07},
        {"prob_hit": 0.8},
        {"prob_miss": 0.1},
        {"prob_thres_min": 0.12},
        {"prob_thres_max": 0.8},
        {"publish_pointcloud": True},
        {"publish_vis_marker": True},
        {"publish_updates": True},
        {"map_directory_path": ""},
        {"remote_mode": False}
        ]
    )

    ld.add_action(vdb_mapping)


    return ld
