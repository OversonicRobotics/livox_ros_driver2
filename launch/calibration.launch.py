import os
from launch import LaunchDescription
from launch_ros.actions import Node

################### user configure parameters for ros2 start ###################
xfer_format   = 0    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
data_src      = 0    # 0-lidar, others-Invalid data src
publish_freq  = 10.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
output_type   = 0
frame_id      = 'calibration_frame'
merge_pointcloud = False

cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
cur_config_path = cur_path + '../config'
user_config_path = os.path.join(cur_config_path, 'calibration.json')
################### user configure parameters for ros2 end #####################

livox_ros2_params = [
    {"xfer_format": xfer_format},
    {"multi_topic": 1},
    {"data_src": data_src},
    {"publish_freq": publish_freq},
    {"output_data_type": output_type},
    {"frame_id": frame_id},
    {"user_config_path": user_config_path},
    {"merge_pointcloud": merge_pointcloud}
]

def generate_launch_description():
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params,
        remappings=[
            ('/livox/lidar', '/lidar_3d/pointcloud_custom_msg' if xfer_format else '/lidar_3d/pointcloud'),
            ('/livox/imu', '/lidar_3d/imu')
        ]
    )

    return LaunchDescription([
        livox_driver,
    ])
