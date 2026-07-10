import os
from launch import LaunchDescription
from launch_ros.actions import Node

################### user configure parameters for ros2 start ###################
xfer_format   = 1    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
multi_topic   = 0    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
                     # if 1, topic names will be /livox/lidar_<ip_address> and /livox/imu_<ip_address>
data_src      = 0    # 0-lidar, others-Invalid data src
publish_freq  = 10.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
output_type   = 0
merge_pointcloud = False

cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
cur_config_path = cur_path + '../config'
user_config_path = os.path.join(cur_config_path, 'MID360_PI_config.json')
################### user configure parameters for ros2 end #####################

livox_ros2_params = [
    {"xfer_format": xfer_format},
    {"multi_topic": multi_topic},
    {"data_src": data_src},
    {"publish_freq": publish_freq},
    {"output_data_type": output_type},
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
