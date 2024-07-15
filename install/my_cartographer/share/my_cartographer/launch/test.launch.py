import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    ## ***** Launch arguments *****

    ## ***** File paths ******
    pkg_share = FindPackageShare('my_cartographer').find('my_cartographer')
    rviz_config_dir = os.path.join(pkg_share, 'rviz_config')
    

    ## ***** Nodes *****

    # Add robot_state_publisher node

    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('uav_description'),
                'launch',
                'uav_urdf.launch.py'
            ])
        ])
    )


    

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
                {'frequency': 30.0},
                {'sensor_timeout': 0.1},
                {'two_d_mode': True},
                {'transform_time_offset': 0.0},
                {'transform_timeout': 0.0},
                {'print_diagnostics': True},
                {'debug': False},
                {'publish_tf': True},
                {'publish_acceleration': False},
                {'map_frame': 'map'},
                {'odom_frame': 'odom'},
                {'base_link_frame': 'base_link'},
                {'world_frame': 'odom'},
                {'imu0': '/imu'},
                {'imu0_config': [False, False, False,
                                 False, False, False,
                                 True, True, False,
                                 True, True, True,
                                 False, False, False]},
                {'imu0_queue_size': 10},
                {'imu0_nodelay': False},
                {'imu0_differential': False},
                {'imu0_relative': False},
                {'imu0_remove_gravitational_acceleration': True},
            ]#,  # 注意这里使用了方括号，将字典包裹成一个列表
        #remappings=[('odom', '/odometry/filtered')]
    )

    imu_data_node = Node(
        package='imu',
        executable='imu_serial',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_launch,
        imu_data_node,
        robot_localization_node
    ])

