import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='False')
    use_sim_time = LaunchConfiguration('use_sim_time')

    pkg_share = FindPackageShare('my_cartographer').find('my_cartographer')
    rviz_config_dir = os.path.join(pkg_share, 'rviz_config')

    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('uav_description'),
                'launch',
                'uav_urdf.launch.py'
            ])
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    lslidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lslidar_driver'),
                'launch',
                'lsn10_launch.py'
            ])  
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    """amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('my_amcl'),
                'launch',
                'my_amcl.launch.py'
            ])
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )"""

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
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
        ],
    )

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        arguments=[
            '-configuration_directory', FindPackageShare('cartographer_ros').find('cartographer_ros') + '/configuration_files',
            '-configuration_basename', 'mycartographer_2d.lua'],
        remappings=[
            ('scan', 'scan'),
            ('imu', 'imu'),
            ('odom', '/odometry/filtered')],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    imu_data_node = Node(
        package='imu',
        executable='imu_serial',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'resolution': 0.05}
        ],
    )

    rviz_node = Node(
        package='rviz2',
        namespace='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        robot_state_publisher_launch,
        rviz_node,
        cartographer_node,
        cartographer_occupancy_grid_node,
        imu_data_node,
        robot_localization_node,
        TimerAction(
            period=2.0,
            actions=[lslidar_launch]
        ),
        # amcl_launch,
    ])
