from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
def generate_launch_description():
    nav2_yaml = os.path.join(get_package_share_directory('my_amcl'), 'config', 'amcl_config.yaml')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='False')
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_yaml]  # 直接传递文件路径
    )

    lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_mapper',
        output='screen',
        parameters=[{'use_sim_time': False},
                    {'autostart': True},
                    {'node_names': ['amcl']},
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}
                    ]  # 这里是多个字典
    )

    return LaunchDescription([amcl_node, lifecycle_node,use_sim_time_arg])
