# my_robot_cartographer.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 1. 获取你的功能包的路径
    pkg_share = get_package_share_directory('navi') # 【重要】修改为你的包名

    # 2. 定义可传入的参数，例如配置文件名称
    configuration_basename = LaunchConfiguration('configuration_basename')
    declare_configuration_basename = DeclareLaunchArgument(
        'configuration_basename',
        default_value='my_trajectory_builder_2d.lua', # 【重要】你的Lua配置文件名
        description='Name of lua configuration file for cartographer'
    )

    # 3. 启动Cartographer节点
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': False}], # 实体机器人设为False
        arguments=[
            '-configuration_directory', os.path.join(pkg_share, 'config'), # 【重要】指向你的config文件夹
            '-configuration_basename', configuration_basename
        ],
        # 可以重映射话题（如果你的话题名不是标准的）
        remappings=[
            ('scan', '/scan'),             # 假设你的雷达话题是 /scan
            ('odom', '/odom'),             # 假设你的里程计话题是 /odom
            ('imu', '/imu')                # 如果需要，映射IMU话题
        ]
    )

    # 4. 启动Cartographer的occupancy_grid_node（将子图转换为可用的地图）
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'] # 地图分辨率和发布周期
    )

    # 5. （可选但推荐）启动静态TF广播，如果你的机器人URDF没发布 base_link->laser_link
    # 假设雷达坐标系叫 `laser_link`， 相对于底盘 `base_link` 有0.2米的高度
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0.2', '0', '0', '0', 'base_link', 'laser_link'] # x, y, z, roll, pitch, yaw, parent, child
    )

    return LaunchDescription([
        declare_configuration_basename,
        cartographer_node,
        occupancy_grid_node,
        static_tf_node, # 根据你的TF树情况决定是否添加
    ])