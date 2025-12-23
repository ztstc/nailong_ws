import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter

def generate_launch_description():

    # 【关键修改1】 设置你的功能包名
    pkg_name = 'my_slam_pkg'

    # 获取功能包路径
    pkg_share = get_package_share_directory(pkg_name)

    # 定义可传入的参数，如地图文件路径
    map_yaml_file = LaunchConfiguration('map')
    declare_map_arg = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([pkg_share, 'maps', 'my_map.yaml']), # 【关键修改2】 你的地图路径
        description='Full path to map yaml file to load'
    )

    # 设置全局参数：使用仿真时间？实体机器人设为False
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # 设置参数文件路径
    params_file = LaunchConfiguration('params_file')
    declare_params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'nav2_params.yaml']), # 【关键修改3】 你的参数文件路径
        description='Full path to the Nav2 parameters file'
    )

    # 启动Nav2的核心主节点（它负责拉起所有服务器）
    nav2_launch_dir = get_package_share_directory('nav2_bringup')
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch_dir, '/launch', '/bringup_launch.py']),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
        }.items()
    )

    # （可选）启动Rviz2，方便可视化与交互
    rviz_config_file = PathJoinSubstitution([pkg_share, 'rviz', 'nav2_default_view.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        declare_map_arg,
        declare_use_sim_time_arg,
        declare_params_file_arg,
        SetParameter(name='use_sim_time', value=use_sim_time),
        nav2_bringup_launch,
        rviz_node, # 可选
    ])