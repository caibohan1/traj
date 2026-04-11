# 文件路径: src/【功能包】/launch/controller.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 获取 YAML 配置文件的绝对路径
    config_file = os.path.join(
        get_package_share_directory('circle'),
        'config',
        'params.yaml'
    )

    # 2. 定义要启动的节点，并挂载参数文件
    controller_node = Node(
        package='circle',
        executable='controller_node', # CMakeLists 中 add_executable 定义的名字
        name='controller_node',                      # 必须与 yaml 文件顶层的名称完全一致
        output='screen',
        parameters=[config_file]             # 核心：将 yaml 传入节点
    )

    return LaunchDescription([
        controller_node
    ])