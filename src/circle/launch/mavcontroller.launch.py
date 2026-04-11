import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # ==============================================================
    # 1. 拼接 YAML 文件的绝对路径
    # ==============================================================
    config_file_path = os.path.join(
        get_package_share_directory('circle'),
        'config',
        'params.yaml'
    )

    # ==============================================================
    # 2. 配置要启动的 Node (节点)
    # ==============================================================
    mavController = Node(
        package='circle',
        executable='mavController',
        name='mavgeometric_controller',
        output='screen',                   # 将终端日志输出到屏幕
        parameters=[config_file_path]      # 将上方的 YAML 文件喂给节点
    )
    # ==============================================================
    # 3. 生成并返回启动描述符
    # ==============================================================
    return LaunchDescription([
        mavController,
        # 如果你以后有别的节点想一起启动，直接在下面接着加：
        # other_node,
    ])