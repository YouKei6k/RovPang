"""팀원 1 영상 노드 3개를 한 번에 실행: object_detector(F-02·F-03), sign_detector(F-09), qr_detector(F-10)

  ros2 launch rovpang_vision vision.launch.py
  ros2 launch rovpang_vision vision.launch.py gate_by_status:=false   # 배송 단계와 상관없이 항상 처리

카메라는 따로 켠다: ros2 launch peripherals depth_camera.launch.py depth_registration:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    gate = ParameterValue(LaunchConfiguration('gate_by_status'), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument('gate_by_status', default_value='true',
                              description='true 면 필요한 배송 단계에서만 처리 (CPU 절약)'),
        Node(package='rovpang_vision', executable='object_detector', name='object_detector',
             output='screen', parameters=[{'gate_by_status': gate}]),
        Node(package='rovpang_vision', executable='sign_detector', name='sign_detector',
             output='screen', parameters=[{'gate_by_status': gate}]),
        Node(package='rovpang_vision', executable='qr_detector', name='qr_detector',
             output='screen', parameters=[{'gate_by_status': gate}]),
    ])
