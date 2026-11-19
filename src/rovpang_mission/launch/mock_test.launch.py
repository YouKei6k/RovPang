"""가짜 로봇 + 가짜 비전 + 미션 관리 노드를 한 번에 실행 (로봇은 움직이지 않음).

  ros2 launch rovpang_mission mock_test.launch.py
  ros2 launch rovpang_mission mock_test.launch.py fail_on:=pick
  ros2 launch rovpang_mission mock_test.launch.py wrong_sign:=true
  ros2 launch rovpang_mission mock_test.launch.py wrong_qr:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    fail_on = LaunchConfiguration('fail_on')
    wrong_sign = LaunchConfiguration('wrong_sign')
    wrong_qr = LaunchConfiguration('wrong_qr')
    return LaunchDescription([
        DeclareLaunchArgument('fail_on', default_value='',
                              description='pick / place / go_to / arm 중 하나를 실패시키기'),
        DeclareLaunchArgument('wrong_sign', default_value='false'),
        DeclareLaunchArgument('wrong_qr', default_value='false'),
        Node(package='rovpang_mission', executable='mock_robot', name='mock_robot',
             output='screen', parameters=[{'fail_on': fail_on, 'delay': 2.0}]),
        Node(package='rovpang_mission', executable='mock_vision', name='mock_vision',
             output='screen', parameters=[{'wrong_sign': wrong_sign, 'wrong_qr': wrong_qr}]),
        Node(package='rovpang_mission', executable='mission_manager', name='mission_manager',
             output='screen'),
    ])
