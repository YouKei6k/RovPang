"""큰 상자 거르기 시험 (11/23 실시간 시험의 모니터 오탐 값 사용)"""
import sys
import types

# object_detector 는 ROS 모듈을 불러오므로, 시험에서는 빈 모듈로 대신한다
for name in ['rclpy', 'rclpy.node', 'rclpy.qos', 'sensor_msgs', 'sensor_msgs.msg',
             'rovpang_interfaces', 'rovpang_interfaces.msg', 'onnxruntime']:
    sys.modules.setdefault(name, types.ModuleType(name))
sys.modules['rclpy.node'].Node = object
sys.modules['rclpy.qos'].qos_profile_sensor_data = None
sys.modules['rclpy.qos'].QoSProfile = object
sys.modules['rclpy.qos'].ReliabilityPolicy = object
sys.modules['sensor_msgs.msg'].Image = object
sys.modules['sensor_msgs.msg'].CameraInfo = object
sys.modules['rovpang_interfaces.msg'].Detection = object
sys.modules['rovpang_interfaces.msg'].DetectionArray = object
sys.modules['rovpang_interfaces.msg'].DeliveryStatus = object

from rovpang_vision.object_detector import filter_by_size, filter_large_boxes  # noqa: E402

BUDS = ('buds', 0.96, 423, 252, 487, 313)
MONITOR = ('buds', 0.59, 0, 0, 518, 359)


def test_monitor_removed_buds_kept():
    assert filter_large_boxes([BUDS, MONITOR], 640, 360, 0.25) == [BUDS]


def test_ratio_one_keeps_everything():
    assert filter_large_boxes([BUDS, MONITOR], 640, 360, 1.0) == [BUDS, MONITOR]


def test_size_filter_removes_too_big_keeps_unknown():
    big = ('buds', 0.55, 0, 0, 100, 80)
    pos_buds = ((0.0, 0.0, 0.30), (0.051, 0.050))     # 11/24: 30cm, 5.1x5.0cm
    pos_big = ((0.0, 0.0, 0.43), (0.130, 0.095))      # 11/24: 13.0x9.5cm 오탐
    r, p = filter_by_size([BUDS, big, BUDS], [pos_buds, pos_big, None], 0.03, 0.09)
    assert r == [BUDS, BUDS] and p == [pos_buds, None]
