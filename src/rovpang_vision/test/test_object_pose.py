"""object_pose 시험 (ROS 없이)"""
import numpy as np

from rovpang_vision.object_pose import correct_depth, depth_to_meters, locate, median_depth, pixel_to_point

K = (460.0, 460.0, 320.0, 180.0)   # fx, fy, cx, cy (640x360 영상 예시)


def test_center_pixel_is_on_axis():
    assert pixel_to_point(320, 180, 0.5, *K) == (0.0, 0.0, 0.5)


def test_right_and_down_are_positive():
    x, y, z = pixel_to_point(320 + 46, 180 + 92, 0.5, *K)
    assert abs(x - 0.05) < 1e-9 and abs(y - 0.10) < 1e-9 and z == 0.5


def test_16uc1_mm_to_m():
    raw = np.full((2, 3), 400, dtype='<u2')
    d = depth_to_meters('16UC1', 2, 3, 6, raw.tobytes())
    assert d.dtype == np.float32 and abs(d[0, 0] - 0.4) < 1e-6


def test_median_ignores_zero_holes():
    d = np.full((20, 20), 0.5, dtype=np.float32)
    d[8:12, 8:12] = 0.0               # 가운데 구멍 (깊이 없음)
    assert median_depth(d, 10, 10, radius=5) == 0.5


def test_too_few_valid_pixels_returns_none():
    d = np.zeros((20, 20), dtype=np.float32)
    d[10, 10] = 0.5
    assert median_depth(d, 10, 10, radius=5) is None


def test_locate_buds_box():
    d = np.zeros((360, 640), dtype=np.float32)
    d[250:320, 420:490] = 0.40       # 버즈가 있는 곳 40cm
    (x, y, z), (w, h) = locate((423, 252, 487, 313), d, K)
    assert abs(z - 0.40) < 1e-6
    assert x > 0 and y > 0           # 화면 오른쪽 아래 = X+, Y+
    assert 0.04 < w < 0.07           # 64픽셀 x 0.4m / 460 = 약 5.6cm


def test_correct_depth_with_1124_ruler_values():
    for true_cm, measured_cm in ((30, 32.9), (50, 54.3), (80, 86.3)):
        z = correct_depth(measured_cm / 100.0, 1.068, 0.0088)
        assert abs(z * 100 - true_cm) < 0.5


def test_no_correction_by_default():
    assert correct_depth(0.5) == 0.5
