"""물품 위치 계산 (F-03): 탐지 상자 + 깊이 영상 + 카메라 보정값 -> 카메라 기준 3차원 위치 (ROS 없이 시험 가능)

전제: 카메라를 depth_registration:=true 로 켜서 깊이 영상이 컬러 영상 좌표에 맞춰져 있어야 한다.
그래야 YOLO가 컬러 영상에서 찾은 픽셀 (u, v)의 깊이를 같은 (u, v)에서 읽을 수 있다.

좌표계: 카메라 광학 좌표계(depth_cam_color_optical_frame)
  X = 오른쪽, Y = 아래쪽, Z = 카메라 앞쪽 (단위 m)
핀홀 모델:  X = (u - cx) * Z / fx,  Y = (v - cy) * Z / fy
"""
import numpy as np


def depth_to_meters(encoding, height, width, step, data):
    """깊이 영상 메시지 필드 -> 미터 단위 float32 배열. 16UC1 은 mm, 32FC1 은 m. 값 0 또는 NaN = 깊이 없음."""
    enc = encoding.upper()
    buf = np.frombuffer(bytes(data), dtype=np.uint8)
    if enc in ('16UC1', 'MONO16'):
        img = np.ndarray((height, width), dtype='<u2', buffer=buf, strides=(step, 2))
        return img.astype(np.float32) / 1000.0
    if enc == '32FC1':
        img = np.ndarray((height, width), dtype='<f4', buffer=buf, strides=(step, 4))
        return np.nan_to_num(img.astype(np.float32), nan=0.0)
    raise ValueError('지원하지 않는 깊이 형식: {}'.format(encoding))


def median_depth(depth_m, u, v, radius=5, min_valid=0.3):
    """(u, v) 주변 (2*radius+1) 정사각형 안에서 0이 아닌 깊이의 중앙값(m).
    유효한 픽셀 비율이 min_valid 보다 적으면 None (반사·너무 가까움 등으로 깊이를 못 잰 경우)."""
    h, w = depth_m.shape
    if not (0 <= u < w and 0 <= v < h):
        return None
    patch = depth_m[max(0, v - radius):v + radius + 1, max(0, u - radius):u + radius + 1]
    valid = patch[patch > 0]
    if valid.size == 0 or valid.size < min_valid * patch.size:
        return None
    return float(np.median(valid))


def correct_depth(z, gain=1.0, bias=0.0):
    """측정 깊이 -> 실제 깊이 보정.  측정값 = gain * 실제 + bias  이므로  실제 = (측정 - bias) / gain
    11/24 자 측정(평평한 물체, 화면 가운데): 30cm->32.9, 50cm->54.3, 80cm->86.3 -> gain 1.068, bias 0.0088 m"""
    return (z - bias) / gain


def pixel_to_point(u, v, z, fx, fy, cx, cy):
    """픽셀 (u, v)와 깊이 z(m) -> 카메라 기준 (X, Y, Z) m"""
    return ((u - cx) * z / fx, (v - cy) * z / fy, z)


def box_size_m(x1, y1, x2, y2, z, fx, fy):
    """상자의 실제 가로·세로 크기(m) 추정. 물품 크기와 크게 다르면 오탐으로 볼 수 있다."""
    return ((x2 - x1) * z / fx, (y2 - y1) * z / fy)


def locate(box, depth_m, K, radius=5, gain=1.0, bias=0.0):
    """상자 (x1, y1, x2, y2) 의 중심 깊이로 3차원 위치를 계산한다.
    K = (fx, fy, cx, cy). 깊이 영상 크기가 컬러와 다르면 비율로 맞춘 좌표를 넘겨야 한다.
    반환: ((X, Y, Z), (가로 m, 세로 m)) 또는 깊이를 못 재면 None"""
    x1, y1, x2, y2 = box
    u, v = (x1 + x2) // 2, (y1 + y2) // 2
    z = median_depth(depth_m, u, v, radius)
    if z is None:
        return None
    z = correct_depth(z, gain, bias)
    fx, fy, cx, cy = K
    return pixel_to_point(u, v, z, fx, fy, cx, cy), box_size_m(x1, y1, x2, y2, z, fx, fy)
