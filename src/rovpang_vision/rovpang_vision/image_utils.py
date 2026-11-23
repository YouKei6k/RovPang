"""ROS 영상 메시지 <-> OpenCV(numpy) 변환 (cv_bridge 없이, ROS 없이 시험 가능)

카메라(/depth_cam/rgb/image_raw)는 rgb8 또는 bgr8 로 들어온다. YOLO 탐지기는 BGR을 받는다.
"""
import numpy as np


def to_bgr(encoding, height, width, step, data):
    """영상 메시지 필드 -> BGR numpy 배열 (height, width, 3). 지원하지 않는 형식이면 ValueError."""
    enc = encoding.lower()
    if enc not in ('rgb8', 'bgr8'):
        raise ValueError('지원하지 않는 영상 형식: {}'.format(encoding))
    buf = np.frombuffer(bytes(data), dtype=np.uint8)
    img = np.ndarray((height, width, 3), dtype=np.uint8, buffer=buf, strides=(step, 3, 1))
    if enc == 'rgb8':
        img = img[:, :, ::-1]
    return np.ascontiguousarray(img)


def from_bgr(img):
    """BGR numpy 배열 -> (encoding, height, width, step, data) : sensor_msgs/Image 필드"""
    img = np.ascontiguousarray(img, dtype=np.uint8)
    h, w = img.shape[:2]
    return 'bgr8', h, w, w * 3, img.tobytes()
