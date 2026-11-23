"""QR 만들기·읽기 시험 (ROS 없이): 카메라 영상 크기(640x360)에 작게 붙여도 읽히는지"""
import cv2
import numpy as np

from rovpang_vision.qr_tools import QrReader, make_image


def place_on_frame(qr, width=640, height=360, size=120):
    frame = np.full((height, width, 3), 180, dtype=np.uint8)
    small = cv2.resize(qr, (size, size), interpolation=cv2.INTER_AREA)
    y, x = (height - size) // 2, (width - size) // 2
    frame[y:y + size, x:x + size] = cv2.cvtColor(small, cv2.COLOR_GRAY2BGR)
    return frame


def test_round_trip_token():
    frame = place_on_frame(make_image('a3f09c1e'))
    texts = [t for t, _ in QrReader().decode(frame)]
    assert texts == ['a3f09c1e']


def test_no_qr():
    frame = np.full((360, 640, 3), 180, dtype=np.uint8)
    assert QrReader().decode(frame) == []


def test_make_image_has_white_border():
    img = make_image('x', module_px=10, border_modules=4)
    assert img[0, 0] == 255 and img.shape[0] == img.shape[1]
