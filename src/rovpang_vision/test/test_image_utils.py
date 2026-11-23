"""image_utils 시험 (ROS 없이): colcon test --packages-select rovpang_vision"""
import numpy as np
import pytest

from rovpang_vision.image_utils import from_bgr, to_bgr


def make_rgb():
    img = np.zeros((2, 3, 3), dtype=np.uint8)
    img[0, 0] = (255, 0, 0)      # RGB 빨강
    return img


def test_rgb8_becomes_bgr():
    rgb = make_rgb()
    bgr = to_bgr('rgb8', 2, 3, 9, rgb.tobytes())
    assert tuple(bgr[0, 0]) == (0, 0, 255)


def test_bgr8_unchanged():
    img = make_rgb()
    out = to_bgr('bgr8', 2, 3, 9, img.tobytes())
    assert np.array_equal(out, img)


def test_row_padding():
    img = make_rgb()
    padded = b''.join(row.tobytes() + b'\x00' * 3 for row in img)   # 한 줄 끝에 빈 바이트 3개
    out = to_bgr('bgr8', 2, 3, 12, padded)
    assert np.array_equal(out, img)


def test_round_trip():
    img = np.random.randint(0, 255, (4, 5, 3), dtype=np.uint8)
    enc, h, w, step, data = from_bgr(img)
    assert (enc, h, w, step) == ('bgr8', 4, 5, 15)
    assert np.array_equal(to_bgr(enc, h, w, step, data), img)


def test_unsupported_encoding():
    with pytest.raises(ValueError):
        to_bgr('mono8', 1, 1, 1, b'\x00')
