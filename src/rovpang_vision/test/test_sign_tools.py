"""표지판 찾기 시험 (ROS 없이): 만든 표지판 그림을 작게·기울여 장면에 붙여도 찾는지, 다른 모양은 무시하는지"""
import cv2
import numpy as np

from rovpang_vision.sign_tools import best_sign, find_signs, make_sign_image, sign_offset


def scene_with(sign_img, size=120, angle=0, at=(260, 120)):
    frame = np.full((360, 640, 3), (190, 190, 190), np.uint8)       # 회색 배경
    small = cv2.resize(sign_img, (size, size), interpolation=cv2.INTER_AREA)
    if angle:
        M = cv2.getRotationMatrix2D((size / 2, size / 2), angle, 1.0)
        small = cv2.warpAffine(small, M, (size, size), borderValue=(255, 255, 255))
    x, y = at
    frame[y:y + size, x:x + size] = small
    return frame


def test_sign_a_red_triangle():
    label, conf = best_sign(scene_with(make_sign_image('A')))
    assert label == 'A' and conf > 0.8


def test_sign_b_blue_circle():
    label, conf = best_sign(scene_with(make_sign_image('B')))
    assert label == 'B' and conf > 0.8


def test_small_and_tilted():
    assert best_sign(scene_with(make_sign_image('A'), size=70, angle=12))[0] == 'A'
    assert best_sign(scene_with(make_sign_image('B'), size=70, angle=12))[0] == 'B'


def test_ignores_red_square_and_blue_rectangle():
    frame = np.full((360, 640, 3), (190, 190, 190), np.uint8)
    cv2.rectangle(frame, (50, 50), (170, 170), (0, 0, 220), -1)       # 빨간 사각형 (소화기 같은 것)
    cv2.rectangle(frame, (300, 100), (500, 160), (220, 60, 0), -1)    # 파란 긴 사각형 (분무기 라벨 같은 것)
    assert find_signs(frame) == []


def test_empty_scene():
    assert best_sign(np.full((360, 640, 3), 190, np.uint8)) == ('', 0.0)


def test_bigger_sign_wins():
    frame = scene_with(make_sign_image('B'), size=60, at=(40, 40))
    small_b = frame.copy()
    big_a = cv2.resize(make_sign_image('A'), (150, 150), interpolation=cv2.INTER_AREA)
    small_b[150:300, 400:550] = big_a
    assert best_sign(small_b)[0] == 'A'


K = (359.1, 359.1, 320.5, 178.8)   # 11/24 카메라 보정값 (640x360)


def test_offset_center():
    off, ang = sign_offset((300, 100, 40, 40), 640, K)       # 중심 u=320
    assert abs(off) < 0.01 and abs(ang) < 0.01


def test_offset_right_and_left():
    off_r, ang_r = sign_offset((560, 100, 80, 80), 640, K)   # 오른쪽 끝 (u=600)
    off_l, ang_l = sign_offset((0, 100, 80, 80), 640, K)     # 왼쪽 끝 (u=40)
    assert off_r > 0.8 and ang_r > 0.6     # 약 38도
    assert off_l < -0.8 and ang_l < -0.6


def test_offset_without_camera_info():
    off, ang = sign_offset((560, 100, 80, 80), 640)
    assert off > 0.8 and ang == 0.0


def test_offset_from_found_sign():
    frame = scene_with(make_sign_image('B'), at=(500, 120))  # 오른쪽에 붙임
    label, _, box, _ = find_signs(frame)[0]
    off, _ = sign_offset(box, frame.shape[1], K)
    assert label == 'B' and off > 0.5
