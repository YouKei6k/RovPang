"""QR 읽기·만들기 (OpenCV만 사용, ROS 없이 시험 가능)

qr_detector 노드가 decode() 를, 시험용 QR 그림을 만들 때 make_image() 를 쓴다.
QR 안에는 배송별 일회용 토큰 문자열(예: 'a3f09c1e')만 들어간다 (task_manager의 recipient_auth가 비교).
"""
import cv2
import numpy as np


class QrReader:
    """QR 읽기. 기본 검출기로 못 읽으면 ArUco 기반 검출기(OpenCV 4.8 이상)로 한 번 더 읽는다.
    ArUco 기반은 작게 찍힌 QR(약 100픽셀)도 더 잘 읽는다 (11/23 시험: 기본 120픽셀부터, ArUco 100픽셀부터)."""

    def __init__(self):
        self.detectors = [cv2.QRCodeDetector()]
        if hasattr(cv2, 'QRCodeDetectorAruco'):
            self.detectors.append(cv2.QRCodeDetectorAruco())

    def decode(self, bgr):
        """영상에서 QR 하나를 읽어 [(문자열, 네 꼭짓점)] 으로 돌려준다. 못 읽으면 빈 목록."""
        for det in self.detectors:
            try:
                text, points, _ = det.detectAndDecode(bgr)
            except cv2.error:
                continue
            if text and points is not None:
                return [(text, points.reshape(-1, 2).astype(int).tolist())]
        return []


def make_image(text, module_px=10, border_modules=4):
    """text 를 담은 QR 그림(흑백 numpy 배열)을 만든다. 칸 하나가 module_px 픽셀, 테두리는 흰 칸 4개."""
    qr = cv2.QRCodeEncoder.create().encode(text)          # 칸 하나 = 1픽셀 (0 검정, 255 흰색)
    qr = cv2.resize(qr, None, fx=module_px, fy=module_px, interpolation=cv2.INTER_NEAREST)
    pad = border_modules * module_px
    return np.pad(qr, pad, constant_values=255)
