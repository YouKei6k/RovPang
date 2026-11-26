"""배송지 표지판 찾기 (F-09): 색 + 모양으로 판단 (OpenCV, 학습 없음, ROS 없이 시험 가능)

  배송지 A = 빨간 삼각형, 배송지 B = 파란 원

방법
  1. HSV(색상·채도·밝기)로 바꿔 빨간색, 파란색 영역만 골라낸다. (빨강은 색상 값이 0 근처와 180 근처 두 군데)
  2. 각 영역의 외곽선을 찾고, 너무 작은 것은 버린다.
  3. 빨강: 외곽선을 단순화했을 때 꼭짓점이 3개이고 삼각형을 꽉 채우면 A
     파랑: 원형도(4π·넓이/둘레²)가 1에 가깝고 둘러싼 원을 꽉 채우면 B
  4. 신뢰도 = 모양이 얼마나 맞는지(0~1). 여러 개면 가장 큰 것.
"""
import cv2
import numpy as np

# HSV 범위 (OpenCV: H 0~180, S·V 0~255). 조명에 따라 조정 가능
RED_RANGES = [((0, 100, 70), (10, 255, 255)), ((170, 100, 70), (180, 255, 255))]
BLUE_RANGES = [((100, 120, 60), (130, 255, 255))]


def color_mask(hsv, ranges):
    mask = None
    for lo, hi in ranges:
        m = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
        mask = m if mask is None else cv2.bitwise_or(mask, m)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)     # 작은 점 지우기
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)    # 작은 구멍 메우기
    return mask


def triangle_score(contour):
    """삼각형다움 0~1: 꼭짓점 3개 + 외곽선이 최소 외접 삼각형을 얼마나 채우는지"""
    peri = cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, 0.04 * peri, True)
    if len(approx) != 3:
        return 0.0
    area = cv2.contourArea(contour)
    tri_area, _ = cv2.minEnclosingTriangle(contour)
    return float(area / tri_area) if tri_area > 0 else 0.0


def circle_score(contour):
    """원다움 0~1: 원형도(4π·넓이/둘레²)와 외접원을 채우는 정도 중 작은 값"""
    area = cv2.contourArea(contour)
    peri = cv2.arcLength(contour, True)
    if peri == 0:
        return 0.0
    circularity = 4 * np.pi * area / (peri * peri)
    (_, _), r = cv2.minEnclosingCircle(contour)
    fill = area / (np.pi * r * r) if r > 0 else 0.0
    return float(min(circularity, fill, 1.0))


def find_signs(bgr, min_area=400, min_score=0.75):
    """영상에서 표지판 후보를 찾는다. 반환: [(라벨 'A'/'B', 신뢰도, (x, y, w, h), 넓이)] 넓이가 큰 순서"""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    found = []
    for label, ranges, scorer in (('A', RED_RANGES, triangle_score), ('B', BLUE_RANGES, circle_score)):
        contours, _ = cv2.findContours(color_mask(hsv, ranges), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            area = cv2.contourArea(c)
            if area < min_area:
                continue
            score = scorer(c)
            if score >= min_score:
                found.append((label, score, cv2.boundingRect(c), area))
    found.sort(key=lambda f: f[3], reverse=True)
    return found


def best_sign(bgr, min_area=400, min_score=0.75):
    """가장 큰 표지판 하나: (라벨, 신뢰도) 또는 ('', 0.0)"""
    found = find_signs(bgr, min_area, min_score)
    if not found:
        return '', 0.0
    return found[0][0], found[0][1]


def sign_offset(box, width, K=None):
    """표지판 상자 (x, y, w, h)가 화면 가운데에서 가로로 얼마나 벗어났는지.
    반환: (offset_x, angle)
      offset_x: -1 왼쪽 끝 ~ 0 가운데 ~ +1 오른쪽 끝 (화면 폭 기준)
      angle   : 카메라 정면 기준 가로 각도(rad), +는 오른쪽. K=(fx, fy, cx, cy) 가 없으면 0
    도착 후 정렬(팀원 2)에서 '표지판이 정면에 오도록 회전'할 때 쓴다."""
    x, _, w, _ = box
    u = x + w / 2.0
    half = width / 2.0
    offset = (u - half) / half
    angle = 0.0
    if K is not None:
        fx, _, cx, _ = K
        angle = float(np.arctan2(u - cx, fx))
    return float(max(-1.0, min(1.0, offset))), angle


def make_sign_image(label, size=600):
    """인쇄·화면 표시용 표지판 그림. A = 빨간 삼각형, B = 파란 원 (흰 배경, 아래에 글자)"""
    img = np.full((size, size, 3), 255, np.uint8)
    s = size
    if label == 'A':
        pts = np.array([[s // 2, int(s * 0.10)], [int(s * 0.10), int(s * 0.78)], [int(s * 0.90), int(s * 0.78)]], np.int32)
        cv2.fillPoly(img, [pts], (0, 0, 220))        # BGR 빨강
    elif label == 'B':
        cv2.circle(img, (s // 2, int(s * 0.44)), int(s * 0.34), (220, 60, 0), -1)   # BGR 파랑
    else:
        raise ValueError('label 은 A 또는 B')
    cv2.putText(img, label, (int(s * 0.46), int(s * 0.95)), cv2.FONT_HERSHEY_SIMPLEX, s / 400, (0, 0, 0), max(2, s // 150))
    return img
