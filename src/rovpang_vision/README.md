# rovpang_vision (팀원 1)

Vision AI: 물품 탐지(F-02), 위치 계산(F-03), QR 인식(F-10), 배송지 표지판 인식(F-09).

| 파일 | 역할 |
| --- | --- |
| object_detector | 카메라 컬러 영상을 YOLO(ONNX)로 탐지하고, 정렬된 깊이로 3차원 위치를 계산해 /rovpang/detections 발행 |
| (모듈) object_pose | 상자 중심 주변 깊이 중앙값 + 카메라 보정값(핀홀) → 카메라 기준 X·Y·Z(m), 실제 크기 추정 |
| qr_detector | 카메라 영상에서 QR을 읽어 /rovpang/qr (std_msgs/String) 발행. 인증 판단은 task_manager(recipient_auth) |
| make_qr | 시험용 QR 그림 만들기 (`--text 토큰`) |
| sign_detector | 컬러 영상에서 빨간 삼각형(A)·파란 원(B) 표지판을 찾아 /rovpang/sign 발행 (색+모양, 학습 없음) |
| make_sign | 시험·인쇄용 표지판 그림 sign_A.png, sign_B.png |
| (모듈) state_gate | /rovpang/status 를 보고 필요한 배송 단계에서만 처리 (CPU 절약). 탐지: DETECTING·PICKING·PLACING, 표지판·QR: VERIFYING. task_manager가 없으면 항상 처리 |
| (모듈) sign_tools | HSV 색 영역 → 외곽선 → 삼각형/원 모양 점수 |
| (모듈) qr_tools | QR 읽기(기본 검출기 → 못 읽으면 ArUco 기반 검출기), QR 그림 만들기 |
| (모듈) yolo_onnx | YOLOv8 ONNX 탐지기. onnxruntime CPU로 실행 (ultralytics·GPU 불필요) |
| (모듈) image_utils | ROS 영상 <-> OpenCV 변환 (cv_bridge 없이) |

## 모델
- Colab에서 best.pt를 ONNX로 변환: `YOLO('best.pt').export(format='onnx', imgsz=320, opset=12, simplify=True)`
- 로봇 위치: `/home/ubuntu/share/tmp/rovpang_models/best_320.onnx` (본체 `~/docker/tmp/rovpang_models`, GitHub에는 올리지 않음)
- Jetson Nano CPU 측정(11/23): 320 → 132 ms/장(7.6 FPS), 640 → 483 ms/장(2.1 FPS, 모니터 오탐 0.57) → 320 사용

## 실행
`ros2 launch rovpang_vision vision.launch.py` : object_detector + sign_detector + qr_detector 한 번에 (gate_by_status 기본 true)

```bash
ros2 launch peripherals depth_camera.launch.py depth_registration:=true   # 창 1: 카메라
ros2 run rovpang_vision object_detector                                     # 창 2
ros2 topic echo /rovpang/detections                                         # 창 3
```
파라미터: model_path, image_topic, conf(0.5), rate(3.0), threads(4), publish_image(true), max_box_ratio(0.25),
use_depth(true), depth_topic, info_topic(/depth_cam/rgb/camera_info), depth_radius(5), max_depth_dt(0.2)

위치는 카메라 광학 좌표계(X 오른쪽, Y 아래, Z 앞, m). 카메라는 반드시 `depth_registration:=true`로 켠다.

## 실시간 시험 (11/23, 카메라 실행 중)
- 버즈 0.95~0.97, 상자 약 64x62 픽셀 (30~50cm), frame_id = depth_cam_color_optical_frame
- 처리 시간 290~440 ms/장, 10초에 22~26장 (카메라 드라이버와 CPU를 나눠 써서 사진 시험 132 ms보다 느림)
- 오탐: 모니터를 화면 대부분 크기 (0,0)-(518,359), 신뢰도 0.55~0.60 의 buds 로 잡음
  -> max_box_ratio 로 영상 넓이 25% 넘는 상자 제거. 근본 해결은 모니터가 보이는 배경 사진을 넣어 다시 학습

## QR 인식 시험
```bash
ros2 run rovpang_vision make_qr --text rovpang-test-01        # qr_rovpang-test-01.png 생성 → PC로 가져와 휴대폰·인쇄
ros2 run rovpang_vision qr_detector                           # 창 2
ros2 topic echo /rovpang/qr                                   # 창 3
```
QR 크기: 영상(640x360)에서 QR이 약 100픽셀 이상이어야 읽힌다(ROS 없는 시험 기준).
카메라 가로 화각을 약 80도로 잡으면 40cm 거리에서 화면 가로가 약 65cm이므로, QR은 약 10cm 이상이거나 더 가까이 보여야 한다.

## 위치 정확도 (11/24, 카메라 depth_registration:=true)
깊이 보정: 평평한 물체를 자로 30/50/80cm에 두고 화면 가운데 깊이를 잼(depth_check.py) → 32.9/54.3/86.3cm.
측정 = 1.068 × 실제 + 0.88cm → `depth_gain=1.068`, `depth_bias=0.0088`로 보정(보정 후 세 점 모두 오차 0.1cm 이내).

| 버즈 거리(자) | Z(보정 후) | 신뢰도 | 크기 추정 |
| --- | --- | --- | --- |
| 22cm | 21.5~21.7cm | 0.98~0.99 | 4.5~4.9cm |
| 40cm | 38.8~39.2cm | 0.91~0.96 | 5.2~5.9cm |
| 50cm | 46~50cm (흔들림) | 0.51~0.87 | 5.0~5.6cm |
| 70cm | 탐지 안 됨 | - | - |

→ LOOK 자세에서 카메라–물품 25~40cm. 실제 크기가 3~9cm 밖이면 버림(min_size_cm, max_size_cm).
깊이 영상은 카메라가 reliable 로 보내므로 reliable 로 구독(best effort 로 받으면 CPU가 바쁠 때 계속 끊김), 추론 스레드 3개.
