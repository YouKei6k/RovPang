# rovpang_vision (팀원 1)

Vision AI: 물품 탐지(F-02), 위치 계산(F-03, 예정), 배송지 표지판·QR 인식(F-09·F-10, 예정).

| 파일 | 역할 |
| --- | --- |
| object_detector | 카메라 컬러 영상을 YOLO(ONNX)로 탐지해 /rovpang/detections 발행 (3차원 위치는 아직 없음) |
| qr_detector | 카메라 영상에서 QR을 읽어 /rovpang/qr (std_msgs/String) 발행. 인증 판단은 task_manager(recipient_auth) |
| make_qr | 시험용 QR 그림 만들기 (`--text 토큰`) |
| (모듈) qr_tools | QR 읽기(기본 검출기 → 못 읽으면 ArUco 기반 검출기), QR 그림 만들기 |
| (모듈) yolo_onnx | YOLOv8 ONNX 탐지기. onnxruntime CPU로 실행 (ultralytics·GPU 불필요) |
| (모듈) image_utils | ROS 영상 <-> OpenCV 변환 (cv_bridge 없이) |

## 모델
- Colab에서 best.pt를 ONNX로 변환: `YOLO('best.pt').export(format='onnx', imgsz=320, opset=12, simplify=True)`
- 로봇 위치: `/home/ubuntu/share/tmp/rovpang_models/best_320.onnx` (본체 `~/docker/tmp/rovpang_models`, GitHub에는 올리지 않음)
- Jetson Nano CPU 측정(11/23): 320 → 132 ms/장(7.6 FPS), 640 → 483 ms/장(2.1 FPS, 모니터 오탐 0.57) → 320 사용

## 실행
```bash
ros2 launch peripherals depth_camera.launch.py depth_registration:=true   # 창 1: 카메라
ros2 run rovpang_vision object_detector                                     # 창 2
ros2 topic echo /rovpang/detections                                         # 창 3
```
파라미터: model_path, image_topic, conf(0.5), rate(3.0), threads(4), publish_image(true), max_box_ratio(0.25)

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
