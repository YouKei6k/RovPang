# RovPang 인터페이스·실행 메모 (팀원 1 초안)

`src/rovpang_interfaces` 패키지와 `tools/` 스크립트 설명. 인터페이스는 팀원 1·2가 합의한 뒤에만 수정한다.

`tools/`: capture_ros.py(ROS 토픽으로 학습 사진 저장), depth_center.py(지정 픽셀 거리 출력), capture_dataset.py(카메라 직접 열어 사진 저장, ROS 카메라가 꺼져 있을 때만)

## 인터페이스

| 이름 | 종류 | 타입 | 제공 | 사용 |
| --- | --- | --- | --- | --- |
| /rovpang/delivery_request | topic | rovpang_interfaces/msg/DeliveryRequest | 관리자 프로그램(1) | 미션 관리(1) |
| /rovpang/detections | topic | rovpang_interfaces/msg/DetectionArray | Vision(1) | 미션 관리(1), 팔 제어(2) |
| /rovpang/sign | topic | rovpang_interfaces/msg/Sign | Vision(1) | 미션 관리(1) |
| /rovpang/qr | topic | std_msgs/msg/String | QR(1) | 미션 관리(1) |
| /rovpang/pick | action | rovpang_interfaces/action/Pick | 팔 제어(2) | 미션 관리(1) |
| /rovpang/place | action | rovpang_interfaces/action/Place | 팔 제어(2) | 미션 관리(1) |
| /rovpang/set_arm_pose | service | rovpang_interfaces/srv/SetArmPose | 팔 제어(2) | 미션 관리(1) |
| /rovpang/go_to | action | rovpang_interfaces/action/GoTo | 주행(2) | 미션 관리(1) |
| /rovpang/status | topic | rovpang_interfaces/msg/DeliveryStatus | 미션 관리(1) | 관리자 프로그램(1) |
| /rovpang/cancel | service | std_srvs/srv/Trigger | 미션 관리(1) | 관리자 프로그램(1) |

좌표: 팀원 1은 카메라 좌표계로 위치를 보내고, 팔 좌표계 변환은 팀원 2가 TF로 처리한다.

## 로봇에서 빌드 (도커, ubuntu 계정)

```bash
sudo systemctl stop start_app_node.service        # 로봇 본체에서 (ROS1이 카메라를 잡지 않게)
docker exec -it -u ubuntu jetrover /bin/zsh
cd /home/ubuntu/share/tmp && git clone https://github.com/mjnx13/RovPang.git
cd /home/ubuntu/share/tmp/RovPang
colcon build --packages-select rovpang_interfaces
source install/setup.zsh
ros2 interface show rovpang_interfaces/action/Pick
```

## 카메라 실행

```bash
ros2 launch peripherals depth_camera.launch.py depth_registration:=true
```
