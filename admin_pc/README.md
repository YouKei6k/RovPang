# admin_pc - RovPang 관리자 프로그램 (A 방식: rosbridge)

로봇과 웹소켓(rosbridge, 포트 9090)으로 통신한다. 관리자 PC에는 ROS2가 필요 없다(Windows 가능).
colcon build 대상이 아니다 (src/ 밖에 둠).

| 기능 | 내용 |
|---|---|
| F-01 배송 요청 | 물품·배송지 선택 → /rovpang/delivery_request 발행, 배송 번호·일회용 QR 토큰 생성, 수령자용 QR 표시 |
| F-15 상태·알림 | /rovpang/status (1초마다 갱신), /rovpang/alert 예외 이유 표시, 상태가 바뀐 기록 |
| 취소·복귀 | /rovpang/cancel, /rovpang/return_home (std_srvs/Trigger) |
| 배송 이력 | 상태 변화·예외 알림을 admin_pc/logs/deliveries.csv 에 저장, 화면에서 조회·CSV 내려받기 |

| 파일 | 역할 |
|---|---|
| app.py | 메인 화면 (Streamlit) |
| ros_client.py | rosbridge(웹소켓) 연결, 토픽 발행·구독, 서비스 호출. FakeLink = 로봇 없이 시험 |
| logger.py | 배송 이력·오류 로그(deliveries.csv) 저장·조회 |
| requirements.txt | PC에 설치할 패키지 (streamlit, roslibpy, qrcode) |

## 실행
로봇 docker (워크스페이스를 source 한 창, 우리 메시지 형식을 알아야 하므로):
```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```
관리자 PC:
```bash
pip install -r requirements.txt
streamlit run app.py
```
로봇 없이 화면만 볼 때는 왼쪽 '로봇 없이 시험'에 체크하고 연결.

## 주의
- 카메라 영상 같은 큰 토픽은 rosbridge 로 받지 않는다 (JSON 변환에 Jetson CPU 사용).
- rosbridge 는 같은 네트워크 누구나 접속 가능 (실습실 범위).
- 로봇 IP가 바뀌면 왼쪽 입력란에서 바꾼다 (공유기에서 고정 IP 권장).
