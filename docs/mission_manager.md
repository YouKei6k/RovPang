# 미션 관리 노드 (rovpang_mission) – 시험 결과와 연동 시 주의할 점

작성: 정서인 (팀원 1) · 2026-11-19

## 1. 무엇을 하는 패키지인가

| 실행 파일 | 역할 |
| --- | --- |
| mission_manager | F-14 배송 상태 관리. 요청을 받아 팀원 2의 액션·서비스를 순서대로 호출하고 /rovpang/status 발행, ~/rovpang_logs/deliveries.csv 기록 |
| mock_robot | 팀원 2 기능(pick, place, go_to, set_arm_pose)을 흉내 내는 가짜 노드. 로봇은 움직이지 않음 |
| mock_vision | 팀원 1 비전 노드(detections, sign, qr)를 흉내 내는 가짜 노드 |
| send_request | 관리자 프로그램 대신 배송 요청 1건 발행 |

상태 흐름: IDLE → DETECTING → PICKING → MOVING → VERIFYING → PLACING → RETURNING → DONE (실패 FAILED, 취소 CANCELED)

## 2. 가짜 노드 시험 결과 (2026-11-19, 로봇 도커 Humble)

| 시험 | 실행 옵션 | 결과 |
| --- | --- | --- |
| 정상 배송 | (없음) | DONE, 약 11.7초 |
| 집기 실패 | fail_on:=pick | FAILED / PICK_FAILED, 출발 전이라 복귀 없음 |
| 다른 배송지 | wrong_sign:=true | FAILED / WRONG_DESTINATION (요청 A / 인식 B 5/5), 하역 안 함, 출발지 복귀 |
| 틀린 QR | wrong_qr:=true | FAILED / QR_INVALID, 하역 안 함, 출발지 복귀 |
| 이동 중 취소 | delay:=10.0 | CANCELED, 출발지 복귀 성공 |
| 복귀 중 취소 | delay:=10.0 | DONE / RETURN_CANCELED (하역까지 끝났으므로 배송 성공으로 기록) |

## 3. 팀원 2 기능과 연동할 때 주의할 점

mock_robot 자리에 실제 노드가 들어갈 때 아래를 지켜야 지금 시험 결과가 그대로 유지된다.

1. **go_to 취소 처리 (중요)**
   이동 중 취소되면 미션 관리 노드는 곧바로 go_to(START)를 다시 보낸다.
   실제 go_to 액션은 취소 요청을 받으면 Nav2 목표를 취소해 **로봇을 먼저 멈춘 뒤** canceled로 끝나야 한다.
   그렇지 않으면 이전 목적지와 출발지 이동이 겹칠 수 있다.
2. **pick 목표 좌표계**
   Pick.Goal.target은 /rovpang/detections의 header.frame_id(카메라 좌표계) 그대로 보낸다.
   팔 좌표계 변환은 팀원 2가 TF로 처리한다. 실패 시 result.success=false와 reason을 채워야 오류가 기록된다.
3. **place**
   지금은 use_default=true(정해진 하역대 위치)만 쓴다.
4. **set_arm_pose (HOME / CARRY / LOOK)**
   LOOK 자세에서 물품·표지판·QR이 카메라에 보여야 하고, 깊이 카메라 특성상 카메라–대상 거리를 25~30cm 이상 두어야 한다(약 20cm 이내는 깊이 없음).
5. **액션 서버 구조**
   액션 콜백 안에서 spin_until_future_complete를 쓰면 멈출 수 있다.
   MultiThreadedExecutor + ReentrantCallbackGroup으로 만들고 서비스 결과는 future가 끝날 때까지 기다리는 방식이 안전하다(mock_robot.py 참고).
6. **시간 제한**
   미션 관리 노드는 액션 서버를 5초 기다리고, 액션 결과는 최대 60초(action_timeout), 팔 자세 서비스는 15초 기다린다. 넘으면 TIMEOUT으로 실패 처리한다.

## 4. 실행 방법

```bash
cd /home/ubuntu/share/tmp/<워크스페이스>
colcon build --packages-select rovpang_interfaces rovpang_mission
source install/setup.zsh
ros2 launch rovpang_mission mock_test.launch.py            # 창 1 (옵션: fail_on, wrong_sign, wrong_qr, delay)
ros2 topic echo /rovpang/status                            # 창 2
ros2 run rovpang_mission send_request --dest A --item buds # 창 3
ros2 service call /rovpang/cancel std_srvs/srv/Trigger     # 취소
```
delay 값은 10.0처럼 소수점을 붙여야 한다.
