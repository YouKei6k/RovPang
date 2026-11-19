# rovpang_mission (팀원 1)

배송 미션 관리 노드(F-14)와 시험용 가짜 노드. 가짜 노드만 쓰면 로봇은 움직이지 않는다.

| 실행 파일 | 역할 |
| --- | --- |
| mission_manager | 배송 상태 머신, /rovpang/status 발행, ~/rovpang_logs/deliveries.csv 기록, /rovpang/cancel |
| mock_robot | 팀원 2의 pick / place / go_to / set_arm_pose 흉내 (fail_on 으로 실패 시험) |
| mock_vision | 팀원 1의 detections / sign / qr 흉내 (wrong_sign, wrong_qr) |
| send_request | 관리자 프로그램 대신 배송 요청 1건 발행 |

```bash
colcon build --packages-select rovpang_interfaces rovpang_mission
source install/setup.zsh
ros2 launch rovpang_mission mock_test.launch.py            # 창 1
ros2 topic echo /rovpang/status                            # 창 2
ros2 run rovpang_mission send_request --dest A --item buds # 창 3
ros2 service call /rovpang/cancel std_srvs/srv/Trigger     # 취소 시험
```
실패 시험: `fail_on:=pick` / `fail_on:=go_to` / `wrong_sign:=true` / `wrong_qr:=true`
