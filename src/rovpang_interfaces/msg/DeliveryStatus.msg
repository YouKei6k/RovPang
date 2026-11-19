# 미션 관리 노드 -> 관리자 프로그램 : /rovpang/status
string STATE_IDLE="IDLE"
string STATE_DETECTING="DETECTING"
string STATE_PICKING="PICKING"
string STATE_MOVING="MOVING"
string STATE_VERIFYING="VERIFYING"
string STATE_PLACING="PLACING"
string STATE_RETURNING="RETURNING"
string STATE_DONE="DONE"
string STATE_FAILED="FAILED"
string STATE_CANCELED="CANCELED"

std_msgs/Header header
string delivery_id
string state               # 위 상수 중 하나
string error_code          # 오류가 없으면 ""
string message             # 사람이 읽을 설명
