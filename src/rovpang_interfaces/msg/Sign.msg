# 배송지 표지판 인식 결과 -> /rovpang/sign
std_msgs/Header header
string label               # "A", "B", 인식 실패 시 ""
float32 confidence

# 표지판 위치 (11/25 추가, 도착 후 정렬용). 기준은 header.frame_id 카메라 좌표계
float32 offset_x           # 표지판 중심이 화면 가운데에서 벗어난 정도: -1 왼쪽 끝, 0 가운데, +1 오른쪽 끝
float32 angle              # 카메라 정면에서 표지판 중심까지 가로 방향 각도(rad). +는 오른쪽 (로봇 기준 시계 방향)
bool has_distance          # 깊이로 거리를 쟀으면 true (너무 가깝거나 반사로 못 재면 false)
float32 distance           # 카메라 정면 방향(Z) 거리(m). has_distance 가 false 면 0
geometry_msgs/Point position   # 카메라 좌표계 3차원 위치(m): x 오른쪽, y 아래, z 앞. has_distance 가 true 일 때만 의미
