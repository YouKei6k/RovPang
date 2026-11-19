# 물체 하나의 탐지 결과 (팀원 1: Vision 노드가 발행)
string class_name          # "buds", "box" 등
float32 confidence         # 0.0 ~ 1.0
int32 x_min                # 컬러 영상 기준 바운딩 박스 (픽셀)
int32 y_min
int32 x_max
int32 y_max
bool has_position          # 깊이 값이 있어 3차원 위치를 계산했으면 true
geometry_msgs/Point position   # 카메라 좌표계(header.frame_id) 기준, 단위 m
