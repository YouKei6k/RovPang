"""필요한 단계에서만 영상 처리하기 (CPU 아끼기, ROS 없이 시험 가능)

Jetson Nano CPU에서 object_detector(약 200~300 ms/장), qr_detector(약 120~220 ms/장)를 계속 같이 돌리면
CPU가 모자라 깊이 영상을 놓치는 일이 있었다(11/24). 배송 단계(/rovpang/status 의 state)를 보고
그 노드가 필요한 단계에서만 처리한다.

  object_detector : DETECTING, PICKING, PLACING   (물품 찾기, 집기 확인, 하역 확인)
  sign_detector   : VERIFYING                      (표지판 확인)
  qr_detector     : VERIFYING                      (QR 인증)

task_manager 상태를 한 번도 받지 못했으면(노드를 따로 시험할 때) 항상 처리한다.
"""


class StateGate:
    def __init__(self, active_states, enabled=True):
        self.active_states = set(active_states)
        self.enabled = enabled
        self.state = None          # 마지막으로 받은 배송 상태 (아직 없으면 None)

    def update(self, state):
        self.state = state

    def active(self):
        if not self.enabled or self.state is None:
            return True
        return self.state in self.active_states
