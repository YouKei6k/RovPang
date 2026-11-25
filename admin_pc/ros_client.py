"""관리자 프로그램 <-> 로봇 연결 (rosbridge 웹소켓, roslibpy)

로봇(docker)에서:  ros2 launch rosbridge_server rosbridge_websocket_launch.xml   (포트 9090)
관리자 PC에서  :  RosLink('로봇IP') 로 접속

ROS2가 없는 PC(Windows 포함)에서도 동작한다. JSON으로 주고받기 때문.
FakeLink 는 로봇 없이 화면을 시험할 때 쓰는 가짜 연결 (상태가 차례로 바뀜).
"""
import secrets
import threading
import time
from collections import deque

STATES = ['DETECTING', 'PICKING', 'MOVING', 'VERIFYING', 'PLACING', 'RETURNING', 'DONE']


def new_request(item, dest):
    """배송 요청 한 건: 배송 번호와 이 배송에만 쓰는 QR 토큰을 새로 만든다."""
    return {
        'delivery_id': time.strftime('D%m%d-%H%M%S'),
        'item_class': item,
        'destination': dest,
        'qr_token': 'rovpang-' + secrets.token_hex(4),
    }


class _Base:
    """상태·알림을 모아 두는 공통 부분 (화면은 이 값을 1초마다 읽어 그린다)
    log 를 주면 상태가 바뀔 때와 알림이 올 때 logger.DeliveryLog 에 남긴다."""
    def __init__(self, log=None):
        self.log = log
        self.lock = threading.Lock()
        self.status = None                 # 가장 최근 /rovpang/status (dict)
        self.status_time = 0.0
        self.history = deque(maxlen=200)   # (시각, 배송번호, 상태, 오류코드, 설명) 상태가 바뀔 때만
        self.alerts = deque(maxlen=50)     # (시각, 안내 문장)

    def _on_status(self, m):
        now = time.time()
        with self.lock:
            prev = self.status
            self.status, self.status_time = m, now
            # task_manager 는 같은 상태를 1초마다 다시 보내므로, 바뀐 것만 기록
            key = (m.get('delivery_id'), m.get('state'), m.get('error_code'), m.get('message'))
            if prev is None or key != (prev.get('delivery_id'), prev.get('state'),
                                       prev.get('error_code'), prev.get('message')):
                self.history.appendleft((time.strftime('%H:%M:%S'),) + key)
                if self.log:
                    self.log.write(*key)

    def _on_alert(self, text):
        with self.lock:
            self.alerts.appendleft((time.strftime('%H:%M:%S'), text))
            if self.log:
                self.log.write('', 'ALERT', '', text)

    def snapshot(self):
        with self.lock:
            return self.status, self.status_time, list(self.history), list(self.alerts)


class RosLink(_Base):
    def __init__(self, host, port=9090, log=None):
        super().__init__(log)
        import roslibpy   # 필요할 때만 불러옴 (FakeLink 는 설치 없이 동작)
        self.roslibpy = roslibpy
        self.ros = roslibpy.Ros(host=host, port=port)
        self.ros.run(timeout=5)            # 연결 실패 시 예외
        self.req_pub = roslibpy.Topic(self.ros, '/rovpang/delivery_request',
                                      'rovpang_interfaces/msg/DeliveryRequest')
        self.req_pub.advertise()
        roslibpy.Topic(self.ros, '/rovpang/status', 'rovpang_interfaces/msg/DeliveryStatus') \
            .subscribe(self._on_status)
        roslibpy.Topic(self.ros, '/rovpang/alert', 'std_msgs/msg/String') \
            .subscribe(lambda m: self._on_alert(m.get('data', '')))

    @property
    def connected(self):
        return self.ros.is_connected

    def send_request(self, req):
        self.req_pub.publish(self.roslibpy.Message(req))

    def _trigger(self, name):
        srv = self.roslibpy.Service(self.ros, name, 'std_srvs/srv/Trigger')
        res = srv.call(self.roslibpy.ServiceRequest(), timeout=5)
        return bool(res['success']), res['message']

    def cancel(self):
        return self._trigger('/rovpang/cancel')

    def return_home(self):
        return self._trigger('/rovpang/return_home')

    def close(self):
        self.ros.terminate()


class FakeLink(_Base):
    """로봇 없이 화면 시험: 요청을 보내면 1.5초마다 다음 단계로 넘어간다."""
    connected = True

    def __init__(self, log=None):
        super().__init__(log)
        self._on_status({'delivery_id': '', 'state': 'IDLE', 'error_code': '', 'message': '대기 중'})
        self._stop = threading.Event()

    def send_request(self, req):
        self._stop.clear()

        def run():
            for st in STATES:
                if self._stop.wait(1.5):
                    return
                self._on_status({'delivery_id': req['delivery_id'], 'state': st, 'error_code': '',
                                 'message': '가짜 진행 ({} -> {})'.format(req['item_class'], req['destination'])})
        threading.Thread(target=run, daemon=True).start()

    def cancel(self):
        self._stop.set()
        self._on_status({'delivery_id': '', 'state': 'CANCELED', 'error_code': 'CANCELED', 'message': '취소'})
        return True, '가짜 취소'

    def return_home(self):
        self._on_alert('[가짜] 복귀 요청')
        return True, '가짜 복귀'

    def close(self):
        self._stop.set()
