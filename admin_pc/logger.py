"""배송 이력·오류 로그 저장·조회 (관리자 PC 쪽 deliveries.csv)

로봇의 task_manager 도 ~/rovpang_logs/deliveries.csv 를 남기지만, 그건 로봇 안에 있다.
관리자 PC에서도 받은 상태 변화와 예외 알림을 같은 형식으로 남겨, 로봇에 접속하지 않고 이력을 볼 수 있게 한다.
  저장 위치: admin_pc/logs/deliveries.csv (GitHub에는 올리지 않음)
"""
import csv
import os
import threading
import time

HEADER = ['time', 'delivery_id', 'state', 'error_code', 'message']


class DeliveryLog:
    def __init__(self, path=None):
        self.path = path or os.path.join(os.path.dirname(os.path.abspath(__file__)), 'logs', 'deliveries.csv')
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        self.lock = threading.Lock()
        if not os.path.exists(self.path):
            with open(self.path, 'w', newline='', encoding='utf-8-sig') as f:   # utf-8-sig: 엑셀에서 한글 안 깨짐
                csv.writer(f).writerow(HEADER)

    def write(self, delivery_id, state, error_code='', message=''):
        row = [time.strftime('%Y-%m-%d %H:%M:%S'), delivery_id, state, error_code, message]
        with self.lock, open(self.path, 'a', newline='', encoding='utf-8-sig') as f:
            csv.writer(f).writerow(row)

    def read(self, last=100):
        """최근 last 줄 (최신이 위)"""
        with self.lock, open(self.path, newline='', encoding='utf-8-sig') as f:
            rows = list(csv.DictReader(f))
        return rows[::-1][:last]

    def raw(self):
        with self.lock, open(self.path, 'rb') as f:
            return f.read()
