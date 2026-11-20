#!/usr/bin/env python3
"""수령자 인증 (F-10) - 배송별 일회용 QR 토큰 관리

ROS에 의존하지 않는 순수 파이썬 모듈이라 단위 시험(test/test_recipient_auth.py)으로 바로 확인할 수 있다.
task_manager가 배송을 시작할 때 register(), QR을 받을 때 verify()를 호출한다.
"""
import secrets

# verify() 결과 코드 (task_manager의 오류 코드와 같음)
OK = 'OK'
QR_INVALID = 'QR_INVALID'      # 이 배송의 토큰이 아님 (다른 배송 QR, 잘못된 문자열)
QR_REUSED = 'QR_REUSED'        # 이미 인증에 사용한 토큰
NOT_REGISTERED = 'NOT_REGISTERED'  # 등록되지 않은 배송 ID


def new_token(nbytes=4):
    """배송별 일회용 토큰 생성 (예: 'a3f09c1e')."""
    return secrets.token_hex(nbytes)


class RecipientAuth:
    def __init__(self):
        self._tokens = {}     # delivery_id -> token
        self._used = set()    # 사용한 토큰

    def register(self, delivery_id, token):
        """배송 요청을 받을 때 그 배송의 토큰을 등록한다."""
        if not delivery_id or not token:
            raise ValueError('delivery_id와 token이 필요합니다')
        self._tokens[delivery_id] = token

    def verify(self, delivery_id, qr_data):
        """수령자가 보여준 QR 문자열을 검사한다. 성공하면 토큰을 사용 처리한다.

        반환: (성공 여부, 결과 코드)
        """
        token = self._tokens.get(delivery_id)
        if token is None:
            return False, NOT_REGISTERED
        if qr_data in self._used:
            return False, QR_REUSED
        if qr_data != token:
            return False, QR_INVALID
        self._used.add(qr_data)
        return True, OK

    def is_used(self, token):
        return token in self._used
