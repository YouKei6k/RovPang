"""recipient_auth 단위 시험 (ROS 없이 실행 가능: python3 -m pytest test/)"""
from rovpang_manager.recipient_auth import (
    NOT_REGISTERED, OK, QR_INVALID, QR_REUSED, RecipientAuth, new_token)


def test_correct_token_passes_once():
    auth = RecipientAuth()
    t = new_token()
    auth.register('D1', t)
    assert auth.verify('D1', t) == (True, OK)
    # 같은 토큰을 다시 보여주면 재사용으로 거부
    assert auth.verify('D1', t) == (False, QR_REUSED)


def test_other_delivery_token_rejected():
    auth = RecipientAuth()
    auth.register('D1', 'aaaa1111')
    auth.register('D2', 'bbbb2222')
    assert auth.verify('D1', 'bbbb2222') == (False, QR_INVALID)
    assert auth.verify('D1', 'WRONG-TOKEN') == (False, QR_INVALID)


def test_used_token_of_previous_delivery_rejected():
    auth = RecipientAuth()
    auth.register('D1', 'aaaa1111')
    assert auth.verify('D1', 'aaaa1111')[0]
    auth.register('D2', 'bbbb2222')
    # 지난 배송에서 이미 쓴 QR을 다음 배송에 보여준 경우
    assert auth.verify('D2', 'aaaa1111') == (False, QR_REUSED)


def test_unregistered_delivery():
    auth = RecipientAuth()
    assert auth.verify('D9', 'x') == (False, NOT_REGISTERED)


def test_new_token_is_random_hex():
    a, b = new_token(), new_token()
    assert a != b and len(a) == 8
    int(a, 16)
