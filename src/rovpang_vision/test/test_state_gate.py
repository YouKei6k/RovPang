"""StateGate 시험: 필요한 배송 단계에서만 처리하는지"""
from rovpang_vision.state_gate import StateGate


def test_active_before_any_status():
    assert StateGate(['VERIFYING']).active()          # task_manager 없이 따로 시험할 때


def test_only_in_listed_states():
    g = StateGate(['VERIFYING'])
    g.update('MOVING')
    assert not g.active()
    g.update('VERIFYING')
    assert g.active()
    g.update('DONE')
    assert not g.active()


def test_disabled_always_active():
    g = StateGate(['VERIFYING'], enabled=False)
    g.update('IDLE')
    assert g.active()
