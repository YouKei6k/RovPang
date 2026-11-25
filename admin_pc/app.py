"""RovPang 관리자 프로그램 (F-01 배송 요청, F-15 상태·예외 알림 표시)

  pip install -r requirements.txt
  streamlit run app.py

왼쪽에서 로봇 IP로 연결(또는 '로봇 없이 시험') -> 물품·배송지를 고르고 요청 ->
수령자용 QR이 화면에 뜬다 (휴대폰으로 찍어 두었다가 배송지에서 로봇 카메라에 보여 줌).
"""
import io
import time

import qrcode
import streamlit as st

from logger import DeliveryLog
from ros_client import FakeLink, RosLink, new_request

ITEMS = ['buds']                 # 학습된 물품 클래스 (모델이 늘면 추가)
DESTS = ['A', 'B']               # A = 빨간 삼각형, B = 파란 원
BUSY = {'DETECTING', 'PICKING', 'MOVING', 'VERIFYING', 'PLACING', 'RETURNING'}
STATE_KO = {'IDLE': '대기', 'DETECTING': '물품 탐색', 'PICKING': '집기', 'MOVING': '배송지로 이동',
            'VERIFYING': '배송지·수령자 확인', 'PLACING': '하역', 'RETURNING': '출발지로 복귀',
            'DONE': '배송 완료', 'FAILED': '실패', 'CANCELED': '취소됨'}

st.set_page_config(page_title='RovPang 관리자', layout='wide')
st.title('RovPang 관리자 프로그램')


@st.cache_resource            # Streamlit 은 버튼을 누를 때마다 파일을 처음부터 다시 실행하므로 연결은 한 번만 만든다
def get_link(host, fake):
    log = DeliveryLog()
    return FakeLink(log) if fake else RosLink(host, log=log)


# ---------- 연결 ----------
with st.sidebar:
    st.header('로봇 연결')
    host = st.text_input('로봇 IP', '10.10.108.148')
    fake = st.checkbox('로봇 없이 시험 (가짜 연결)', value=False)
    if st.button('연결'):
        get_link.clear()
        st.session_state.conn = (host, fake)
    if 'conn' not in st.session_state:
        st.info('IP를 확인하고 [연결]을 누르세요')
        st.stop()
    try:
        link = get_link(*st.session_state.conn)
    except Exception as e:      # rosbridge 가 꺼져 있거나 IP가 틀림
        st.error('연결 실패: {}\n\n로봇 docker 에서 rosbridge 를 켰는지, IP·포트(9090)를 확인하세요.'.format(e))
        get_link.clear()
        st.stop()
    st.success('연결됨' + (' (가짜)' if st.session_state.conn[1] else ''))

# ---------- 배송 요청 (F-01) ----------
left, right = st.columns([1, 1])
with left:
    st.subheader('배송 요청')
    item = st.selectbox('물품', ITEMS)
    dest = st.radio('배송지', DESTS, horizontal=True)
    status, _, _, _ = link.snapshot()
    busy = bool(status) and status.get('state') in BUSY
    if st.button('요청 보내기', type='primary', disabled=busy):
        req = new_request(item, dest)
        link.send_request(req)
        st.session_state.last_req = req
    if busy:
        st.caption('배송 중에는 새 요청을 보낼 수 없습니다')

    c1, c2 = st.columns(2)
    if c1.button('배송 취소'):
        ok, msg = link.cancel()
        (st.success if ok else st.warning)('취소: {}'.format(msg))
    if c2.button('출발지로 복귀'):
        ok, msg = link.return_home()
        (st.success if ok else st.warning)('복귀: {}'.format(msg))

    req = st.session_state.get('last_req')
    if req:
        st.markdown('**수령자 QR** - 배송 {} / {} → {}'.format(req['delivery_id'], req['item_class'],
                                                            req['destination']))
        buf = io.BytesIO()
        qrcode.make(req['qr_token']).save(buf, format='PNG')
        st.image(buf.getvalue(), width=220)
        st.caption('이 QR을 수령자에게 전달하세요 (이 배송에서 한 번만 사용)')


# ---------- 상태·알림 (F-15), 1초마다 새로 그림 ----------
@st.fragment(run_every=1.0)
def live():
    status, t, history, alerts = link.snapshot()
    st.subheader('로봇 상태')
    if status is None:
        st.warning('상태를 아직 받지 못했습니다 (task_manager 실행 확인)')
    else:
        state = status.get('state', '')
        age = time.time() - t
        label = '{} ({})'.format(STATE_KO.get(state, state), state)
        if state == 'FAILED':
            st.error('{}  -  {} {}'.format(label, status.get('error_code', ''), status.get('message', '')))
        elif state == 'DONE':
            st.success(label)
        else:
            st.info('{}  -  {}'.format(label, status.get('message', '')))
        if age > 3.0:   # task_manager 는 1초마다 상태를 다시 보낸다
            st.warning('{:.0f}초 동안 상태가 오지 않습니다. 로봇·task_manager 를 확인하세요'.format(age))
    st.subheader('예외 알림')
    if alerts:
        for when, text in alerts[:10]:
            st.write('{}  {}'.format(when, text))
    else:
        st.caption('알림 없음')
    st.subheader('진행 기록')
    st.dataframe([{'시각': h[0], '배송번호': h[1], '상태': h[2], '오류': h[3], '설명': h[4]} for h in history],
                 width='stretch', hide_index=True)


with right:
    live()


# ---------- 배송 이력 (logger.py, admin_pc/logs/deliveries.csv) ----------
st.divider()
with st.expander('배송 이력 (이 PC에 저장된 deliveries.csv)'):
    hist = DeliveryLog()
    st.dataframe(hist.read(200), width='stretch', hide_index=True)
    st.download_button('CSV 내려받기', hist.raw(), file_name='deliveries.csv', mime='text/csv')
