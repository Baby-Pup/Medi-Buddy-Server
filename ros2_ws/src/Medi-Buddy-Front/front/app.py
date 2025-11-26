import streamlit as st
import json
import time
import os

st.set_page_config(layout="wide")

FILE_PATH = "/tmp/robot_ui_status.json"

# ===== 세션 상태 초기화 =====
if "qr_start_time" not in st.session_state:
    st.session_state.qr_start_time = None   # qr_start 발생 시 기록할 시간

if "intro_step" not in st.session_state:
    st.session_state.intro_step = 0
    st.session_state.start_time = time.time()


def read_status():
    if os.path.exists(FILE_PATH):
        with open(FILE_PATH) as f:
            data = json.load(f)
        return data.get("status", "")
    return ""


status = read_status()

# ---------- QR START EVENT ----------
if status == "qr_start":
    # 처음 qr_start 감지 시 시간 기록
    if st.session_state.qr_start_time is None:
        st.session_state.qr_start_time = time.time()

    # 즉시 start_page 로 전환
    st.switch_page("pages/start_page.py")

# ======= 전체 화면 배경색 + 레이아웃 CSS =======
GLOBAL_CSS = """
<style>
/* 전체 화면 배경 */
.stApp {
    background-color: #102A4C !important;
}

/* 기본 패딩 제거 */
.block-container {
    padding: 0 !important;
}

/* 상단 헤더 숨기기 */
header, .stToolbar {
    display: none !important;
}

/* 중앙 정렬 컨테이너 */
.center-container {
    width: 100%;
    height: 100vh;
    display: flex;
    justify-content: center;
    align-items: center;
    flex-direction: column;
}

/* 눈 */
.eye {
    width: 45px;
    height: 60px;
    background: #D8D8D8;
    border-radius: 50%;
    margin: 0 240px;
    animation: blink 3s infinite;
}

/* 입: 미소 */
.mouth-smile {
    width: 60px;
    height: 20px;
    border: 3px solid white;
    border-color: transparent transparent white transparent;
    border-radius: 0 0 40px 40px;
    margin-top: 10px;
}

/* 눈 깜빡임 */
@keyframes blink {
    0% { transform: scaleY(1); }
    45% { transform: scaleY(1); }
    50% { transform: scaleY(0.1); }
    55% { transform: scaleY(1); }
    100% { transform: scaleY(1); }
}
</style>
"""

st.markdown(GLOBAL_CSS, unsafe_allow_html=True)


# ======= 메인 UI =======
st.markdown("""
    <div class="center-container">
        <div style="display: flex; flex-direction: row;">
            <div class="eye"></div>
            <div class="eye"></div>
        </div>
        <div class="mouth-smile"></div>
    </div>
""", unsafe_allow_html=True)

# ======= 자동 rerun =======
time.sleep(0.08)
st.rerun()
