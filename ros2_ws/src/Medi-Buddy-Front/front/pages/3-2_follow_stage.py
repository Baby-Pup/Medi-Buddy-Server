import streamlit as st
import base64
import json
import time
import os

st.set_page_config(layout="wide")

FILE_PATH = "/tmp/robot_ui_status.json"

# ========== 세션 상태 초기화 ==========
if "qr_complete" not in st.session_state:
    st.session_state.qr_complete = time.time()   # QR 완료 시간 기록

if "text_changed" not in st.session_state:
    st.session_state.text_changed = False        # 텍스트 변경 여부 기록


# ========== Base64 이미지 ==========
def get_base64_image(path):
    try:
        with open(path, "rb") as f:
            return base64.b64encode(f.read()).decode()
    except:
        return None

body_img = get_base64_image("assets/body_surprize.png")

def read_name():
    if not os.path.exists(FILE_PATH):
        return ""

    try:
        with open(FILE_PATH, "r") as f:
            txt = f.read().strip()
            if not txt:
                return ""
            data = json.loads(txt)
            return data.get("client_name", "")
    except Exception:
        # JSON이 깨졌거나, 쓰는 중이거나, parse 실패 → 기본값 반환
        return ""


name = read_name()

# ========== TEXT 자동 변경 (2초 후) ==========
elapsed = time.time() - st.session_state.qr_complete

if elapsed >= 2.0 and not st.session_state.text_changed:
    st.session_state.text_changed = True

# 변경되는 텍스트
if not st.session_state.text_changed:
    bottom_text = f"{name} 님, 오늘의 예약을 안내해드릴까요?"
else:
    bottom_text = "가는 중에 필요하신게 있다면 말씀해주세요."


# ========== 4초 후 자동 페이지 이동 ==========
if elapsed >= 4.0:
    st.switch_page("pages/1_map.py")


# =============================
# CSS
# =============================
st.markdown("""
<style>

@import url('https://fonts.googleapis.com/css2?family=Jua&display=swap');

* { font-family: "Jua", sans-serif !important; }

.stApp { background-color: #102A4C !important; }
header, .stToolbar { display: none !important; }

.page-wrapper {
    height: 100vh;
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-content: center;
    transform: translateY(-6%);
}

.character-img {
    width: 45vw;
    max-width: 520px;
    margin-bottom: 25px;
    transform: translateX(-6%);
}

.bottom-box {
    background: #F7F3EB;
    padding: 22px 40px;
    border-radius: 20px;
    font-size: 1.6rem;
    color: #0E2C55;
    text-align: center;
    min-width: 700px;
    max-width: 1150px;
}
</style>
""", unsafe_allow_html=True)


# =============================
# HTML
# =============================
st.html(f"""
<div class="page-wrapper">
    <img src="data:image/png;base64,{body_img}" class="character-img">
    <div class="bottom-box">{bottom_text}</div>
</div>
""")

# ========== 자동 rerun ==========
time.sleep(0.08)
st.rerun()
