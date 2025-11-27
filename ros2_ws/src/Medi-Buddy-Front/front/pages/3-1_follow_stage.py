import streamlit as st
import base64
import json
import time
import os
import streamlit.components.v1 as components

st.set_page_config(layout="wide")

FILE_PATH = "/tmp/robot_ui_status.json"

# ===== 세션 상태 초기화 =====
if "qr_complete_time" not in st.session_state:
    st.session_state.qr_complete_time = None   # qr_complete 발생 시 기록할 시간

# Base64 이미지
def get_base64_image(path):
    try:
        with open(path, "rb") as f:
            return base64.b64encode(f.read()).decode()
    except:
        return None

body_img = get_base64_image("assets/face_smile.png")

def read_status():
    if not os.path.exists(FILE_PATH):
        return ""

    try:
        with open(FILE_PATH, "r") as f:
            txt = f.read().strip()
            if not txt:
                return ""
            data = json.loads(txt)
            return data.get("status", "")
    except Exception:
        # JSON이 깨졌거나, 쓰는 중이거나, parse 실패 → 기본값 반환
        return ""


status = read_status()

# ---------- QR START EVENT ----------
if status == "qr_complete":
    # 처음 qr_complete 감지 시 시간 기록
    if st.session_state.qr_complete_time is None:
        st.session_state.qr_complete_time = time.time()
    # 즉시 3-2_follow_stage 로 전환
    st.switch_page("pages/3-2_follow_stage.py")

# =============================
# ⭐ HTML + CSS + JS 직접 구성
# =============================
st.markdown("""
<style>
html, body, .stApp, .block-container, .main {
    background-color: #102A4C !important;
}
header, .stToolbar { display: none !important; }
</style>
""", unsafe_allow_html=True)

components.html(f"""
<!DOCTYPE html>
<html>
<head>
<style>

@import url('https://fonts.googleapis.com/css2?family=Jua&display=swap');
* {{
    font-family: "Jua", sans-serif;
    margin: 0;
    padding: 0;
}}

body {{
    background-color: #102A4C;
}}

.inner-wrapper {{
    background: #F7F3EB;
    width: 100vw;
    height: 100vh;
    display: flex;
    flex-direction: row;
    align-items: center;
    justify-content: center;
}}

.left-area {{
    display: flex;
    flex-direction: column;
    align-items: center;
    transform: translateY(-4%);
    width: 55%;
    margin-top: -40px;
}}

.character-img {{
    width: 34vw;
    max-width: 380px;
}}

.guide-text {{
    font-size: 3.6rem;
    color: #0E2C55;
    margin-top: 20px;
    text-align: center;
}}

.right-area {{
    width: 45%;
    display: flex;
    justify-content: center;
    transform: translateX(-40px);
}}

.camera-square {{
    width: 450px;
    height: 450px;
    background: black;
    border-radius: 20px;
    overflow: hidden;
    position: relative;
}}

.camera-square video {{
    width: 100%;
    height: 100%;
    object-fit: cover;
}}

</style>
</head>

<body>
<div class="inner-wrapper">

    <div class="left-area">
        <img src="data:image/png;base64,{body_img}" class="character-img">
        <div class="guide-text">
            문진표 상단의 QR를<br>
            오른쪽 화면에 보여주세요!
        </div>
    </div>

    <div class="right-area">
        <div class="camera-square">
            <video id="cam" autoplay playsinline></video>
        </div>
    </div>

</div>

<script>
navigator.mediaDevices.getUserMedia({{
    video: {{ facingMode: "environment" }},
    audio: false
}})
.then(stream => {{
    document.getElementById("cam").srcObject = stream;
}})
.catch(err => {{
    console.log("Camera error:", err);
}});
</script>

</body>
</html>
""", height=770, scrolling=False)

# ======= 자동 rerun =======
time.sleep(0.08)
st.rerun()