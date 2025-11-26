import streamlit as st
import base64
import json
import time
import os


st.set_page_config(layout="wide")

# Base64 인코딩 함수
def get_base64_image(path):
    try:
        with open(path, "rb") as f:
            return base64.b64encode(f.read()).decode()
    except:
        return None

body_img = get_base64_image("assets/body_stethoscope.png")
speechbubble_img = get_base64_image("assets/text_bubble.png")

# ---------- FOLLOW PAGE 자동 전환 ----------
if st.session_state.qr_start_time is not None:
    if time.time() - st.session_state.qr_start_time >= 2.0:
        st.switch_page("pages/3-1_follow_stage.py")

# =============================
# CSS
# =============================
st.markdown("""
<style>

@import url('https://fonts.googleapis.com/css2?family=Jua&display=swap');
* { font-family: "Jua", sans-serif !important; }

.stApp { background-color: #102A4C !important; }
header, .stToolbar { display: none !important; }

/* 전체 래퍼: 화면 중앙에 배치 */
.page-wrapper {
    width: 80vw;
    height: 75vh;
    position: relative;
    display: flex;
    justify-content: center;
    align-items: center;
}

/* 좌측 하단 캐릭터 */
.character-img {
    position: absolute;
    bottom: 0%;
    left: 5%;
    width: 60vw;
    max-width: 600px;
    height: auto;
    margin-bottom: -100px;
}

/* 우측 상단 말풍선 */
.speech-wrapper {
    position: absolute;
    top: 0%;
    right: 30%;
    width: 33vw;
    max-width: 420px;
    margin-top: -200px;
}

/* 말풍선 이미지 */
.speech-img {
    width: 95vw;
    max-width: 950px;
    height: auto;
}

/* 말풍선 내부 텍스트 */
.speech-text {
    position: absolute;
    top: 23%;
    left: 55%;
    width: 120%;
    font-size: 2.6rem;
    line-height: 1.35;
    color: #0E2C55;
    text-align: center;
    margin-top: 100px;
}

/* 말풍선 하단 텍스트 추가 */
.speech-text-bottom {
    position: absolute;
    top: 75%;         /* 말풍선 하단 위치 조정 */
    left: 25%;
    width: 100%;
    font-size: 2.0rem;
    line-height: 1.4;
    color: #FFFFFF;  
    text-align: center;
    margin-top: 100px;
}

</style>
""", unsafe_allow_html=True)

# =============================
# HTML
# =============================
st.html(f"""
<div class="page-wrapper">

    <!-- 좌측 하단 캐릭터 -->
    <img src="data:image/png;base64,{body_img}" class="character-img">

    <!-- 우측 상단 말풍선 + 텍스트 -->
    <div class="speech-wrapper">
        <img src="data:image/png;base64,{speechbubble_img}" class="speech-img">
        <div class="speech-text">
            안녕하세요!<br>
            저는 메디버디입니다.<br><br>
            병원에서 길을 안내해드려요<br>
            저에게 말을 걸어주세요
        </div>
    </div>

    <div class="speech-text-bottom">
        저는 음성으로 알아들을 수 있어요!<br>
        "메디버디, 약에 대해 궁금한게 있어"<br>
        "메디버디, 방사선실까지 안내해줘"
    </div>

</div>
""")

# ======= 자동 rerun =======
time.sleep(0.08)
st.rerun()