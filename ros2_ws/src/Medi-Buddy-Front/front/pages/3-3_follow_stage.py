import streamlit as st
import base64
import json
import time
import os

st.set_page_config(layout="wide")

FILE_PATH = "/tmp/robot_ui_status.json"

# Base64 이미지 인코딩
def get_base64_image(path):
    try:
        with open(path, "rb") as f:
            return base64.b64encode(f.read()).decode()
    except:
        return None

body_img = get_base64_image("assets/body_surprize.png")

def read_status():
    if os.path.exists(FILE_PATH):
        with open(FILE_PATH) as f:
            data = json.load(f)
        return data.get("status", "")
    return ""


status = read_status()

if status == "null":
    # 즉시 3-3_follow_stage 로 전환
    st.switch_page("pages/미정.py")
elif status == "detour":
    # 즉시 3-3_follow_stage 로 전환
    st.switch_page("pages/미정.py")
elif status == "ocr":
    # 즉시 2-1_drug_ocr 로 전환
    st.switch_page("pages/2-1_drug_ocr.py")
elif status == "question":
    # 즉시 2-1_drug_ocr 로 전환
    st.switch_page("pages/미정.py")

# =============================
# CSS (캐릭터 위로 이동 + 박스 확대 + 위치 조정)
# =============================
st.markdown("""
<style>

@import url('https://fonts.googleapis.com/css2?family=Jua&display=swap');

* { font-family: "Jua", sans-serif !important; }

.stApp { background-color: #102A4C !important; }

/* 전체 높이는 유지하되 캐릭터/박스를 조금 위로 이동 */
.page-wrapper {
    height: 100vh;
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-content: center;
    transform: translateY(-6%);     /* 🔥 전체 콘텐츠 위로 이동 */
}

/* 캐릭터 */
.character-img {
    width: 45vw;
    max-width: 520px;
    height: auto;
    margin-bottom: 25px;   /* 박스와 적당히 간격 */
    transform: translateX(-6%);   /* 🔥 아주 살짝 왼쪽으로 이동 이미지 자체가 우측으로 쏠려있음*/
}

/* 텍스트 박스*/
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

    <!-- 캐릭터 이미지 -->
    <img src="data:image/png;base64,{body_img}" class="character-img">

    <!-- 텍스트 박스 -->
    <div class="bottom-box">
        귀 기울이고 있습니다. 말씀하세요!
    </div>

</div>
""")

# ========== 자동 rerun ==========
time.sleep(0.08)
st.rerun()