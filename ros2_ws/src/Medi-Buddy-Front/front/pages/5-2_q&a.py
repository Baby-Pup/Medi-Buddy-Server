import streamlit as st
import base64
import json
import time
import os
import streamlit.components.v1 as components

st.set_page_config(layout="wide")

FILE_PATH = "/tmp/robot_ui_status.json"

# Base64 이미지
def get_base64_image(path):
    try:
        with open(path, "rb") as f:
            return base64.b64encode(f.read()).decode()
    except:
        return None

body_img = get_base64_image("assets/body_star.png")

def read_llm():
    if os.path.exists(FILE_PATH):
        with open(FILE_PATH) as f:
            data = json.load(f)
        return data.get("llm_result", "")
    return ""

# llm 결과
result = read_llm()

# 마침표마다 줄바꿈 적용
result_html = result.replace(".", ".<br><br>")

elapsed = time.time() - st.session_state.question_start_time

# ========== 6초 후 자동 페이지 이동 ==========
if elapsed >= 10.0:
    st.switch_page("pages/1_map.py")

# =============================
# ⭐ HTML + CSS + JS 직접 구성
# =============================
st.markdown(f"""
<style>
@import url('https://fonts.googleapis.com/css2?family=Jua&display=swap');

html, body, .stApp, .block-container, .main {{
    background-color: #102A4C !important;
}}
header, .stToolbar {{ display: none !important; }}

* {{
    font-family: "Jua", sans-serif !important;
}}

.inner-wrapper {{
    background: #F7F3EB;
    width: 100%;
    height: 83dvh;
    display: flex;
    flex-direction: column;
    justify-content: flex-start;
    padding-top: 60px;
    position: relative;
}}

.character-img {{
    width: 34vw;
    max-width: 340px;
    position: absolute;
    top: 10px;
    right: 50px;
}}

.guide-text {{
    font-size: 2.4rem;
    color: #0E2C55;
    text-align: left;
    max-width: 70%;           /* 텍스트가 이미지와 겹치지 않게 */
    margin-left: 5%;
    line-height: 1.5;
}}

</style>
""", unsafe_allow_html=True)

st.html(f"""
<div class="inner-wrapper">

    <!-- 텍스트 영역 -->
    <div class="guide-text">
        {result_html}
    </div>

    <!-- 캐릭터 이미지 (우측 상단 고정) -->
    <img src="data:image/png;base64,{body_img}" class="character-img">

</div>
""")

# ======= 자동 rerun =======
time.sleep(0.08)
st.rerun()