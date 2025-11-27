import streamlit as st
import base64

st.set_page_config(layout="wide")

# Base64 이미지 인코딩 함수
def get_base64_image(path):
    try:
        with open(path, "rb") as f:
            return base64.b64encode(f.read()).decode()
    except:
        return None

body_img = get_base64_image("assets/body_lost.png")

# ============================================
# 🎨 CSS + HTML — Streamlit 내부에서 직접 렌더링
# ============================================
st.markdown(f"""
<style>
@import url('https://fonts.googleapis.com/css2?family=Jua&display=swap');

html, body, .stApp, .block-container, .main {{
    background-color: #102A4C !important;
}}

* {{
    font-family: "Jua", sans-serif !important;
}}

.inner-wrapper {{
    background: #F7F3EB;
    width: 100%;
    height: 83dvh;
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-content: center;
}}

/* 404 텍스트 */
.error-title {{
    font-size: 9.0rem;
    font-weight: 700;
    color: #000;
    margin-top: 0px;
    text-align: center;
}}
            
.character-img {{
    width: 40vw;
    max-width: 400px;
    margin-top: -90px;
}}

/* 안내 문구 */
.error-text {{
    margin-top: -20px;
    font-size: 32px;
    color: #000;
}}

/* 버튼 느낌 텍스트 */
.error-btn {{
    margin-top: 30px;
    padding: 14px 34px;
    background-color: #496A90;
    color: white;
    border-radius: 30px;
    font-size: 22px;
    width: fit-content;
}}s

</style>
""", unsafe_allow_html=True)

# ============================
# HTML 렌더링
# ============================
st.html(f"""
<div class="inner-wrapper">

    <div class="error-title">
        404
    </div>

    <!-- 길 잃은 메디버디 이미지 -->
    <img src="data:image/png;base64,{body_img}" class="character-img">

    <div class="error-text">
        Medi-Buddy가 길을 잃었어요
    </div>

    <div class="error-btn">
        관리자 오는중…
    </div>

</div>
""")
