import streamlit as st
import base64

st.set_page_config(layout="wide")

result = "던파사 캡슐"

# Base64 이미지 로드 함수
def get_base64_image(path):
    try:
        with open(path, "rb") as f:
            return base64.b64encode(f.read()).decode()
    except:
        return None

body_img = get_base64_image("assets/body_medicine.png")

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

.character-img {{
    width: 24vw;
    max-width: 240px;
    margin-top: 20px;
}}

.guide-text-top {{
    font-size: 2.0rem;
    color: #0E2C55;
    margin-top: 50px;
    margin-bottom: 50px;
    text-align: center;
}}
                
.guide-text-center {{
    font-size: 3.6rem;
    color: #0E2C55;
    margin-bottom: 50px;
    text-align: center;
}}
                
.guide-text-bottom {{
    font-size: 2.4rem;
    color: #0E2C55;
    margin-bottom: 20px;
    text-align: center;
}}

</style>
""", unsafe_allow_html=True)
# =============================
# HTML
# =============================
st.html(f"""
<div class="inner-wrapper">

    <div class="guide-text-top">
        자동 인식 결과:
    </div>

    <div class="guide-text-center">
        {result}
    </div>

    <div class="guide-text-bottom">
        쉬운말로 알려드릴게요
    </div>

    <img src="data:image/png;base64,{body_img}" class="character-img">
        
</div>
""")