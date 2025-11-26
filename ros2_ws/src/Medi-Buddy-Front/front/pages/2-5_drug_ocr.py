import streamlit as st
import base64
import streamlit.components.v1 as components

st.set_page_config(layout="wide")

# AI 모델 결과 (예시)
result = "잇몸이 붓고 피가 나나요? 그건 염증이 있다는 신호에요.이 약은 잇몸 염증을 가라앉힐 때 보조해주는 약입니다.성인 기준 식후에 1알, 하루 세번 섭취를 권장해요.유당 성분이 포함돼 있어서, 우유 및 포도당이 맞지 않는다면 섭취금지.1개월 이상 복용했는데도 증상이 그대로라면 의사와 상의해주세요.우측 QR 코드로 약 정보를 그대로 받아보실 수 있어요!추가로 궁금하신 정보가 있으시다면 다시 말씀해주세요"

# 마침표마다 줄바꿈 적용
result_html = result.replace(".", ".<br><br>")

# Base64 이미지
def get_base64_image(path):
    try:
        with open(path, "rb") as f:
            return base64.b64encode(f.read()).decode()
    except:
        return None

body_img = get_base64_image("assets/body_star.png")

# =============================
# ⭐ HTML + CSS + JS 직접 구성
# =============================
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
