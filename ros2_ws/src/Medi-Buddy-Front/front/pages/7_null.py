import streamlit as st
import base64

st.set_page_config(layout="wide")

# Base64 인코딩 함수
def get_base64_image(path):
    try:
        with open(path, "rb") as f:
            return base64.b64encode(f.read()).decode()
    except:
        return None

body_img = get_base64_image("assets/body_book.png")
speechbubble_img = get_base64_image("assets/text_bubble.png")

# =============================
# CSS
# =============================
st.markdown("""
<style>

@import url('https://fonts.googleapis.com/css2?family=Jua&display=swap');
* { font-family: "Jua", sans-serif !important; }

.stApp { background-color: #102A4C !important; }

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
    left: 20%;
    width: 32vw;
    max-width: 380px;
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
    width: 90vw;
    max-width: 900px;
    height: auto;
}

/* 말풍선 내부 텍스트 */
.speech-text {
    position: absolute;
    top: 25%;
    left: 60%;
    width: 100%;
    font-size: 2.6rem;
    line-height: 1.35;
    color: #0E2C55;
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
            제가 알지 못하는 정보에요<br>
            조금 더 공부해볼게요!<br><br>
            다른 질문이 있으실까요?
        </div>
    </div>

</div>
""")
