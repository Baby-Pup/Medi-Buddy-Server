import streamlit as st
import base64
import streamlit.components.v1 as components

st.set_page_config(layout="wide")

# Base64 이미지
def get_base64_image(path):
    try:
        with open(path, "rb") as f:
            return base64.b64encode(f.read()).decode()
    except:
        return None

body_img = get_base64_image("assets/face_sad.png")

# =============================
# ⭐ HTML + CSS + JS 직접 구성
# =============================
st.markdown("""
<style>
html, body, .stApp, .block-container, .main {
    background-color: #102A4C !important;
}
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
    margin-bottom: -40px;
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
            정보를 읽지 못했어요.<br>
            다시 한번 보여주세요
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
