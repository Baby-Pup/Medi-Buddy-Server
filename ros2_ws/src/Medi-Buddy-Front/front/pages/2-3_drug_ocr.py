import streamlit as st
import base64
import streamlit.components.v1 as components
import cv2
from flask import Flask, Response
import threading

st.set_page_config(layout="wide")

# ================================================
# 1) Base64 이미지 로더
# ================================================
def get_base64_image(path):
    try:
        with open(path, "rb") as f:
            return base64.b64encode(f.read()).decode()
    except:
        return None

body_img = get_base64_image("assets/face_smile.png")

# ================================================
# 2) Flask 기반 MJPEG 스트리밍 서버 (서버 PC 카메라)
# ================================================
flask_app = Flask(__name__)

def gen_frames():
    cap = cv2.VideoCapture(0)          # 서버 PC 카메라 캡처
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    while True:
        success, frame = cap.read()
        if not success:
            continue
        _, buffer = cv2.imencode('.jpg', frame)
        frame_bytes = buffer.tobytes()

        # MJPEG 스트림 형식
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@flask_app.route('/camera')
def video_feed():
    return Response(gen_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

def run_flask():
    flask_app.run(host="0.0.0.0", port=9090, debug=False, use_reloader=False)

# Flask 스트리밍 서버를 백그라운드에서 실행
threading.Thread(target=run_flask, daemon=True).start()


# ================================================
# 3) Streamlit 스타일 설정
# ================================================
st.markdown("""
<style>
html, body, .stApp, .block-container, .main {
    background-color: #102A4C !important;
}
</style>
""", unsafe_allow_html=True)

# ================================================
# 4) HTML + CSS + MJPEG 카메라 표시
# ================================================
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

.camera-square img {{
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
            읽고 싶은 글씨를<br>
            화면에 보여주세요!
        </div>
    </div>

    <div class="right-area">
        <div class="camera-square">
            <!-- ⭐ 서버 카메라 MJPEG 스트림 표시 -->
            <img src="http://192.168.0.14:9090/camera">
        </div>
    </div>

</div>
</body>
</html>
""", height=770, scrolling=False)
