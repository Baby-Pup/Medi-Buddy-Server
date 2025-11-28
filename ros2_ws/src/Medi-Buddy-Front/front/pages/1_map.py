import streamlit as st
import base64
import time
import requests

st.set_page_config(layout="wide")

# =========================================================
# Base64 이미지 로더
# =========================================================
def img64(path):
    try:
        with open(path, "rb") as f:
            return base64.b64encode(f.read()).decode()
    except:
        return None

face_img = img64("assets/face_smile.png")
big_buddy = img64("assets/body_flag.png")
small_buddy = img64("assets/body_flag.png")
map_img = img64("assets/map_line.png")

# =========================================================
# 병원 지도 좌표 (%)
# =========================================================
map_points = {
    "X-ray실":  {"left": 24.9, "top": 13.9},
    "응급실":   {"left": 61.7, "top": 13.9},
    "화장실":   {"left": 90.2, "top": 26.7},
    "약국":     {"left": 19.3, "top": 47.2},
    "수납":     {"left": 49.7, "top": 48.7},
    "채혈실":   {"left": 65.7, "top": 69.6},
}

# =========================================================
# 🔥 직각 이동 waypoints
# =========================================================
waypoints = {
    ("채혈실", "X-ray실"): [
        {"left": 65.7, "top": 40},
        {"left": 24.9, "top": 40},
    ],

    ("X-ray실", "수납"): [
        {"left": 24.9, "top": 30},
        {"left": 49.7, "top": 30},
    ],

    # 자연스러운 ㄱ자 이동
    ("채혈실", "수납"): [
        {"left": 65.7, "top": 60},
    ],

    ("수납", "채혈실"): [
        {"left": 49.7, "top": 60},
        {"left": 65.7, "top": 60},
    ],

    ("약국", "수납"): [
        {"left": 19.3, "top": 48.7},
        {"left": 49.7, "top": 48.7},
    ],
}

# =========================================================
# 상태값 초기화
# =========================================================
session = st.session_state

if "qr_data" not in session:
    # 초기값 (실제 QR로 대체)
    session["qr_data"] = {
        "name": "정지아",
        "date": "2025년 11월 28일",
        "route": ["채혈실", "X-ray실", "수납"]
    }

if "route_original" not in session:
    session["route_original"] = session["qr_data"]["route"]

if "route_current" not in session:
    session["route_current"] = session["qr_data"]["route"]

if "bathroom_mode" not in session:
    session["bathroom_mode"] = False

if "face_detected" not in session:
    session["face_detected"] = False

if "anim_speed" not in session:
    session["anim_speed"] = 8   # ⬅ 매우 느림 속도 적용 (8초)

# =========================================================
# 📡 FastAPI 폴링
# =========================================================
FACE_URL = "http://127.0.0.1:8000/face-status"
VOICE_URL = "http://127.0.0.1:8000/voice"
QR_URL   = "http://127.0.0.1:8000/qr"

# 얼굴 인식 polling
try:
    res = requests.get(FACE_URL, timeout=0.2).json()
    if res.get("face_detected"):
        session["face_detected"] = True
except:
    pass

# 음성 명령 polling
try:
    res = requests.get(VOICE_URL, timeout=0.2).json()
    if res.get("go_bathroom"):
        session["bathroom_mode"] = True
        session["route_current"] = ["화장실"]
        session["face_detected"] = False
except:
    pass

# QR polling
try:
    res = requests.get(QR_URL, timeout=0.2).json()
    if res.get("route"):
        session["qr_data"] = res
        session["route_original"] = res["route"]

        if not session["bathroom_mode"]:
            session["route_current"] = res["route"]
except:
    pass


# =========================================================
# 얼굴 인식 → 화장실 종료 → 경로 복귀
# =========================================================
if session["bathroom_mode"] and session["face_detected"]:
    session["bathroom_mode"] = False
    session["route_current"] = session["route_original"]
    session["face_detected"] = False
    st.rerun()

# =========================================================
# 현재 경로
# =========================================================
route = session["route_current"]

# =========================================================
# 🔥 애니메이션 keyframes 생성
# =========================================================
if session["bathroom_mode"]:
    pos = map_points["화장실"]

    keyframes = f"""
    @keyframes buddyBounce {{
      0%   {{ top: {pos['top'] - 2}%; left: {pos['left']}%; }}
      50%  {{ top: {pos['top'] + 2}%; left: {pos['left']}%; }}
      100% {{ top: {pos['top'] - 2}%; left: {pos['left']}%; }}
    }}
    """
    animation_css = "animation: buddyBounce 1s infinite ease-in-out;"

else:
    full_path = []

    for i in range(len(route) - 1):
        s = route[i]
        e = route[i + 1]

        full_path.append(map_points[s])

        if (s, e) in waypoints:
            full_path.extend(waypoints[(s, e)])

        full_path.append(map_points[e])

    if not full_path:
        full_path = [map_points[route[0]]]

    step = 100 / (len(full_path) - 1)

    keyframes = "@keyframes moveBuddy {\n"
    for i, p in enumerate(full_path):
        keyframes += f"{round(i * step, 2)}% {{ top:{p['top']}%; left:{p['left']}%; }}\n"
    keyframes += "}\n"

    animation_css = f"animation: moveBuddy {session['anim_speed']}s infinite alternate ease-in-out;"


# Inject CSS
st.markdown(f"""
<style>
{keyframes}
.small-buddy {{
    width:100px;
    position:absolute;
    transform:translate(-50%, -50%);
    {animation_css}
}}
</style>
""", unsafe_allow_html=True)

# =========================================================
# 텍스트 부분
# =========================================================
order_html = "".join([f"{i+1}. {r}<br>" for i, r in enumerate(route)])
title_text = (
    f"{session['qr_data']['name']}님 화장실 대기 중"
    if session["bathroom_mode"]
    else f"{session['qr_data']['name']}님 진료 순서표"
)

# =========================================================
# 메인 UI
# =========================================================
st.html(f"""
<div style="display:flex; justify-content:center; margin-top:40px;">
  <div style="width:92%; max-width:1400px; background:#0E2C55;
              padding:60px; border-radius:25px;">

    <div style="background:#F7F3EB; padding:60px 50px;
                border-radius:18px; display:grid;
                grid-template-columns:45% 55%; gap:10px;">

      <!-- 왼쪽 정보 -->
      <div style="position:relative;">
        <img src="data:image/png;base64,{face_img}" style="width:140px;">
        <div style="font-size:40px; margin-top:10px;">개인 진료 MAP</div>

        <div style="font-size:24px; margin:20px 0 25px;">
          {session['qr_data']['date']}<br>
          {title_text}
        </div>

        <div style="font-size:24px; line-height:1.8;">
          {order_html}
        </div>

        <img src="data:image/png;base64,{big_buddy}"
             style="width:180px; position:absolute; bottom:0; left:0;">
      </div>

      <!-- 오른쪽 지도 -->
      <div style="position:relative;">
        <img src="data:image/png;base64,{map_img}"
             style="width:100%; border-radius:12px;">
        <img src="data:image/png;base64,{small_buddy}" class="small-buddy">
      </div>

    </div>

  </div>
</div>
""")
