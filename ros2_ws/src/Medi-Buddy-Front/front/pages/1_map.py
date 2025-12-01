import streamlit as st
import base64
import json
import time
import os

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
# JSON 파일 로드
# =========================================================
FILE_PATH = "/tmp/robot_ui_status.json"

def read_status():
    if not os.path.exists(FILE_PATH):
        return {}

    try:
        with open(FILE_PATH, "r") as f:
            data = json.loads(f.read().strip())
            return data
    except:
        return {}

data = read_status()

# =========================================================
# JSON에서 상태 불러오기
# =========================================================
client_name = data.get("client_name", "이름 없음")
date_str = data.get("date", "")
destinations_raw = data.get("destinations", "")
status = data.get("status", "")
detour_req = data.get("detour", "")
current_dest = data.get("current_destination", "")

# 경로 리스트
route = destinations_raw.split(",") if destinations_raw else []

# =========================================================
# 화장실/엘리베이터 우회 모드
# =========================================================
bathroom_mode = False
if detour_req and detour_req != "none":
    bathroom_mode = True
    route = ["화장실"]  # 우회 목적지 고정

# =========================================================
# 🔥 애니메이션 경로 생성
# =========================================================
if bathroom_mode:
    # 화장실에서 통통 튀는 모션
    pos = map_points["화장실"]
    keyframes = f"""
    @keyframes buddyBounce {{
      0%   {{ top: {pos['top'] - 2}%; left: {pos['left']}%; }}
      50%  {{ top: {pos['top'] + 2}%; left: {pos['left']}%; }}
      100% {{ top: {pos['top'] - 2}%; left: {pos['left']}%; }}
    }}
    """
    animation_css = "animation: buddyBounce 1.2s infinite ease-in-out;"

else:
    full_path = []
    if len(route) >= 1:
        for i in range(len(route) - 1):
            s = route[i]
            e = route[i + 1]

            full_path.append(map_points[s])

            if (s, e) in waypoints:
                full_path.extend(waypoints[(s, e)])

            full_path.append(map_points[e])
    else:
        full_path = [map_points[current_dest]] if current_dest in map_points else []

    if full_path:
        step = 100 / (len(full_path) - 1)
        keyframes = "@keyframes moveBuddy {\n"
        for i, p in enumerate(full_path):
            keyframes += f"{round(i * step, 2)}% {{ top:{p['top']}%; left:{p['left']}%; }}\n"
        keyframes += "}\n"
        animation_css = f"animation: moveBuddy 10s infinite alternate ease-in-out;"
    else:
        # fallback 정적 표시
        keyframes = ""
        animation_css = ""

# =========================================================
# 순서표 텍스트
# =========================================================
order_html = "".join([f"{i+1}. {r}<br>" for i, r in enumerate(route)])
title_text = (
    f"{client_name}님 화장실 이동 중"
    if bathroom_mode else
    f"{client_name}님의 진료 순서표"
)

# =========================================================
# CSS 적용
# =========================================================
st.markdown(f"""
<style>
@import url('https://fonts.googleapis.com/css2?family=Jua&display=swap');
* {{ font-family: "Jua"; }}

.small-buddy {{
    width:100px;
    position:absolute;
    transform:translate(-50%, -50%);
    {animation_css}
}}

{keyframes}
</style>
""", unsafe_allow_html=True)

# =========================================================
# UI 출력
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
          {date_str}<br>
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

# 자동 업데이트
time.sleep(0.2)
st.rerun()
