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
# 🗺 병원 지도 좌표 (입구 포함)
# =========================================================
map_points = {
    "hospital_entrance": {"left": 40.0, "top": 87.0},   # ⭐ 추가된 입구 위치

    "x_ray_room":       {"left": 24.9, "top": 13.9},
    "emergency_room":   {"left": 61.7, "top": 13.9},
    "restroom":         {"left": 90.2, "top": 26.7},
    "pharmacy":         {"left": 19.3, "top": 47.2},
    "reception":        {"left": 49.7, "top": 48.7},
    "blood_draw_room":  {"left": 65.7, "top": 69.6},
}

# =========================================================
# 🔥 직각 이동 waypoints (입구 → 장소 포함)
# =========================================================
waypoints = {
    ("blood_draw_room", "x_ray_room"): [
        {"left": 65.7, "top": 40},
        {"left": 24.9, "top": 40},
    ],
    ("x_ray_room", "reception"): [
        {"left": 24.9, "top": 30},
        {"left": 49.7, "top": 30},
    ],
    ("blood_draw_room", "reception"): [
        {"left": 65.7, "top": 60},
    ],
    ("reception", "blood_draw_room"): [
        {"left": 49.7, "top": 60},
        {"left": 65.7, "top": 60},
    ],
    ("pharmacy", "reception"): [
        {"left": 19.3, "top": 48.7},
        {"left": 49.7, "top": 48.7},
    ],

    # ⭐ 입구 → 각 방
    ("hospital_entrance", "pharmacy"): [
        {"left": 40.0, "top": 60.0},
        {"left": 19.3, "top": 60.0},
    ],
    ("hospital_entrance", "reception"): [
        {"left": 40.0, "top": 60.0},
        {"left": 49.7, "top": 60.0},
    ],
    ("hospital_entrance", "blood_draw_room"): [
        {"left": 50.0, "top": 87.0},
        {"left": 50.0, "top": 70.0},
        {"left": 65.7, "top": 70.0},
    ],
    ("hospital_entrance", "x_ray_room"): [
        {"left": 40.0, "top": 60.0},
        {"left": 24.9, "top": 60.0},
        {"left": 24.9, "top": 13.9},
    ],
    ("hospital_entrance", "emergency_room"): [
        {"left": 40.0, "top": 60.0},
        {"left": 61.7, "top": 60.0},
        {"left": 61.7, "top": 13.9},
    ],
    ("hospital_entrance", "restroom"): [
        {"left": 40.0, "top": 60.0},
        {"left": 90.2, "top": 60.0},
    ],
}

# =========================================================
# JSON 상태 로드
# =========================================================
FILE_PATH = "/tmp/robot_ui_status.json"

def read_status():
    if not os.path.exists(FILE_PATH):
        return {}
    try:
        with open(FILE_PATH, "r") as f:
            return json.loads(f.read().strip())
    except:
        return {}

data = read_status()

# =========================================================
# JSON 파싱
# =========================================================
client_name = data.get("client_name", "No Name")
date_str = data.get("date", "")
destinations_raw = data.get("destinations", "")
status = data.get("status", "")
detour_req = data.get("detour", "")
current_dest = data.get("current_destination", "").strip()

# route 파싱
route = [r.strip() for r in destinations_raw.split(",")] if destinations_raw else []

# =========================================================
# ⭐ 입구에서 시작하도록 route 수정
# =========================================================
# detour(화장실 우회)일 경우
if detour_req and detour_req != "none":
    bathroom_mode = True
    route = ["hospital_entrance", "restroom"]
else:
    bathroom_mode = False

    # 원래 route가 있을 경우 → 맨 앞에 hospital_entrance 추가
    if len(route) >= 1 and route[0] != "hospital_entrance":
        route = ["hospital_entrance"] + route


# =========================================================
# 🔥 경로 애니메이션 생성
# =========================================================
full_path = []
keyframes = ""
animation_css = ""

if bathroom_mode:
    pos = map_points["restroom"]
    keyframes = f"""
    @keyframes buddyBounce {{
      0%   {{ top: {pos['top'] - 2}%; left: {pos['left']}%; }}
      50%  {{ top: {pos['top'] + 2}%; left: {pos['left']}%; }}
      100% {{ top: {pos['top'] - 2}%; left: {pos['left']}%; }}
    }}
    """
    animation_css = "animation: buddyBounce 1.2s infinite ease-in-out;"
else:
    if len(route) >= 2:
        s = route[0]
        e = route[1]

        full_path.append(map_points[s])
        if (s, e) in waypoints:
            full_path.extend(waypoints[(s, e)])
        full_path.append(map_points[e])

    if full_path:
        step = 100 / (len(full_path) - 1)
        keyframes = "@keyframes moveBuddy {\n"
        for i, p in enumerate(full_path):
            keyframes += f"{round(i * step, 2)}% {{ top:{p['top']}%; left:{p['left']}%; }}\n"
        keyframes += "}\n"
        animation_css = "animation: moveBuddy 8s linear forwards;"


# =========================================================
# ⭐ CSS 적용 (입구에서 시작하도록 초기 위치 고정)
# =========================================================
st.markdown(f"""
<style>
@import url('https://fonts.googleapis.com/css2?family=Jua&display=swap');
* {{ font-family: "Jua"; }}

.small-buddy {{
    width:100px;
    position:absolute;
    transform:translate(-50%, -50%);
    top:87%;        /* ⭐ hospital_entrance 위치 */
    left:40%;
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

      <!-- left info box -->
      <div style="position:relative;">
        <img src="data:image/png;base64,{face_img}" style="width:140px;">
        <div style="font-size:40px; margin-top:10px;">Personal Medical MAP</div>

        <div style="font-size:24px; margin:20px 0 25px;">
          {date_str}<br>
          {client_name}'s Medical Route
        </div>

        <div style="font-size:24px; line-height:1.8;">
          {"<br>".join(route)}
        </div>

        <img src="data:image/png;base64,{big_buddy}"
             style="width:180px; position:absolute; bottom:0; left:0;">
      </div>

      <!-- map -->
      <div style="position:relative;">
        <img src="data:image/png;base64,{map_img}"
             style="width:100%; border-radius:12px;">
        <img src="data:image/png;base64,{small_buddy}" class="small-buddy">
      </div>

    </div>

  </div>
</div>
""")

time.sleep(0.2)
st.rerun()
