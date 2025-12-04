import streamlit as st
import base64
import json
import time
import os

st.set_page_config(layout="wide")

# =========================================================
# 🔧 세션 초기화
# =========================================================
if "test_target" not in st.session_state:
    st.session_state.test_target = "none"

if "preview_target" not in st.session_state:
    st.session_state.preview_target = "none"


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
# 🗺 병원 지도 좌표
# =========================================================
map_points = {
    "hospital_entrance": {"left": 40.0, "top": 87.0},  # 입구 고정 좌표

    "x_ray_room":       {"left": 24.9, "top": 13.9},
    "emergency_room":   {"left": 61.7, "top": 13.9},
    "restroom":         {"left": 90.2, "top": 26.7},
    "pharmacy":         {"left": 19.3, "top": 47.2},
    "reception":        {"left": 49.7, "top": 48.7},
    "blood_draw_room":  {"left": 65.7, "top": 69.6},
}


def safe_point(key):
    if key not in map_points:
        st.sidebar.error(f"⚠ map_points['{key}'] 없음")
        return None
    return map_points[key]


# =========================================================
# 🔥 직각 이동 waypoints
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
}

# ⭐ 입구 → 각 방 waypoints
waypoints.update({

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

})


# =========================================================
# JSON 상태 불러오기
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

status = data.get("status", "")
client_name = data.get("client_name", "No Name")
date_str = data.get("date", "")
destinations_raw = data.get("destinations", "")
detour_req = data.get("detour", "")
current_dest = data.get("current_destination", "").strip()


# =========================================================
# 🧪 테스트 버튼 패널 (session_state 기반)
# =========================================================
st.sidebar.title("🚀 Test Mode")

selected = st.sidebar.radio(
    "이동 테스트",
    ["none", "x_ray_room", "emergency_room", "restroom",
     "pharmacy", "reception", "blood_draw_room"]
)

# 선택값 → session_state 저장
st.session_state.test_target = selected


# =========================================================
# ───────────── route 구성 로직 ─────────────
# =========================================================
if st.session_state.test_target != "none":
    # 테스트 모드 → route 강제 설정
    route = ["hospital_entrance", st.session_state.test_target]
    current_dest = "hospital_entrance"
    detour_req = "none"
else:
    # 실제 JSON 목적지 리스트 파싱
    dest_list = [r.strip() for r in destinations_raw.split(",")] if destinations_raw else []

    # current_destination이 목적지 리스트 안에 있어야 사용 가능
    if current_dest and current_dest in dest_list:

        idx = dest_list.index(current_dest)

        # 출발지 결정
        if idx == 0:
            start_point = "hospital_entrance"
        else:
            start_point = dest_list[idx - 1]

        # 도착지 = 현재 목적지
        end_point = current_dest

        route = [start_point, end_point]

    else:
        # fallback: 첫 목적지로만 이동
        route = ["hospital_entrance"] + dest_list[:1]



# =========================================================
# 🛰 Waypoint 미리보기 (session_state 기반)
# =========================================================
st.sidebar.markdown("---")
st.sidebar.subheader("🛰 Waypoint 미리보기")

preview = st.sidebar.selectbox(
    "입구 → 목적지 경로",
    ["none", "pharmacy", "reception", "blood_draw_room",
     "x_ray_room", "emergency_room", "restroom"]
)

st.session_state.preview_target = preview

if preview != "none":
    key = ("hospital_entrance", preview)
    st.sidebar.success(f"입구 → {preview} 경로")

    if key in waypoints:
        st.sidebar.write("**좌표 리스트:**")
        for i, p in enumerate(waypoints[key]):
            st.sidebar.write(f"{i+1}. left={p['left']} , top={p['top']}")
        st.sidebar.code(json.dumps(waypoints[key], indent=2))
    else:
        st.sidebar.warning("⚠ waypoints 없음")


# =========================================================
# 🔥 애니메이션 full_path 생성
# =========================================================
full_path = []
keyframes = ""
animation_css = ""

if len(route) >= 2:
    s_name = route[0]
    e_name = route[1]

    s = safe_point(s_name)
    e = safe_point(e_name)

    if s and e:
        full_path.append(s)

        if (s_name, e_name) in waypoints:
            full_path.extend(waypoints[(s_name, e_name)])

        full_path.append(e)


# keyframes 생성
if len(full_path) >= 2:
    step = 100 / (len(full_path) - 1)

    keyframes = "@keyframes buddyMove {\n"
    for i, p in enumerate(full_path):
        keyframes += f"{round(i*step, 2)}% {{ top:{p['top']}%; left:{p['left']}%; }}\n"
    keyframes += "}\n"

    animation_css = "animation: buddyMove 7s linear forwards;"

# ---------- QR START EVENT ----------
if status == "audio_incoming":
    # 즉시 3-3_follow_stage 로 전환
    st.switch_page("pages/3-3_follow_stage.py")
    
# =========================================================
# CSS 적용
# =========================================================
st.markdown(f"""
<style>
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
korean_names = {
    "restroom": "화장실",
    "x_ray_room": "X-ray실",
    "emergency_room": "응급실",
    "pharmacy": "약국",
    "reception": "수납처",
    "blood_draw_room": "채혈실",
    "hospital_entrance": "병원 입구"
}

display_all_dest = [korean_names.get(d, d) for d in dest_list]

st.html(f"""
<div style="display:flex; justify-content:center; margin-top:40px;">
  <div style="width:92%; max-width:1400px; background:#0E2C55;
              padding:60px; border-radius:25px;">

    <div style="background:#F7F3EB; padding:60px 50px;
                border-radius:18px; display:grid;
                grid-template-columns:45% 55%; gap:10px;">

      <!-- left info -->
      <div style="position:relative;">
        <img src="data:image/png;base64,{face_img}" style="width:140px;">
        <div style="font-size:40px; margin-top:10px;">Personal Medical MAP</div>
        <div style="font-size:24px; margin:20px 0 25px;">
          {date_str}<br>{client_name}
        </div>

        <div style="font-size:24px; line-height:1.8;">
          {"<br>".join(display_all_dest)}
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
