#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import base64
import time
import os
import subprocess
import soundfile as sf
import numpy as np
from openai import OpenAI
from dotenv import load_dotenv
import tensorflow as tf
from transformers import AutoTokenizer, TFAutoModelForSequenceClassification

load_dotenv()

########################################################
# 1. Intent 모델 로드
########################################################
model = TFAutoModelForSequenceClassification.from_pretrained(
    '/home/ubuntu/ros2_ws/src/medi_buddy_server/intent_model'
)
tokenizer = AutoTokenizer.from_pretrained(
    '/home/ubuntu/ros2_ws/src/medi_buddy_server/intent_model'
)

label_names = {
    0: "무관한내용",
    1: "경로",
    2: "OCR요청",
    3: "약정보",
    4: "의료정보"
}

def predict(text):
    inputs = tokenizer(
        text,
        max_length=128,
        padding='max_length',
        truncation=True,
        return_tensors='tf'
    )
    outputs = model(inputs)
    logits = outputs.logits

    probabilities = tf.nn.softmax(logits, axis=-1)
    predicted_label = tf.argmax(probabilities, axis=-1).numpy()[0]
    confidence = tf.reduce_max(probabilities).numpy()

    return {
        'label': predicted_label,
        'label_name': label_names[predicted_label],
        'confidence': float(confidence)
    }


########################################################
# 2. LLM 호출
########################################################
def llm(sys_message, query):
    client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
    re = client.chat.completions.create(
        model='gpt-3.5-turbo',
        messages=[
            {"role": "system", "content": "당신은 의료 지원 로봇 Medi-Buddy입니다." + sys_message},
            {"role": "user", "content": query}
        ]
    ).choices[0].message.content

    llm_pub_node.publish_result(re)
    return re


########################################################
# 3. TTS
########################################################
from medi_buddy_server.modules.tts import TTS
tts = TTS()


########################################################
# 4. TTS Publisher
########################################################
class TtsPublisher(Node):
    def __init__(self):
        super().__init__("tts_audio_publisher")
        self.pub = self.create_publisher(String, "tts_audio_wav", 10)

    def publish_wav(self, wav_path):
        try:
            with open(wav_path, "rb") as f:
                wav_bytes = f.read()

            b64 = base64.b64encode(wav_bytes).decode("utf-8")

            msg = String()
            msg.data = b64
            self.pub.publish(msg)

            self.get_logger().info(f"📤 TTS wav 파일 Base64 송신 완료: {wav_path}")

        except Exception as e:
            self.get_logger().error(f"❌ TTS publish 오류: {e}")


tts_pub_node = None



########################################################
# 5. OCR 결과 & Client Name 구독 
########################################################

latest_ocr_text = None

class OcrResultSubscriber(Node):
    def __init__(self):
        super().__init__("ocr_result_subscriber")
        self.subscription = self.create_subscription(
            String,
            "ocr_result",
            self.callback_ocr_result,
            10
        )
        self.get_logger().info("📥 OCR 결과 구독 시작 (/ocr_result)")

    def callback_ocr_result(self, msg):
        global latest_ocr_text
        latest_ocr_text = msg.data
        self.get_logger().info(f"📌 OCR 결과 수신: {latest_ocr_text}")



client_name = None

medical_records = {'정지아': ['발열', '기침', '코로나 의심'],
                   '채서린': ['고혈압', '안압 상승'],
                   '염한결': ['뇌혈관 질환'],
                   '황혜윤': ['빈맥', '저혈압'],
                   '박현욱': ['심한 스트레스', '불면증'],
                   '정지아': ['근육 긴장']}

class ClientNameSubscriber(Node):
    def __init__(self):
        super().__init__("client_name_subscriber")
        self.subscription = self.create_subscription(
            String,
            "client_name",
            self.callback_client_name,
            10
        )
        self.get_logger().info("📥 Client Name 구독 시작 (/client_name)")

    def callback_client_name(self, msg):
        global client_name
        client_name = msg.data
        self.get_logger().info(f"📌 Client Name 수신: {client_name}")



repeat = False
current_destination = None

class CurrentDestSubscriber(Node):
    def __init__(self):
        super().__init__("current_dest_subscriber")
        self.subscription = self.create_subscription(
            String,
            "current_destination",
            self.callback_current_dest,
            10
        )
        self.get_logger().info("📥 현재 목적지 구독 시작 (/current_destination)")

    def callback_current_dest(self, msg):
        global current_destination, repeat
        current_destination = msg.data
        message = ""

        room_map = {
                "X-ray실": "x_ray_room",
                "응급실": "emergency_room",
                "채혈실": "blood_draw_room",
                "약국": "pharmacy",
                "수납": "reception",
                "화장실": "restroom"
            }

        for kor, eng in room_map.items():
            if eng == current_destination:
                current_destination = kor
                message = f"{current_destination} 안내를 시작합니다"
                break
        if not repeat:
            wav_path = tts.make_tts(message)
            tts_pub_node.publish_wav(wav_path)
        else:
            repeat = False
        self.get_logger().info(f"📌 현재 목적지 수신: {current_destination}")



class ArrivalSubscriber(Node):
    def __init__(self):
        super().__init__("arrival_subscriber")
        self.subscription = self.create_subscription(
            Bool,
            "destination_arrival",
            self.callback_arrival,
            10
        )
        self.get_logger().info("📥 목적지 도착 여부 구독 시작 (/destination_arrival)")

    def callback_arrival(self, msg):
        global current_destination
        arrival = msg.data
        if arrival:
            message = f"{current_destination} 에 도착했습니다."
            wav_path = tts.make_tts(message)
            tts_pub_node.publish_wav(wav_path)

        self.get_logger().info(f"📌 목적지 도착 여부 수신: {arrival}")



########################################################
# 6. 트리 라우팅
########################################################
def tree(voice):
    global latest_ocr_text, repeat

    mode = predict(voice)['label']
    print("분류 결과:", mode)

    match mode:

        case 0:
            status_pub_node.publish_status("null")
            message = "제가 답변드릴 수 없을 것 같아요"

        case 1:
            status_pub_node.publish_status("detour")

            room_map = {
                "X-ray실": "x_ray_room",
                "응급실": "emergency_room",
                "채혈실": "blood_draw_room",
                "약국": "pharmacy",
                "수납": "reception",
                "화장실": "restroom"
            }

            message = "안내할 수 있는 장소를 찾지 못했습니다"

            for kor, eng in room_map.items():
                if kor in voice:
                    message = f"{kor} 안내를 시작합니다"
                    repeat = True
                    detour_pub_node.publish_destination(eng)
                    break

        case 2:
            status_pub_node.publish_status("ocr_start")

            ### OCR 노드 트리거
            ocr_pub_node.publish_request()

            ### TTS 안내
            guide_path = tts.make_tts("원하는 문서를 보여주세요")
            tts_pub_node.publish_wav(guide_path)
            print("추가 텍스트:", "원하는 문서를 보여주세요")

            ### OCR 텍스트가 들어올 때까지 기다림
            #    - 비동기 ROS 구조에서 polling 방식으로
            wait_t = 0
            while latest_ocr_text is None and wait_t < 300:
                time.sleep(0.2)
                wait_t += 0.2

            if latest_ocr_text is None:
                message = "문서를 인식하지 못했습니다."
            else:
                sys_message = "약 정보를 효능, 부작용, 주의 사항 중심으로 3문장의 쉬운 존댓말로 설명해주세요."
                message = llm(sys_message, query=latest_ocr_text)
                latest_ocr_text = None

        case 3:
            status_pub_node.publish_status("question_drug")
            sys_message = "약 정보를 효능, 부작용, 주의 사항 중심으로 3문장의 쉬운 존댓말로 설명해주세요."
            message = llm(sys_message, query=voice)

        case 4:
            status_pub_node.publish_status("question_disease")
            sys_message = "질병 정보를 3문장의 쉬운 말로 설명해주세요."
            message = llm(sys_message, query=voice)

        case _:
            message = "처리할 수 없는 요청입니다"

    # 최종 음성 안내
    wav_path = tts.make_tts(message)
    tts_pub_node.publish_wav(wav_path)

    return mode, message



########################################################
# 7. STT 노드
########################################################
class VoiceRouterNode(Node):
    def __init__(self):
        super().__init__("voice_router_node")

        self.subscription = self.create_subscription(
            String,
            "recorded_audio_mp3",
            self.callback_received_mp3,
            10
        )

        self.client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
        self.get_logger().info("🎧 Voice Router Node Started")

    def callback_received_mp3(self, msg):
        try:
            mp3_path = "/tmp/received.mp3"
            wav_path = "/tmp/received.wav"

            mp3_bytes = base64.b64decode(msg.data)
            with open(mp3_path, "wb") as f:
                f.write(mp3_bytes)

            self.get_logger().info(f"💾 MP3 저장완료: {mp3_path}")

            subprocess.run([
                "ffmpeg", "-y", "-i", mp3_path,
                "-ar", "16000",
                "-ac", "1",
                wav_path
            ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

            self.get_logger().info("🎵 WAV 변환 완료")

            with open(wav_path, "rb") as audio_file:
                start_t = time.time()
                voice = self.client.audio.transcriptions.create(
                    model='gpt-4o-mini-transcribe',
                    file=audio_file,
                    response_format='text',
                    language='ko'
                )
                duration = time.time() - start_t

            self.get_logger().info(f"📝 STT 결과: {voice}")
            self.get_logger().info(f"⏱ STT 처리 시간: {duration:.2f}s")

            mode, message = tree(voice)

        except Exception as e:
            self.get_logger().error(f"❌ 오류 발생: {e}")


########################################################
# 8. ROS2 Utility 노드들
########################################################
class OcrRequestPublisher(Node):
    def __init__(self):
        super().__init__("ocr_request_publisher")
        self.pub = self.create_publisher(Bool, "ocr_request", 10)

    def publish_request(self):
        msg = Bool()
        msg.data = True
        self.pub.publish(msg)
        self.get_logger().info("📤 OCR 요청 신호 발행")


class StatusPublisher(Node):
    def __init__(self):
        super().__init__("status_publisher")
        self.pub = self.create_publisher(String, "robot_status", 10)

    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.pub.publish(msg)
        self.get_logger().info(f"📤 Robot Status: {text}")


class LlmResultPublisher(Node):
    def __init__(self):
        super().__init__("llm_result_publisher")
        self.pub = self.create_publisher(String, "llm_result", 10)

    def publish_result(self, text):
        msg = String()
        msg.data = text
        self.pub.publish(msg)
        self.get_logger().info("📤 LLM Result Published")


class DetourPublisher(Node):
    def __init__(self):
        super().__init__("detour_publisher")
        self.pub = self.create_publisher(String, "detour", 10)

    def publish_destination(self, text):
        msg = String()
        msg.data = text
        self.pub.publish(msg)
        self.get_logger().info(f"📤 Detour Published: {text}")


########################################################
# 9. MAIN
########################################################
def main(args=None):
    global tts_pub_node, ocr_pub_node, status_pub_node, llm_pub_node, detour_pub_node

    rclpy.init(args=args)

    voice_node = VoiceRouterNode()
    tts_pub_node = TtsPublisher()
    ocr_pub_node = OcrRequestPublisher()
    status_pub_node = StatusPublisher()
    llm_pub_node = LlmResultPublisher()
    detour_pub_node = DetourPublisher()
    ocr_result_node = OcrResultSubscriber()
    client_name_node = ClientNameSubscriber()
    current_dest_node = CurrentDestSubscriber()
    arrival_node = ArrivalSubscriber()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(voice_node)
    executor.add_node(tts_pub_node)
    executor.add_node(ocr_pub_node)
    executor.add_node(status_pub_node)
    executor.add_node(llm_pub_node)
    executor.add_node(detour_pub_node)
    executor.add_node(ocr_result_node)
    executor.add_node(client_name_node)
    executor.add_node(current_dest_node)
    executor.add_node(arrival_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        voice_node.destroy_node()
        tts_pub_node.destroy_node()
        ocr_pub_node.destroy_node()
        status_pub_node.destroy_node()
        llm_pub_node.destroy_node()
        detour_pub_node.destroy_node()
        ocr_result_node.destroy_node()
        client_name_node.destroy_node()
        current_dest_node.destroy_node()
        arrival_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()