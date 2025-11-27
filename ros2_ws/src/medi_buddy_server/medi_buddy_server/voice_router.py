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
            {
                "role": "system",
                "content": "당신은 의료 지원 로봇 Medi-Buddy입니다." + sys_message
            },
            {"role": "user", "content": query}
        ]
    ).choices[0].message.content

    llm_pub_node.publish_result(re)
    return re


########################################################
# 3. TTS (기존 시스템 사용)
########################################################
from medi_buddy_server.modules.tts import TTS
tts = TTS()


########################################################
# 4-A. TTS Publisher 노드 (Base64 WAV 발행)
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


# 전역 변수로 선언 (tree()에서 사용)
tts_pub_node = None


########################################################
# 4-B. 트리 라우팅 (make_and_play 제거)
########################################################
def tree(voice):
    mode = predict(voice)['label']
    print("분류 결과:", mode)

    match mode:
        case 0:
            status_pub_node.publish_status("null")
            message = "제가 답변드릴 수 없을 것 같아요"

        case 1:
            status_pub_node.publish_status("detour")
            rooms = ['X-ray실', '물리치료실', '채혈실', '척추센터', '수납', '화장실']
            message = "안내할 수 있는 장소를 찾지 못했습니다"
            for r in rooms:
                if r in voice:
                    message = f"{r} 안내를 시작합니다"
                    break

        case 2:
            status_pub_node.publish_status("ocr_start")
            sys_message = "사용자의 문서를 3문장으로 요약하고 쉬운 말로 설명해주세요."

            ocr_pub_node.publish_request()

            # TTS 파일 생성 후 publish
            guide_path = tts.make_tts("원하는 문서를 보여주세요")
            tts_pub_node.publish_wav(guide_path)
            print("추가 텍스트:", "원하는 문서를 보여주세요")

            OCR_result = """
            1. 본 시술은 다양한 급성 및 만성 통증을 완화하고 치료하기 위해 시행됩니다.
            2. 경막외 카테터 삽입술을 시행하지 않을 경우 효과적인 통증 관리가 어려울 수 있습니다.
            3. 시술 과정은 다음과 같습니다...
            """

            message = llm(sys_message, query=OCR_result)

        case 3:
            status_pub_node.publish_status("question_drug")
            sys_message = "약 정보를 효능, 부작용, 주의 사항 중심으로 3문장의 쉬운 말로 설명해주세요."
            message = llm(sys_message, query=voice)

        case 4:
            status_pub_node.publish_status("question_disease")
            sys_message = "질병 정보를 3문장의 쉬운 말로 설명해주세요."
            message = llm(sys_message, query=voice)

        case _:
            message = "처리할 수 없는 요청입니다"

    print("🧠 LLM 결과 텍스트:", message)

    wav_path = tts.make_tts(message)
    tts_pub_node.publish_wav(wav_path)

    return mode, message


########################################################
# 5. Whisper 모델 (STT)
########################################################
from medi_buddy_server.modules.stt import RMS_VAD
stt = RMS_VAD()


########################################################
# 6. ROS2: MP3 → STT → Intent
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

        self.get_logger().info("🎧 Voice Router Node Started (MP3 → STT → Intent → LLM → TTS)")


    def callback_received_mp3(self, msg):
        try:
            mp3_path = f"/tmp/received.mp3"
            wav_path = f"/tmp/received.wav"

            # Base64 → MP3 저장
            mp3_bytes = base64.b64decode(msg.data)
            with open(mp3_path, "wb") as f:
                f.write(mp3_bytes)

            self.get_logger().info(f"💾 MP3 저장완료: {mp3_path}")

            # MP3 → WAV 변환
            subprocess.run([
                "ffmpeg", "-y", "-i", mp3_path,
                "-ar", "16000",
                "-ac", "1",
                wav_path
            ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

            # WAV → numpy
            audio_np, sr = sf.read(wav_path, dtype="float32")

            # Whisper STT
            text, infer_time = stt.transcribe(audio_np)
            self.get_logger().info(f"📝 STT 결과: {text}")

            # Intent + LLM + TTS
            mode, message = tree(text)
            self.get_logger().info(f"분류 결과: {mode}")
            self.get_logger().info(f"📝 TTS 결과: {message}")

        except Exception as e:
            self.get_logger().error(f"❌ 오류 발생: {e}")


########################################################
# 7. OCR노드  실행
########################################################
class OcrRequestPublisher(Node):
    def __init__(self):
        super().__init__("ocr_request_publisher")
        self.pub = self.create_publisher(Bool, "ocr_request", 10)

    def publish_request(self):
        msg = Bool()
        msg.data = True
        self.pub.publish(msg)
        self.get_logger().info("📤 OCR 요청 신호 발행 (ocr_request=True)")

########################################################
# 8. StatusPublisher노드 실행
########################################################
class StatusPublisher(Node):
    def __init__(self):
        super().__init__("status_publisher")
        self.pub = self.create_publisher(String, "robot_status", 10)

    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.pub.publish(msg)
        self.get_logger().info(f"📤 Robot Status Published: {text}")

########################################################
# 9. LlmResultPublisher노드 실행
########################################################
class LlmResultPublisher(Node):
    def __init__(self):
        super().__init__("llm_result_publisher")
        self.pub = self.create_publisher(String, "llm_result", 10)

    def publish_result(self, text):
        msg = String()
        msg.data = text
        self.pub.publish(msg)
        self.get_logger().info(f"📤 LLM Result Published: {text[:50]}...")

########################################################
# 10. LlmResultPublisher노드 실행
########################################################
class DetourPublisher(Node):
    def __init__(self):
        super().__init__("detour_publisher")
        self.pub = self.create_publisher(String, "detour", 10)

    def publish_result(self, text):
        msg = String()
        msg.data = text
        self.pub.publish(msg)
        self.get_logger().info(f"📤 Detour Published: {text}...")

########################################################
# 11. ROS2 실행
########################################################
def main(args=None):
    global tts_pub_node, ocr_pub_node, status_pub_node, llm_pub_node, detour_pub_node

    rclpy.init(args=args)

    voice_node = VoiceRouterNode()
    tts_pub_node = TtsPublisher()
    ocr_pub_node = OcrRequestPublisher()
    status_pub_node = StatusPublisher()
    llm_pub_node = LlmResultPublisher()
    detour_pub_node = LlmResultPublisher()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(voice_node)
    executor.add_node(tts_pub_node)
    executor.add_node(ocr_pub_node)
    executor.add_node(status_pub_node)
    executor.add_node(llm_pub_node)
    executor.add_node(detour_pub_node)

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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
