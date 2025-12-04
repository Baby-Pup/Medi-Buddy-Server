#!/usr/bin/env python3
import os
os.environ["QT_QPA_PLATFORM"] = "xcb"

import cv2
import numpy as np
from paddleocr import PaddleOCR
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
import time

# 설정값
CONFIDENCE_THRESHOLD = 0.3
FONT_PATH = "/usr/share/fonts/truetype/nanum/NanumGothic.ttf"


# ---------------- OCR utility ----------------

def extract_ocr_data_from_result(result):
    ocr_data = []
    if not result:
        return ocr_data

    for res in result:
        if isinstance(res, dict):
            dt_polys = res.get('dt_polys')
            rec_texts = res.get('rec_texts')
            rec_scores = res.get('rec_scores')
        else:
            dt_polys = None
            rec_texts = []
            rec_scores = []
            try:
                for entry in res:
                    if isinstance(entry, list) and len(entry) == 2:
                        bbox = entry[0]
                        text, score = entry[1]
                        if score >= CONFIDENCE_THRESHOLD:
                            ocr_data.append({'bbox': bbox, 'text': text, 'score': float(score)})
                continue
            except Exception:
                pass

        if dt_polys is None or rec_texts is None:
            continue

        if hasattr(dt_polys, 'tolist'):
            dt_polys = dt_polys.tolist()
        if hasattr(rec_scores, 'tolist'):
            rec_scores = rec_scores.tolist()

        for bbox, text, score in zip(dt_polys, rec_texts, rec_scores):
            if score >= CONFIDENCE_THRESHOLD:
                ocr_data.append({'bbox': bbox, 'text': text, 'score': float(score)})

    return ocr_data


def find_brand_name(ocr_data, image_shape):
    if not ocr_data:
        return None

    img_h, img_w = image_shape[:2]
    max_score = -1
    brand_info = None

    for item in ocr_data:
        bbox = np.array(item['bbox'])
        text = item['text']

        box_height = np.max(bbox[:, 1]) - np.min(bbox[:, 1])

        if len(text) < 2 and box_height < (img_h * 0.08):
            continue
        if len(text) > 50:
            continue

        score = box_height ** 2

        if "제품명" in text:
            score *= 1.2

        if score > max_score:
            max_score = score
            brand_info = item.copy()
            brand_info.update({'ranking_score': float(score)})

    return brand_info


# ---------------- ROS2 Node ----------------

class OcrNode(Node):
    def __init__(self):
        super().__init__("ocr_node")

        self.ocr_request = False
        self.bridge = CvBridge()

        # 요청사항 반영
        self.skip_frames = 2
        self.frame_counter = 0
        self.brand_votes = {}
        self.required_votes = 3

        # ⏱️ 추가: OCR 타임아웃
        self.ocr_start_time = None
        self.ocr_timeout = 300.0   # 30초

        # ROS 설정
        self.create_subscription(Bool, "/ocr_request", self.ocr_request_callback, 10)
        self.image_sub = None

        self.ocr_result_pub = self.create_publisher(String, "/ocr_result", 10)
        self.status_pub = self.create_publisher(String, '/robot_status', 10)

        # OCR 초기화
        self.get_logger().info("⚙️ OCR 초기화 중...")
        self.ocr = PaddleOCR(use_angle_cls=False, lang="korean", device="cpu")

        self.get_logger().info("📸 OCR Node Ready - waiting for /ocr_request")

    # ---------------- ocr_request ----------------
    def ocr_request_callback(self, msg: Bool):
        requested = bool(msg.data)

        if requested and not self.ocr_request:
            self.get_logger().info("▶ OCR 요청 ON → 카메라 구독 시작")

            # 초기화
            self.frame_counter = 0
            self.brand_votes = {}
            self.ocr_start_time = time.time()   # 🔥 타임아웃 시작

            # 카메라 구독 시작
            self.image_sub = self.create_subscription(
                Image,
                "/camera/image_raw",
                self.image_callback,
                10
            )

        self.ocr_request = requested

    # ---------------- image callback ----------------
    def image_callback(self, msg: Image):
        if not self.ocr_request:
            return

        # 🔥 20초 타임아웃 체크
        if time.time() - self.ocr_start_time > self.ocr_timeout:
            self.get_logger().info("⛔ OCR 시간 초과(30초) → OCR 실패 처리")

            # 구독 종료
            if self.image_sub:
                self.destroy_subscription(self.image_sub)
                self.image_sub = None

            self.ocr_request = False
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.frame_counter += 1

        # 1) 초기 프레임 스킵
        if self.frame_counter <= self.skip_frames:
            self.get_logger().info(f"⏭ 초기 프레임 스킵 {self.frame_counter}/{self.skip_frames}")
            return

        # 3) OCR 수행
        self.get_logger().info("🔍 OCR 수행 중...")
        try:
            result = self.ocr.predict(input=cv_image)
        except Exception:
            result = self.ocr.ocr(cv_image, cls=False)

        ocr_data = extract_ocr_data_from_result(result)
        brand_info = find_brand_name(ocr_data, cv_image.shape)
        brand = brand_info['text'] if brand_info else ""

        if not brand:
            self.get_logger().info("❗ 텍스트 없음 → 다음 프레임")
            return

        self.get_logger().info(f"📌 검출 브랜드: {brand}")

        # 4) 브랜드 투표 집계
        self.brand_votes[brand] = self.brand_votes.get(brand, 0) + 1

        # 5) 기준 이상 반복 검출 → 확정
        if self.brand_votes[brand] >= self.required_votes:
            self.get_logger().info(f"🏆 브랜드 확정: {brand}")

            msg_out = String()
            msg_out.data = brand
            self.ocr_result_pub.publish(msg_out)

            status_msg = String()
            status_msg.data = "ocr_complete"
            self.status_pub.publish(status_msg)

            # 구독 종료
            if self.image_sub:
                self.destroy_subscription(self.image_sub)
                self.image_sub = None

            self.ocr_request = False
            return


# ---------------- main ----------------

def main(args=None):
    rclpy.init(args=args)
    node = OcrNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()