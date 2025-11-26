#!/usr/bin/env python3
import os
os.environ["QT_QPA_PLATFORM"] = "xcb"

import json
import cv2
import numpy as np
from paddleocr import PaddleOCR
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node

# 설정값
CONFIDENCE_THRESHOLD = 0.3
DETECT_BRAND_NAME = True
FONT_PATH = "/usr/share/fonts/truetype/nanum/NanumGothic.ttf"  # 필요시 사용 (현재 랜더는 없음)

# ---------------- utility (기존 로직에서 OCR 가공 부분만 그대로 사용) ----------------

def extract_ocr_data_from_result(result):
    """PaddleOCR 결과에서 bbox, text, score 리스트로 변환"""
    ocr_data = []
    if not result:
        return ocr_data

    # result 형식은 사용한 paddleocr 버전에 따라 다를 수 있음.
    # 사용자 제공 코드에 맞춰 .get('dt_polys') 스타일을 먼저 시도하고, 없으면 기본 포맷(ocr 결과)을 처리.
    for res in result:
        # res이 dict 스타일일 경우
        if isinstance(res, dict):
            dt_polys = res.get('dt_polys')
            rec_texts = res.get('rec_texts')
            rec_scores = res.get('rec_scores')
        else:
            # PaddleOCR의 ocr(...) 반환 포맷일 경우: list of [bbox, (text, score)]
            # 예: [ [[x1,y1],[x2,y2],...], ('텍스트', 0.98) ]
            dt_polys = None
            rec_texts = []
            rec_scores = []
            try:
                # res이 list 형태로 여러 검출을 담고 있는 경우
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

        # numpy -> py list 처리
        if hasattr(dt_polys, 'tolist'):
            dt_polys = dt_polys.tolist()
        if hasattr(rec_scores, 'tolist'):
            rec_scores = rec_scores.tolist()

        for bbox, text, score in zip(dt_polys, rec_texts, rec_scores):
            if score >= CONFIDENCE_THRESHOLD:
                ocr_data.append({'bbox': bbox, 'text': text, 'score': float(score)})

    return ocr_data


def find_brand_name(ocr_data, image_shape):
    """가장 큰(높이 기반) 텍스트를 약품명 후보로 반환"""
    if not ocr_data:
        return None

    img_h, img_w = image_shape[:2]
    max_score = -1
    brand_info = None

    for item in ocr_data:
        bbox = np.array(item['bbox'])
        text = item['text']

        # bbox 높이
        box_height = np.max(bbox[:, 1]) - np.min(bbox[:, 1])

        # 1글자 노이즈 제거 (단, 이미지 높이의 20% 이상이면 예외)
        if len(text) < 2 and box_height < (img_h * 0.2):
            continue
        # 너무 긴 문장 제거
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

# ------------------------------- ROS2 Node -------------------------------

class OcrNode(Node):
    def __init__(self):
        super().__init__("ocr_node")

        # 상태
        self.ocr_request = False  # /ocr_request가 True일 때만 처리 (처리 후 자동 리셋)
        self.bridge = CvBridge()

        # publishers / subscribers
        self.create_subscription(Bool, "/ocr_request", self.ocr_request_callback, 10)
        self.create_subscription(Image, "/camera/image_raw", self.image_callback, 5)
        self.ocr_result_pub = self.create_publisher(String, "/ocr_result", 10)

        # PaddleOCR 초기화 (무거우니 한 번만)
        self.get_logger().info("⚙️ PaddleOCR 초기화 중 (cpu)...")
        try:
            self.ocr = PaddleOCR(use_angle_cls=False, lang="korean", device="cpu")
            # 사용환경에 따라: PaddleOCR(..., use_angle_cls=True) 등 옵션 조절 가능
        except Exception as e:
            self.get_logger().error(f"❌ PaddleOCR 초기화 실패: {e}")
            raise

        # 내부 프레임 카운터 (과부하 방지용)
        self._frame_idx = 0
        self.get_logger().info("📸 OCR Node Ready - waiting for /ocr_request (Bool)")

    # /ocr_request 토픽 핸들러
    def ocr_request_callback(self, msg: Bool):
        self.ocr_request = bool(msg.data)
        if self.ocr_request:
            self.get_logger().info("▶ /ocr_request = True: 다음 수신 프레임에서 OCR 수행")
        else:
            self.get_logger().info("■ /ocr_request = False: OCR 비활성화")

    # 이미지 콜백: ocr_request이 True일 때 한 프레임만 처리하고 리셋
    def image_callback(self, msg: Image):
        # 요청이 없으면 바로 리턴
        if not self.ocr_request:
            return

        try:
            # ROS Image -> OpenCV BGR
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # OCR 수행 (이미지 크기/프레임 건너뛰기 정책은 필요시 조절)
            self.get_logger().info("🔍 OCR 수행 시작")
            # PaddleOCR의 반환 포맷은 버전에 따라 다름. 사용하던 predict 방식도 가능하면 그걸 쓰도록 시도.
            try:
                result = self.ocr.predict(input=cv_image)
            except Exception:
                # fallback to ocr.ocr(...) 포맷
                result = self.ocr.ocr(cv_image, cls=False)

            ocr_data = extract_ocr_data_from_result(result)
            brand_info = find_brand_name(ocr_data, cv_image.shape) if DETECT_BRAND_NAME else None

            # 결과 JSON 구성
            items = []
            for it in ocr_data:
                # bbox를 직렬화 가능한 형식으로 변환 (list of [x,y])
                bbox = [[float(p[0]), float(p[1])] for p in it['bbox']]
                items.append({
                    "text": it['text'],
                    "score": float(it['score']),
                    "bbox": bbox
                })

            output = {
                "brand_name": brand_info['text'] if brand_info else None,
                "brand_score": float(brand_info['score']) if (brand_info and 'score' in brand_info) else None,
                "brand_ranking_score": float(brand_info['ranking_score']) if (brand_info and 'ranking_score' in brand_info) else None,
                "items": items
            }

            # publish JSON string
            msg_out = String()
            print(msg_out)
            msg_out.data = json.dumps(output, ensure_ascii=False)
            self.ocr_result_pub.publish(msg_out)
            self.get_logger().info(f"✅ OCR 결과 발행 (/ocr_result). items={len(items)}, brand={output['brand_name']}")

        except Exception as e:
            self.get_logger().error(f"❌ OCR 처리 중 예외 발생: {e}")

        finally:
            # 한 번 처리했으면 자동 리셋: 다음 /ocr_request가 True가 될 때까지 처리 안 함
            self.ocr_request = False
            self.get_logger().info("⏸ OCR request 자동 리셋 (ocr_request=False)")

# ------------------------------- main -------------------------------

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
