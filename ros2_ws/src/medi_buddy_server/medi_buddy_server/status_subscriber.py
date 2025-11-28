import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import threading
import time
import tempfile
import os

FILE_PATH = "/tmp/robot_ui_status.json"

def atomic_write_json(path, data):
    """JSON 파일을 atomic하게 쓰기 (깨질 위험 0%)"""
    dir_path = os.path.dirname(path)
    with tempfile.NamedTemporaryFile("w", delete=False, dir=dir_path) as tmp:
        json.dump(data, tmp, ensure_ascii=False, indent=2)
        tmp_name = tmp.name
    os.replace(tmp_name, path)

class StatusSubscriber(Node):
    def __init__(self):
        super().__init__('status_subscriber')

        # 현재 상태 값들
        self.current_status = "idle"
        self.destinations = ""
        self.current_client_name = ""
        self.current_ocr_brand_name = ""
        self.current_llm_result = ""
        self.detour = ""
        self.current_date = ""
        self.current_destination = "s"

        # ---- 토픽 구독 ----
        self.create_subscription(String, '/robot_status', self.status_callback, 10)
        self.create_subscription(String, '/destination_list', self.destination_callback, 10)
        self.create_subscription(String, '/client_name', self.client_name_callback, 10)
        self.create_subscription(String, '/ocr_result', self.ocr_result_callback, 10)
        self.create_subscription(String, '/llm_result', self.llm_result_callback, 10)
        self.create_subscription(String, '/detour', self.detour_callback, 10)
        self.create_subscription(String, '/date', self.date_callback, 10)
        self.create_subscription(String, '/current_destination', self.current_destination_callback, 10)

        self.write_json()
        self.get_logger().info("Status listener ready.")

    # -------------------------------
    # JSON atomic write
    # -------------------------------
    def write_json(self):
        data = {
            "status": self.current_status,
            "destinations": self.destinations,
            "client_name": self.current_client_name,
            "ocr_brand_name": self.current_ocr_brand_name,
            "llm_result": self.current_llm_result,
            "detour": self.detour,
            "date": self.current_date,
            "current_destination": self.current_destination,
        }
        atomic_write_json(FILE_PATH, data)

    # -------------------------------
    # robot_status 콜백
    # -------------------------------
    def status_callback(self, msg):
        self.current_status = msg.data
        self.write_json()
        self.get_logger().info(f"[WRITE] status: {self.current_status}")

        # idle reset
        threading.Thread(target=self.reset_status, daemon=True).start()

    def destination_callback(self, msg):
        self.destinations = msg.data
        self.write_json()
        self.get_logger().info(f"[WRITE] destinations: {self.destinations}")

    def client_name_callback(self, msg):
        self.current_client_name = msg.data
        self.write_json()
        self.get_logger().info(f"[WRITE] client_name: {self.current_client_name}")

    def ocr_result_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.current_ocr_brand_name = data.get("brand_name", "")
            self.write_json()
            self.get_logger().info(f"[WRITE] ocr_brand_name: {self.current_ocr_brand_name}")
        except Exception as e:
            self.get_logger().error(f"OCR result JSON parsing error: {e}")

    def llm_result_callback(self, msg):
        self.current_llm_result = msg.data
        self.write_json()
        self.get_logger().info(f"[WRITE] llm_result: {self.current_llm_result}")

    def detour_callback(self, msg):
        self.detour = msg.data
        self.write_json()
        self.get_logger().info(f"[WRITE] detour: {self.detour}")

    def date_callback(self, msg):
        self.current_date = msg.data
        self.write_json()
        self.get_logger().info(f"[WRITE] date: {self.current_date}")

    def current_destination_callback(self, msg):
        self.current_destination = msg.data
        self.write_json()
        self.get_logger().info(f"[WRITE] current destination: {self.current_destination}")

    # -------------------------------
    # idle reset
    # -------------------------------
    def reset_status(self):
        time.sleep(0.2)
        self.current_status = "idle"
        self.current_ocr_brand_name = ""
        self.current_llm_result = ""
        self.detour = ""
        self.write_json()
        self.get_logger().info("[RESET]")

def main():
    rclpy.init()
    node = StatusSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
