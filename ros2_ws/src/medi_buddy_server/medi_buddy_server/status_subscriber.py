# ros_listener.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import threading
import time

FILE_PATH = "/tmp/robot_ui_status.json"

class StatusSubscriber(Node):
    def __init__(self):
        super().__init__('status_subscriber')

        # 현재 메모리 값 (JSON에 매번 덮어쓰지 않도록 캐시)
        self.current_status = "idle"
        self.current_destinations = ""
        self.current_client_name = ""

        # ---- 토픽 구독 ----
        self.create_subscription(String, '/robot_status', self.status_callback, 10)
        self.create_subscription(String, '/destination_list', self.destination_callback, 10)
        self.create_subscription(String, '/client_name', self.client_name_callback, 10)

        # 초기 JSON 생성
        self.write_json()
        self.get_logger().info("Status listener ready.")

    # -------------------------------
    # robot_status 콜백
    # -------------------------------
    def status_callback(self, msg):
        self.current_status = msg.data

        # 상태 업데이트 기록
        self.write_json()
        self.get_logger().info(f"[WRITE] status: {self.current_status}")

        # 상태 이벤트 후 자동 idle 리셋
        threading.Thread(target=self.reset_status, daemon=True).start()

    # -------------------------------
    # 목적지 리스트 콜백
    # -------------------------------
    def destination_callback(self, msg):
        self.current_destinations = msg.data
        self.write_json()
        self.get_logger().info(f"[WRITE] destinations: {self.current_destinations}")

    # -------------------------------
    # 사용자 이름 콜백
    # -------------------------------
    def client_name_callback(self, msg):
        self.current_client_name = msg.data
        self.write_json()
        self.get_logger().info(f"[WRITE] client_name: {self.current_client_name}")

    # -------------------------------
    # JSON 파일 쓰기
    # -------------------------------
    def write_json(self):
        data = {
            "status": self.current_status,
            "destinations": self.current_destinations,
            "client_name": self.current_client_name
        }
        with open(FILE_PATH, "w") as f:
            json.dump(data, f)

    # -------------------------------
    # status -> idle 자동 리셋
    # -------------------------------
    def reset_status(self):
        time.sleep(0.25)  # Streamlit과 충돌 방지
        self.current_status = "idle"
        self.write_json()
        self.get_logger().info("[RESET]")


def main():
    rclpy.init()
    node = StatusSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
