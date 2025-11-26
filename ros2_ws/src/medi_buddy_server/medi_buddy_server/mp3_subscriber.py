#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import base64
import time
import os

class MP3SaverNode(Node):
    def __init__(self):
        super().__init__('mp3_saver_node')

        # 📥 Base64 MP3 구독
        self.subscription = self.create_subscription(
            String,
            'recorded_audio_mp3',
            self.callback_received_mp3,
            10
        )

        self.get_logger().info("📥 MP3 Saver Node Started (Base64 → MP3 파일 저장 전용)")

    def callback_received_mp3(self, msg):
        try:
            timestamp = int(time.time())
            file_path = f"/tmp/received_{timestamp}.mp3"

            # Base64 → bytes
            mp3_bytes = base64.b64decode(msg.data)

            # 파일 저장
            with open(file_path, "wb") as f:
                f.write(mp3_bytes)

            self.get_logger().info(f"💾 MP3 파일 저장 완료: {file_path}")

        except Exception as e:
            self.get_logger().error(f"❌ 디코딩/저장 중 오류 발생: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MP3SaverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

