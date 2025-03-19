# Test comment
# Placeholder for ROS example program: Create a simple ROS node and publish messages.
# Main function to initiate a ROS node (example placeholder).
# Example: rospy.init_node('example_node')
# More implementation would go here for publishers/subscribers.

import rclpy  # ROS 2のPythonライブラリ
from rclpy.node import Node  # ノードクラス
from std_msgs.msg import String  # トピックで使うメッセージ型


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')  # ノード名を指定
        self.publisher_ = self.create_publisher(String, 'topic', 10)  # Publisherを作成
        timer_period = 1.0  # データの送信間隔を指定（1秒ごと）
        self.timer = self.create_timer(timer_period, self.timer_callback)  # タイマーで送信
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'  # メッセージ内容
        self.publisher_.publish(msg)  # メッセージをトピックに配信
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)  # ROS 2通信を初期化
    minimal_publisher = MinimalPublisher()  # ノードのインスタンスを作成
    rclpy.spin(minimal_publisher)  # ノードを実行
    minimal_publisher.destroy_node()  # ノードを終了
    rclpy.shutdown()  # ROS 2通信を終了


if __name__ == '__main__':
    main()


# 1. Node名: `minimal_publisher`
# 2. "Hello World"という文字列を1秒ごとに配信。
# 3. 配信先トピック名: `topic`
# 4. メッセージ型: `std_msgs.msg.String`

