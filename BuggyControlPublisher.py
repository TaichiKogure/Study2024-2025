import rclpy  # ROS 2のPythonライブラリ
from rclpy.node import Node  # ノードクラス
from geometry_msgs.msg import Twist  # バギー制御で使うメッセージ型（速度コマンド）


class BuggyPublisher(Node):
    def __init__(self):
        super().__init__('buggy_publisher')  # ノード名を設定
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)  # トピック名: "cmd_vel"
        timer_period = 0.5  # 送信間隔（0.5秒ごと）
        self.timer = self.create_timer(timer_period, self.timer_callback)  # タイマーで送信

    def timer_callback(self):
        # Twistメッセージを作成
        msg = Twist()
        msg.linear.x = 0.5  # 前進速度（m/s）
        msg.linear.y = 0.0  # 横移動速度（m/s, 通常は0）
        msg.linear.z = 0.0  # 上下移動（m/s, 通常は0）
        msg.angular.x = 0.0  # 回転速度（ラジアン/s, 通常は0）
        msg.angular.y = 0.0  # 通常は不要
        msg.angular.z = 0.2  # 回転速度（左回りが正、右回りが負）

        # トピックに送信
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing linear.x={msg.linear.x}, angular.z={msg.angular.z}")


def main(args=None):
    rclpy.init(args=args)  # ROS 2環境の初期化
    buggy_publisher = BuggyPublisher()  # バギーの制御ノードを作成
    rclpy.spin(buggy_publisher)  # ノードを実行
    buggy_publisher.destroy_node()  # ノードの終了
    rclpy.shutdown()  # ROS 2の終了


if __name__ == '__main__':
    main()