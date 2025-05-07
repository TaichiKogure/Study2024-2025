import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# MinimalPublisher クラスを Node から継承して定義します。
class MinimalSubscriber(Node):
    # クラスの初期化メソッド
    def __init__(self):
        # Node クラスの初期化メソッドを呼び出します。
        super().__init__('minimal_subscriber')

        # String型のメッセージを "topic" というトピックでサブスクライブします。
        # コールバック関数を listener_callback に設定し、キューサイズを 10 に設定します。
        self.subscription = self.create_subscription(String, 'topic', self.listener_callback, 10)

    # メッセージコールバック関数を定義します。
    def listener_callback(self, msg):
        # ロガーを取得し、受信したメッセージを印刷します。
        self.get_logger().info('I heard: "%s"' % msg.data)

# メイン関数を定義します。
def main(args=None):
    # ROS2ノードを初期化します。
    rclpy.init(args=args)

    # MinimalSubscriberオブジェクトを作成します。
    minimal_subscriber = MinimalSubscriber()

    # ROS2ノードのイベントループに入ります。
    rclpy.spin(minimal_subscriber)

    # ノードオブジェクトを破棄します。
    minimal_subscriber.destroy_node()

    # ROS2ノードをシャットダウンします。
    rclpy.shutdown()

# このスクリプトがメインプログラムの場合、main関数を実行します。
if __name__ == '__main__':
    main()