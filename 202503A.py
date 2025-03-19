# Test comment
# Placeholder for ROS example program: Create a simple ROS node and publish messages.
# Main function to initiate a ROS node (example placeholder).
# Example: rospy.init_node('example_node')
# More implementation would go here for publishers/subscribers.

# Import libraries for decision tree analysis
import pandas as pd
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.tree import DecisionTreeClassifier
from sklearn.metrics import accuracy_score
from sklearn.datasets import fetch_openml

# Load Pima Indians Diabetes Dataset from UCI Machine Learning Repository (via OpenML)
df = fetch_openml(name='diabetes', version=1, as_frame=True).frame

# Split the dataset into features and target
X = df.drop(columns=['class'])
y = df['class']

# Split into training and testing sets
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.3, random_state=42)

# Create and train the decision tree model
model = DecisionTreeClassifier(random_state=42)
model.fit(X_train, y_train)

# Make predictions and evaluate the model
y_pred = model.predict(X_test)
accuracy = accuracy_score(y_test, y_pred)

# Print the results
print("Decision Tree Accuracy: {:.2f}%".format(accuracy * 100))

#%%

# ROS 2 imports (unchanged)
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

### バギーの動作に必要な構成例
# - **Publisher**: 「速度コマンド」を送るノード。
#     - たとえば、車輪を動かすための速度データ（リニア速度と回転速度）をトピックに配信します。
#
# - **Subscriber**（またはハードウェアのドライバ）: 配信された速度コマンドを実際のモーターに送り、バギーを動かします。
#
# 以下はシンプルな例で、バギーに一定の速度指令を送るPublisherノードをPythonで書いてみます。
## ROS 2でのバギー制御用コード例
# このコードでは、`geometry_msgs/Twist` というメッセージ型を用いて、バギーに速度指令を送ります。

#%%

