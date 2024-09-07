import time
from picamera2 import Picamera2
import libcamera
import cv2
import numpy as np
from PIL import Image  # PILのImageモジュールをインポート

# Picamera2の初期化
picamera = Picamera2()

# カメラの設定
config = picamera.create_preview_configuration(main={"format": 'YUV420', "size": (640, 480)},
                                               raw={"format": "SRGGB12", "size": (1920, 1080)})

# Transformの設定を修正。hflipとvflipをブール値にする。
config["transform"] = libcamera.Transform(hflip=False, vflip=True)

# カメラのコンフィグを設定
picamera.configure(config)

# カメラを開始
picamera.start()

# 少し待機してカメラの準備を完了
time.sleep(2)

# イメージをキャプチャ
image_array = picamera.capture_array()

# YUV420からRGBに変換
yuv_image = image_array.reshape((int(480 * 1.5), 640))
rgb_image = cv2.cvtColor(yuv_image, cv2.COLOR_YUV2RGB_I420)

# イメージを保存
image = Image.fromarray(rgb_image)
image.save('image.jpg')

# カメラを停止
picamera.stop()
