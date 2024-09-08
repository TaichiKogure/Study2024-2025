from collections import deque
import numpy as np
import argparse
import imutils
import cv2
import threading
from picamera2 import Picamera2, Preview

# 引数解析
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64, help="max buffer size")
args = vars(ap.parse_args())

# マスク用のカラーレンジ
colorLower = (24, 100, 100)
colorUpper = (44, 255, 255)

# バッファ用のデック
pts = deque(maxlen=args["buffer"])

# カメラのセットアップ
picamera = Picamera2()
picamera.preview_configuration = picamera.create_preview_configuration(
    main={"format": "YUV420", "size": (640, 480)}
)
picamera.configure("preview")
picamera.start()


def yuv420_to_bgr(yuv_frame, width, height):
    # YUVフレームをBGRフレームに変換
    y_size = width * height
    uv_height = height // 2
    uv_width = width // 2

    # YUV420データをY, U, Vに分割
    Y = yuv_frame[:y_size].reshape((height, width))
    U = yuv_frame[y_size:y_size + (uv_height * uv_width)].reshape((uv_height, uv_width))
    V = yuv_frame[y_size + (uv_height * uv_width):].reshape((uv_height, uv_width))

    # YUV420フォーマットからYUV444フォーマットにアップサンプリング
    U = cv2.resize(U, (width, height), interpolation=cv2.INTER_LINEAR)
    V = cv2.resize(V, (width, height), interpolation=cv2.INTER_LINEAR)

    # YUVチャンネルを合成
    YUV = cv2.merge([Y, U, V])

    # YUVからBGRに変換
    bgr_frame = cv2.cvtColor(YUV, cv2.COLOR_YUV2BGR)
    return bgr_frame


def video_display():
    try:
        while True:
            stream = picamera.capture_buffer("main")
            frame = yuv420_to_bgr(stream, 640, 480)

            # 画像を上下反転
            frame = cv2.flip(frame, 0)

            frame = imutils.resize(frame, width=600)
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, colorLower, colorUpper)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
            center = None
            if len(cnts) > 0:
                c = max(cnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                if radius > 10:
                    cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    cv2.circle(frame, center, 5, (0, 0, 255), -1)
            pts.appendleft(center)
            for i in range(1, len(pts)):
                if pts[i - 1] is None or pts[i] is None:
                    continue
                thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
                cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)
            cv2.imshow("Frame", frame)

            # プログラム終了条件
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except Exception as e:
        print(f"ビデオ表示中のエラー: {e}")
    finally:
        picamera.stop()
        picamera.close()
        cv2.destroyAllWindows()


# ディスプレイスレッドの開始
t = threading.Thread(target=video_display)
t.start()

# 必要に応じてスレッドを安全に停止
# t.join()  # スレッドの終了を待つ場合
