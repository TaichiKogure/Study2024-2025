import cv2
import threading
import ctypes
import inspect
import numpy as np
import time
import libcamera
from picamera2 import Picamera2


def bgr8_to_jpeg(value, quality=75):
    return bytes(cv2.imencode('.jpg', value)[1])


def _async_raise(tid, exetype):
    tid = ctypes.c_long(tid)
    if not inspect.isclass(exetype):
        exetype = type(exetype)
    res = ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, ctypes.py_object(exetype))
    if res == 0:
        raise ValueError("invalid thread id")
    elif res != 1:
        ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, None)
        raise SystemError("PyThreadState_SetAsyncExc failed")


def stop_thread(thread):
    _async_raise(thread.ident, SystemExit)


picamera = Picamera2()
config = picamera.create_preview_configuration(main={"format": 'RGB888', "size": (640, 480)},
                                               raw={"format": "SRGGB12", "size": (1920, 1080)})
config["transform"] = libcamera.Transform(hflip=False, vflip=True)
picamera.configure(config)
picamera.start()


def Video_display():
    while True:
        frame = picamera.capture_array()  # フレームを取得
        cv2.imshow('Frame', frame)  # ウィンドウに表示

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


t = threading.Thread(target=Video_display)
t.daemon = True
t.start()

try:
    while True:
        time.sleep(1)  # メインスレッドのキープアライブ
        if not t.is_alive():
            break
except KeyboardInterrupt:
    stop_thread(t)

picamera.stop()
cv2.destroyAllWindows()

# 実行中のスレッドを強制終了しますが、通常は不要です
# stop_thread(t)
