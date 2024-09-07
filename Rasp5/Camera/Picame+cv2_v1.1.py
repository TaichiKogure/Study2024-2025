import cv2
import threading
import ctypes
import inspect
import traitlets
import ipywidgets.widgets as widgets
from IPython.display import display
import numpy as np
import time
import libcamera
from picamera2 import Picamera2


def bgr8_to_jpeg(value, quality=75):
    return bytes(cv2.imencode('.jpg', value)[1])


makerobo_image = widgets.Image(format='jpeg', width=640, height=480)
display(makerobo_image)


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
        frame = picamera.capture_array()
        makerobo_image.value = bgr8_to_jpeg(frame)
        time.sleep(0.01)


t = threading.Thread(target=Video_display)
t.daemon = True  # ここを修正
t.start()

stop_thread(t)  # 実行中のスレッドを強制終了しますが、通常は不要です
