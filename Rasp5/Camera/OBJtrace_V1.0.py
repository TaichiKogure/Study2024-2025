from collections import deque
from datetime import time

import numpy as np
import argparse
import imutils
import cv2
import threading
import ctypes
import inspect

def _async_raise(tid, exctype):
    tid =  ctypes.c_long(tid)
    if not inspect.isclass(exctype):
        exctype = type(exctype)
    res = ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, ctypes.py_object(exctype))
    if res == 0:
        raise ValueError("invalid thread id")
    elif res != 1:
        ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, None)
        raise SystemError("PyThreadState_SetAsyncExc failed")

def stop_thread(thread):
    _async_raise(thread.ident, SystemExit)

ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64, help="max buffer size")
args = vars(ap.parse_args(args=[]))

colorLower = (24, 100, 100)
colorUpper = (44, 255, 255)
pts = deque(maxlen=args["buffer"])

import libcamera
from picamera2 import Picamera2
picamera = Picamera2()
config = picamera.create_preview_configuration(main={"size": (640, 480)},
                                               raw={"size": (1920, 1080), "format": "SRGGB12"})
config["transform"] = libcamera.Transform(hflip=True, vflip=True)
picamera.configure(config)
picamera.start()

def bgr8_to_jpeg(value, quality=75):
    return bytes(cv2.imencode('.jpg', value)[1])

import traitlets
import ipywidgets.widgets as widgets
from IPython.display import display

Frame = widgets.Image(format='jpeg', width=640, height=480)
display(Frame)

def video_display():
    while True:
        frame = picamera.capture_array()
        frame = imutils.resize(frame, width=600)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, colorLower, colorUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        #cnts = imutils.grab_contours(cnts)
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
            cv2.line(frame, pts[i - 1], pts[i], (0,0,255), thickness)
        Frame.value = bgr8_to_jpeg(frame)
    camera.release()
t = threading.Thread(target=video_display)
t.setDaemon(True)
t.start()

stop_thread(t)

