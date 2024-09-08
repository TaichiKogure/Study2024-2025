import cv2
import numpy as np
from adafruit_servokit import ServoKit
import time
import threading
import ctypes
import inspect

kit = ServoKit(channels=16)
pan = 40
tilt = 50

# Initial position
kit.servo[0].angle = pan
kit.servo[1].angle = tilt


def bgr8_to_jpeg(value, quality=75):
    return bytes(cv2.imencode('.jpg', value)[1])


import traitlets
import ipywidgets.widgets as widgets
from IPython.display import display

FGmaskComp_img = widgets.Image(format='jpeg', width=320, height=240)
frame_img = widgets.Image(format='jpeg', width=320, height=240)

display_img = widgets.HBox([FGmaskComp_img, frame_img])
display(display_img)


def _async_raise(tid, exctype):
    tid = ctypes.c_long(tid)
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


import libcamera
from picamera2 import Picamera2

picamera = Picamera2()
config = picamera.create_preview_configuration(main={"format": 'RGB888', "size": (320, 240)},
                                               raw={"format": "SRGGB12", "size": (1920, 1080)})

config["transform"] = libcamera.Transform(hflip=False, vflip=True)
picamera.configure(config)
picamera.start()

width = 320
height = 240

hueLower = widgets.IntSlider(min=96, max=179, step=1, value=96, description='Hue lower')
hueUpper = widgets.IntSlider(min=120, max=179, step=1, value=120, description='Hue upper')

hue2Lower = widgets.IntSlider(min=50, max=255, step=1, value=50, description='Hue2lower')
hue2Upper = widgets.IntSlider(min=0, max=255, step=1, value=0, description='Hue2upper')

satLow = widgets.IntSlider(min=157, max=255, step=1, value=157, description='Sat lower')
satHigh = widgets.IntSlider(min=255, max=255, step=1, value=255, description='Sat upper')

valLow = widgets.IntSlider(min=100, max=255, step=1, value=100, description='Val lower')
valHigh = widgets.IntSlider(min=255, max=255, step=1, value=255, description='Val upper')

slider_img = widgets.VBox([
    widgets.HBox([hueLower, hueUpper]),
    widgets.HBox([hue2Lower, hue2Upper]),
    widgets.HBox([satLow, satHigh]),
    widgets.HBox([valLow, valHigh])
])

display(slider_img)


def video_display():
    global pan
    global tilt
    try:
        while True:
            frame = picamera.capture_array()
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            hueLow = hueLower.value
            hueUp = hueUpper.value
            hue2Low = hue2Lower.value
            hue2Up = hue2Upper.value
            Ls = satLow.value
            Us = satHigh.value
            Lv = valLow.value
            Uv = valHigh.value

            I_b = np.array([hueLow, Ls, Lv])
            u_b = np.array([hueUp, Us, Uv])

            I_b2 = np.array([hue2Low, Ls, Lv])
            u_b2 = np.array([hue2Up, Us, Uv])
            FGmask = cv2.inRange(hsv, I_b, u_b)
            FGmask2 = cv2.inRange(hsv, I_b2, u_b2)
            FGmaskComp = cv2.add(FGmask, FGmask2)
            FGmaskComp_img.value = bgr8_to_jpeg(FGmaskComp)
            contours, _ = cv2.findContours(FGmaskComp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                (x, y, w, h) = cv2.boundingRect(cnt)
                if area >= 50:
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 3)
                    objX = x + w / 2
                    objY = y + h / 2
                    errorPan = objX - width / 2
                    errorTilt = objY - height / 2
                    if abs(errorPan) > 15:
                        pan = pan - errorPan / 75
                    if abs(errorTilt) > 15:
                        tilt = tilt - errorTilt / 75
                    if pan > 180:
                        pan = 180
                        print("Pan Out of Range")
                    if pan < 0:
                        pan = 0
                        print("Pan Out of Range")
                    if tilt > 180:
                        tilt = 180
                        print("Tilt Out of Range")
                    if tilt < 0:
                        tilt = 0
                        print("Tilt Out of Range")

                    kit.servo[0].angle = 180 - pan
                    kit.servo[1].angle = 180 - tilt
                break
            frame_img.value = bgr8_to_jpeg(frame)
    except Exception as e:
        print(f"ビデオ表示中のエラー: {e}")
    finally:
        picamera.stop()


t = threading.Thread(target=video_display)
t.daemon = True  # Changed from setDaemon(True) to daemon = True
t.start()
