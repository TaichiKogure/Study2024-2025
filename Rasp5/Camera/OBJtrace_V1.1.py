from collections import deque
import numpy as np
import argparse
import imutils
import cv2
import threading
import ctypes
import inspect
import libcamera
from picamera2 import Picamera2
import ipywidgets.widgets as widgets
from IPython.display import display

# Argument parsing (consider removing args=[] in real usage)
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64, help="max buffer size")
args = vars(ap.parse_args(args=[]))

# Color range for masking
colorLower = (24, 100, 100)
colorUpper = (44, 255, 255)

# Deque for buffer
pts = deque(maxlen=args["buffer"])

# Setup the camera
picamera = Picamera2()
config = picamera.create_preview_configuration(main={"size": (640, 480)},
                                               raw={"size": (1920, 1080), "format": "SRGGB12"})
config["transform"] = libcamera.Transform(hflip=True, vflip=True)
picamera.configure(config)
picamera.start()


def bgr8_to_jpeg(value, quality=75):
    return bytes(cv2.imencode('.jpg', value)[1])


# For displaying the video frame
Frame = widgets.Image(format='jpeg', width=640, height=480)
display(Frame)


def video_display():
    try:
        while True:
            frame = picamera.capture_array()
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
            Frame.value = bgr8_to_jpeg(frame)
    except Exception as e:
        print(f"Error in video display: {e}")
    finally:
        picamera.stop()
        picamera.close()


# Start the display thread
t = threading.Thread(target=video_display)
t.start()

# Placeholder for proper synchronization and stopping the thread, if needed
# t.join()  # If you want to wait for the thread to finish
