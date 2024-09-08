import cv2
import numpy as np
from adafruit_servokit import ServoKit
import time

kit = ServoKit(channels=16)
pan = 90
tilt = 90
kit.servo[0].angle = pan
kit.servo[1].angle = tilt
