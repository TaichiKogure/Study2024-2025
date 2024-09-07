import libcamera
from picamera2 import Picamera2

picamera = Picamera2()

config = picamera.create_preview_configuration(main={"format": 'RGB888',"size": (640, 480)},
                                               raw={"format": "SRGGB12", "size": (1920, 1080)})

config["transform"] = libcamera.Transform(hflip=False, vflip=True)

picamera.configure(config)
picamera.start()