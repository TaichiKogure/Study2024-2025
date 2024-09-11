import cv2
import numpy as np
from PIL import Image, ImageTk
import tkinter as tk
from adafruit_servokit import ServoKit
import threading
import time

kit = ServoKit(channels=16)
pan = 20
tilt = 20

# Initial position
kit.servo[0].angle = pan
kit.servo[1].angle = tilt


def bgr8_to_jpeg(value, quality=75):
    return Image.fromarray(cv2.cvtColor(value, cv2.COLOR_BGR2RGB))


class App:
    def __init__(self, window, window_title):
        self.window = window
        self.window.title(window_title)

        # Video Frame
        self.video_frame_label = tk.Label(window)
        self.video_frame_label.pack()

        # Servos frame
        self.servo_frame = tk.Frame(window)
        self.servo_frame.pack()

        # Sliders for manual control
        self.pan_slider = self.create_slider(self.servo_frame, 'Pan', pan, 180, 1, self.update_servo_angles)
        self.tilt_slider = self.create_slider(self.servo_frame, 'Tilt', tilt, 180, 1, self.update_servo_angles)

        # Sliders for color settings
        self.controls_frame = tk.Frame(window)
        self.controls_frame.pack()

        self.hueLower = self.create_slider(self.controls_frame, 'Hue lower', 96, 179, 1, self.update_color_preview)
        self.hueUpper = self.create_slider(self.controls_frame, 'Hue upper', 120, 179, 1, self.update_color_preview)
        self.hue2Lower = self.create_slider(self.controls_frame, 'Hue2 lower', 50, 255, 1, self.update_color_preview)
        self.hue2Upper = self.create_slider(self.controls_frame, 'Hue2 upper', 0, 255, 1, self.update_color_preview)
        self.satLow = self.create_slider(self.controls_frame, 'Sat lower', 157, 255, 1, self.update_color_preview)
        self.satHigh = self.create_slider(self.controls_frame, 'Sat upper', 255, 255, 1, self.update_color_preview)
        self.valLow = self.create_slider(self.controls_frame, 'Val lower', 100, 255, 1, self.update_color_preview)
        self.valHigh = self.create_slider(self.controls_frame, 'Val upper', 255, 255, 1, self.update_color_preview)

        # Color Preview
        self.color_preview = tk.Label(window, text="Color Preview", bg="white")
        self.color_preview.pack(pady=10)

        # Stop Button
        self.stop_button = tk.Button(window, text='Stop', command=self.stop_video_display)
        self.stop_button.pack()

        # Stop event
        self.stop_event = threading.Event()
        self.video_thread = threading.Thread(target=self.video_display, args=(self.stop_event,))
        self.video_thread.daemon = True
        self.video_thread.start()

        self.window.mainloop()

    def create_slider(self, parent, label, value, max_value, step, command=None):
        frame = tk.Frame(parent)
        frame.pack()
        label = tk.Label(frame, text=label)
        label.pack(side=tk.LEFT)
        slider = tk.Scale(frame, from_=0, to=max_value, orient=tk.HORIZONTAL, command=command)
        slider.set(value)
        slider.pack(side=tk.RIGHT)
        return slider

    def update_servo_angles(self, _=None):
        global pan
        global tilt
        try:
            pan = self.pan_slider.get()
            tilt = self.tilt_slider.get()

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
        except IOError as e:
            if e.errno == 121:
                print("I2C通信エラー: Remote I/O error (Errno 121)")
            else:
                print(f"その他のI/Oエラー: {e}")

    def stop_video_display(self):
        self.stop_event.set()
        self.video_thread.join()

    def update_color_preview(self, _=None):
        lower_hue = self.hueLower.get()
        upper_hue = self.hueUpper.get()
        lower_hue2 = self.hue2Lower.get()
        upper_hue2 = self.hue2Upper.get()
        sat_low = self.satLow.get()
        sat_high = self.satHigh.get()
        val_low = self.valLow.get()
        val_high = self.valHigh.get()

        # Create a small image with the selected color range
        lower_bound = np.uint8([[[lower_hue, sat_low, val_low]]])
        upper_bound = np.uint8([[[upper_hue, sat_high, val_high]]])
        lower_bound2 = np.uint8([[[lower_hue2, sat_low, val_low]]])
        upper_bound2 = np.uint8([[[upper_hue2, sat_high, val_high]]])

        img = np.zeros((100, 200, 3), np.uint8)
        img[:, :100] = cv2.cvtColor(lower_bound, cv2.COLOR_HSV2BGR)
        img[:, 100:] = cv2.cvtColor(upper_bound, cv2.COLOR_HSV2BGR)
        img[:, :100] = cv2.cvtColor(lower_bound2, cv2.COLOR_HSV2BGR)
        img[:, 100:] = cv2.cvtColor(upper_bound2, cv2.COLOR_HSV2BGR)

        img = Image.fromarray(img)
        img_tk = ImageTk.PhotoImage(image=img)
        self.color_preview.config(image=img_tk)
        self.color_preview.image = img_tk

    def video_display(self, stop_event):
        global pan
        global tilt
        try:
            picamera = Picamera2()
            config = picamera.create_preview_configuration(main={"format": 'RGB888', "size": (320, 240)},
                                                           raw={"format": "SRGGB12", "size": (1920, 1080)})
            config["transform"] = libcamera.Transform(hflip=False, vflip=True)
            picamera.configure(config)
            picamera.start()

            while not stop_event.is_set():
                frame = picamera.capture_array()
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

                hueLow = self.hueLower.get()
                hueUp = self.hueUpper.get()
                hue2Low = self.hue2Lower.get()
                hue2Up = self.hue2Upper.get()
                Ls = self.satLow.get()
                Us = self.satHigh.get()
                Lv = self.valLow.get()
                Uv = self.valHigh.get()

                I_b = np.array([hueLow, Ls, Lv])
                u_b = np.array([hueUp, Us, Uv])

                I_b2 = np.array([hue2Low, Ls, Lv])
                u_b2 = np.array([hue2Up, Us, Uv])
                FGmask = cv2.inRange(hsv, I_b, u_b)
                FGmask2 = cv2.inRange(hsv, I_b2, u_b2)
                FGmaskComp = cv2.add(FGmask, FGmask2)

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
                        self.update_servo_angles()

                    break

                frame_img = bgr8_to_jpeg(frame)
                frame_photo = ImageTk.PhotoImage(image=frame_img)
                self.video_frame_label.config(image=frame_photo)
                self.video_frame_label.image = frame_photo

        except Exception as e:
            print(f"ビデオ表示中のエラー: {e}")
        finally:
            picamera.stop()


if __name__ == "__main__":
    App(tk.Tk(), "Object Tracking with Servo Control")
