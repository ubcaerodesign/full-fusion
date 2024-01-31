import os
import sys
import cv2
import logging
import imutils
from copy import copy
from datetime import datetime
from utilities.config import Config
from pathlib import Path
import time
import atexit
import getpass

JPEG_QUALITY = 80


def add_name_to_image(image):
    user_name = getpass.getuser()
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.5
    font_color = (255, 255, 255)
    font_thickness = 1

    text_position = (0, 15)

    cv2.putText(
        image,
        user_name,
        text_position,
        font,
        font_scale,
        font_color,
        font_thickness,
    )


class Camera:
    def __init__(self, zmq_queue, data_folder):
        self.config = Config()
        self.use_camera = self.config.camera["use_camera"]
        self.show_images = self.config.camera["show_images"]
        self.image_format = self.config.img_format
        self.display_size = self.config.camera["display_size"]

        self.image_count = 0
        self.zmq_queue = zmq_queue
        self.data_folder = data_folder

        if self.use_camera:
            camera_source = self.config.camera["camera_source"]
        else:
            camera_source = str(
                Path(__file__).resolve().parent.parent
                / self.config.camera["video_file"].lower()
            )

        self.cap = cv2.VideoCapture(camera_source)
        atexit.register(self.dispose)

    def read_image(self):
        ret, frame = self.cap.read()

        # Loop fake video if not a webcam
        if not ret and not self.use_camera:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ret, frame = self.cap.read()
            if not ret:  # if we still can't read the frame, problem with video file
                logging.error("Error reading fake video file.")
                sys.exit(1)

        # Save data if we were using an actual camera
        if self.use_camera:
            if ret:
                save_ret = cv2.imwrite(
                    f"{self.data_folder}/{self.image_count}.jpg", frame
                )
                if not save_ret:
                    logging.info("Error saving image to disk.")
            else:
                logging.error("Error reading image from camera.")

        else:  # using fake data instead of real camera
            if self.image_count % 100 == 0:
                logging.info(f"Reading fake image {self.image_count}")

        if self.show_images:
            display = cv2.resize(frame, dsize=tuple(self.display_size))
            cv2.imshow(f"Camera", display)
            cv2.waitKey(1)

        # Resize image from 1080x1920p to 540x960p
        frame = imutils.resize(frame, width=384)

        if not self.use_camera:
            # add name to image, so we know who is using the server
            add_name_to_image(frame)

        # Jpegify image
        ret, jpg_buffer = cv2.imencode(
            ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
        )

        # Push img packet to zmq queue
        img_data = copy(self.image_format)
        img_data["image"] = jpg_buffer.tolist()
        img_data["timestamp"] = time.time_ns() // 1000000
        self.zmq_queue.put(img_data)
        self.image_count += 1

    def dispose(self):
        # Release unmanaged resources
        self.cap.release()
        cv2.destroyAllWindows()
