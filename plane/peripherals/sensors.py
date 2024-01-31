from __future__ import print_function
import json
import serial
from time import sleep
import zmq
import logging
from filter import KalmanFilter

import sys
from pathlib import Path
sys.path.append(str(Path(__file__).resolve().parent.parent))
from fake_data.fake_flight_data_generator import fake_data_generator
from utilities.config import Config

class Sensors:
    def __init__(self, baud=115200, tries=30):
        self.config = Config()

        self.baud = baud
        self.tries = tries
        self.port = self.config.sensors["port"]
        self.fake = True
        self.generator = None
        self.filter = KalmanFilter()

        try:

            self.serial = serial.Serial(self.port, self.baud, timeout=1)
            # clear initial bad frames
            dog = self.read_data_raw()
            self.init_filter()
            logging.info("Sensor Board connection successful.")
        except Exception as e:
            print(e)
            logging.info("Sensor Board connection failed, using fake data.")
            self.fake = True

    def read_data_raw(self):
        """reads from the sensorboard uart and produces a dictionary for sensor fusion"""

        if self.fake:
            return fake_data_generator()
        else:
            for _ in range(self.tries):
                res = self.serial.readline()
                if res != b"":
                    print(res)
                    return json.loads(str(res)[2:][:-5])
            return None

    def read_data(self):
        """takes a reading from the sensor board and runs it through the kalman filter"""
        data = None
        while data is None:
            data = self.read_data_raw()
        # update kalman filter
        filtered_data = self.filter.filter_data(data)

        print(filtered_data)
        # get values


    def init_filter(self):
        self.filter.init(self.read_data_raw())

if __name__ == "__main__":
    sensors = Sensors()

    raw = True
    while True:
        if raw:
            data = sensors.read_data_raw()
            print(type(data))
            print(data)
        else:
            print(sensors.read_data())
        sleep(1)
