import serial
import time
from utilities.config import Config
import logging
import json

class Radio:
    def __init__(self, baud=57600, tries=10):
        self.config = Config()

        self.baud = baud
        self.tries = tries
        self.port = self.config.radio["port"]

        try:
            self.serial = serial.Serial(self.port, self.baud, timeout=0)
            # shouldn't need to clear bad initial frames in buffer
            logging.info("Radio connection successful")
        except Exception as e:
            print(e)
            logging.info("Radio connection failed")
            
    def send_data(self, data):
        json_data = json.dumps(data, indent=1)
        serial_message_bytes = (json_data + '\n').encode('utf-8')
        self.serial.write(serial_message_bytes)

    def read(self):
        res = self.serial.readline()

        for _ in range(self.tries):
            if res != b"":
                print(str(res))
                return json.loads(str(res)[2:][:-3])
        return None
        
if __name__ == "__main__":
    radio = Radio()
    # while True:
    #     print(radio.read())
    while True:

        radio.send_message("balls!")
