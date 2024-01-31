#!/usr/bin/python

# Authors: Many
"""
In the realm of the intangible,
Where thoughts take flight and soar,
We bring form to our creations,
Building aircraft evermore.

With calculations and designs,
We forge our dreams in metal and air,
Engineered to soar beyond the clouds,
Our art, a tribute to man's flair.

The mystery of flight is a symphony,
Of science and spirit intertwined,
A dance of mechanics and magic,
As our machines are made to glide.

And as they take to the sky,
We bask in their graceful allure,
A creation of engineering and art,
An ethereal beauty to endure.

- ChatGPT
"""

import os
import json
import yaml
import traceback
import logging
import sys
from copy import copy
from datetime import datetime
from queue import Queue
from time import sleep
from utilities.states import States, StateMachine
from utilities.helpers import *
from utilities.config import Config
from peripherals.comms import PlanePub, PlaneSub
from peripherals.sensors import Sensors
from peripherals.radio import Radio
from peripherals.servos import Servos
from peripherals.sprite import Pada
from peripherals.camera import Camera
from dotenv import load_dotenv
from pathlib import Path


class Plane:
    def __init__(self):
        # Init config
        self.config = Config()

        # Init plane state
        self.state = (
            States.COLLECT
            if self.config.plane["start_state"].lower() == "collect"
            else States.IDLE
        )

        # Init zmq publishing queue
        self.zmq_queue = Queue()

        # Init logging and comms
        self.init_logging()
        self.init_comms()

        # Init sensors, sensor fusion, servos, and camera
        self.sensors = Sensors()
        self.radio = Radio()
        self.servos = Servos() if is_raspberrypi() else None
        self.camera = Camera(self.zmq_queue, self.data_folder)

        if self.sensors.fake:
            logging.info("Using randomly generated data, is the Pixhawk connected?")
        if not self.servos:
            logging.info("Couldn't connect to GPIO, is this running on a Pi?")

        # Run state machine
        try:
            self.fly()
        except Exception as e:
            logging.error(f"Exiting state machine. {e}")
            traceback.print_exc()

    def preflight_checks(self):
        """Calibrate altitude and starting sensor threads"""
        logging.info("Performing preflight checks.")
        # something should go here one day
        logging.info("Preflight check done.")

    def fly(self):
        # Start state machine
        sm = StateMachine(self.state)
        logging.info("Starting State Machine.")
        self.preflight_checks()

        target_lat = None
        target_long = None

        # Working loop
        while sm.state != States.STOP:
            # Set self.state to sm.state
            if self.state != sm.state:
                # Push plane state data packet to zmq queue
                logging.info("State changed to " + str(sm.state)[7:])
                state_data = copy(self.config.state_format)
                state_data["state"] = str(sm.state)[
                    7:
                ]  # Need to convert to string or else ZMQ can't serialize the data
                state_data["timestamp"] = time.time_ns() // 1000000
                self.zmq_queue.put(state_data)

            self.state = sm.state

            # Read ZMQ socket for new commands from ground
            command = self.read_command()
            print(command)

            if self.state == States.IDLE:
                """Just send sensor data, just wait for ground to send start command"""
                if command and command.get("type") == "capture":
                    self.camera.read_image()

            elif self.state == States.COLLECT:
                """Send image + sensor data"""
                self.camera.read_image()
                flight_data = self.sensors.read_data()

                # update to use fusion
                filtered_data = flight_data

                self.radio.send_data(filtered_data)
                
                if command and command.get("type") == "set_target":
                    # Target validation, only change state if target != None
                    if command.get("latitude") and command.get("longitude"):
                        target_lat = command.get("latitude")
                        target_long = command.get("longitude")
                        sm.state = States.GPS_AVAIL

            elif self.state == States.GPS_AVAIL:
                """Get GPS coordinates from command and arm PADA"""
                logging.info("Got GPS coordinates, sending to PADA.")
                sm.state = States.ARMING_PADA

            # Blocking until PADA is armed
            elif self.state == States.ARMING_PADA:
                logging.info("Waiting for PADA to finish arming.")
                self.arm_pada(target_lat, target_long)
                sm.state = States.READY

            elif self.state == States.READY:
                logging.info("Dropping payload.")
                if self.servos:
                    self.servos.drop()
                sm.state = States.DROPPED

            elif self.state == States.DROPPED:
                logging.info("PADA Dropped, continuing to collect data.")
                sm.state = States.COLLECT

            elif self.state == States.STOP:
                self.camera.dispose()

            # Handle manual override for state
            if command and command.get("type") == "set_state":
                force_state = command.get("state")
                sm.state = sm.states_lookup[force_state]

            # Handle manual override for PADA release
            if command and command.get("type") == "force_drop":
                sm.state = States.READY
    
    def init_logging(self):
        """Sets up file directory for writing logs and data."""
        now = datetime.now()
        timestamp = now.strftime("%Y_%m_%d_%H.%M.%S")

        logging_folder = (
            Path(__file__).resolve().parent / self.config.logging["directory"].lower()
        )

        self.data_folder = logging_folder / f"{timestamp}"
        log_file = self.data_folder / "log.txt"

        # Check if the directories exist and if they don't, make them
        if not os.path.exists(self.data_folder):
            os.makedirs(self.data_folder)
        with open(log_file, "x", encoding="utf-8") as f:
            pass

        # Configure logging
        logging.basicConfig(
            level=logging._nameToLevel[self.config.logging["level"].upper()],
            format="%(asctime)s:%(msecs)03d [%(filename)s - %(levelname)s] \t\t %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S",
            handlers=[
                logging.FileHandler(log_file),
                logging.StreamHandler(sys.stdout),
            ],
        )

        logging.info("Logging configured successfully.")

    def init_comms(self):
        """
        Starts all the sockets used to transmit and receive flight data and images
        """
        try:
            self.plane_pub = PlanePub(self.zmq_queue)
            self.plane_sub = PlaneSub()
            logging.info("Sucessfully initialized communication nodes.")

        except Exception as e:
            traceback.print_exc()
            logging.error(f"Error initializing sockets: {e}")
            raise e

    def read_command(self):
        """
        Tries to read from the subscriber socket,
        unpacks it, and returns the message received
        """
        try:
            message = self.radio.read()
            if message:
                logging.info(f"Received message from ground: {message}")
            return message

        except Exception as e:
            logging.warning(f"Error reading ground message: {e}")
            traceback.print_exc()
            return None

    def arm_pada(self, target_lat, target_long):
        self.pada = Pada()
        self.pada.arm_pada()
        logging.info("Arming PADA.")


if __name__ == "__main__":
    plane = Plane()
