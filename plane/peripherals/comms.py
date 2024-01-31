"""
Probably read our ZMQ docs if anything doesn't make sense:
https://docs.google.com/document/d/1J_Gdc7iPWV1LekobKUTpKMzVt-vNSunPHbohVICuwz4/edit
"""
import time
import logging
from threading import Thread
import sys
from pathlib import Path

import zmq
from zmq.error import Again

# python doesn't allow relative imports in scripts that are run directly, only in modules, so do this path hack
from helpers.networking import make_zmq_socket
sys.path.append(str(Path(__file__).resolve().parent.parent.parent))
from utilities.config import Config


class PlanePub:
    def __init__(self, zmq_queue):
        """
        Transmit data to the server. If remote_server, connect to it, otherwise connect to localhost.
        Args:
            zmq_queue: queue where we put data and images into
        """
        self.config = Config()
        server_ip = (
            self.config.plane["remote_server_ip"]
            if self.config.remote_server
            else "127.0.0.1"
        )
        self.data_pub = make_zmq_socket(
            server_ip, self.config.plane["pub_port"], "plane", "connect", "pub"
        )

        # Start new thread to continuously read from zmq queue and and sends to publisher socket
        zmq_publisher = Thread(target=self.send, daemon=True, args=(zmq_queue,))
        zmq_publisher.start()

    def send(self, zmq_queue):
        """
        Reads from the queues to try and find matching data-image pairs,
        and sends them, async or something i hope
        NOTE: we may want to switch to a system where we always send data if available, and add image when available
        """
        while True:
            try:
                item = zmq_queue.get(block=False)
            except:
                continue

            logging.debug(f"Publishing data of type {item['type']}")
            self.data_pub.send_json(item)


class PlaneSub:
    def __init__(self):
        """

        Receive commands from the ground. If remote_server, connect to it, otherwise bind to 0.0.0.0 to listen to
        the server running in Docker.
        """
        self.config = Config()

        if self.config.remote_server:
            ip = self.config.plane["remote_server_ip"]
            self.plane_sub = make_zmq_socket(
                ip, self.config.plane["sub_port"], "plane", "connect", "sub"
            )
        else:
            self.plane_sub = make_zmq_socket(
                "0.0.0.0", self.config.plane["sub_port"], "plane", "bind", "sub"
            )
        time.sleep(1)

    def read(self):
        try:
            message = self.plane_sub.recv_json(flags=zmq.NOBLOCK)
            return message
        except Again:
            pass
