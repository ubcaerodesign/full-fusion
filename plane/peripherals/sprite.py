from pymavlink import mavutil
import logging


class Pada:
    def __init__(self):
        """Initialize PADA object"""
        # Wirelessly connect to PADA and wait for heartbeat
        self.pada_connection = mavutil.mavlink_connection("udpin:0.0.0.0:14551")
        self.pada_connection.wait_heartbeat()
        logging.info("Heartbeat received")

    def arm_pada(self):
        """Arm PADA"""
        # Arm PADA
        self.pada_connection.mav.command_long_send(
            self.pada_connection.target_system,
            self.pada_connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,
            0,
            0,
            0,
            0,
            0,
            0,
        )

        # Wait for PADA to arm
        self.pada_connection.motors_armed_wait()
        logging.info("Armed!")


if __name__ == "__main__":
    pada = Pada()
    pada.arm_pada()
