import math
import random

tbird_rc = (32.60979, -97.48397)


def fake_data_generator(center=tbird_rc):
    # Fake generator that produces lat, long, altitude
    # generator that creates gps points in an oval around tbird stadium with an angle of -40 from the x axis
    # math is from https://math.stackexchange.com/a/2647450
    angle = -100  # degrees
    off_angle = -0 * math.pi / 180
    Rx = 0.0008
    Ry = 0.0004
    while True:
        # 1. Get the lat long of the flight path
        angle += 2
        angle %= 360
        rad = angle * math.pi / 180
        lat = (
            center[0]
            + Rx * math.cos(rad) * math.cos(off_angle)
            - Ry * math.sin(rad) * math.sin(off_angle)
        )
        long = (
            center[1]
            + Rx * math.cos(rad) * math.sin(off_angle)
            + Ry * math.sin(rad) * math.cos(off_angle)
        )

        # 2. Get the altitude
        altitude = 15 + random.uniform(-0.5, 0.5)

        yield lat, long, altitude
