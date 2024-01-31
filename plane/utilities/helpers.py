import time
import random
import os
import io
import cv2


def millis():
    return time.time_ns() // 1000000


def bash():
    os.system("bash zapdos.sh")


def fake_data_gen(count):
    return {
        "altitude": round(min(count / 10, 15) + random.uniform(-0.5, 0.5), 4),
        "pressure": 1.01 + random.uniform(-0.001, 0.001),
        "latitude": round(32.768 + count / 1000, 4),
        "longitude": round(-97.309 + count / 1000, 4),
        "pitch": round(random.uniform(-15, 15), 4),
        "roll": round(random.uniform(-15, 15), 4),
        "yaw": round(random.uniform(-15, 15), 4),
        "ax": round(random.uniform(-15, 15), 4),
        "ay": round(random.uniform(-15, 15), 4),
        "az": round(random.uniform(-15, 15), 4),
        "gx": round(random.uniform(-15, 15), 4),
        "gy": round(random.uniform(-15, 15), 4),
        "gz": round(random.uniform(-15, 15), 4),
        "mx": round(random.uniform(-15, 15), 4),
        "my": round(random.uniform(-15, 15), 4),
        "mz": round(random.uniform(-15, 15), 4),
        "timestamp": millis(),
    }


def convert_tuple(tup):
    # initialize an empty string
    str = ""
    for item in tup:
        str = str + item
    return str


def convert_row_to_ascii(row):
    ORDER = (
        " ",
        ".",
        "'",
        ",",
        ":",
        ";",
        "c",
        "l",
        "x",
        "o",
        "k",
        "X",
        "d",
        "O",
        "0",
        "K",
        "N",
    )

    line = tuple(ORDER[int(x / (255 / 16))] for x in row)[::-1]
    string = convert_tuple(line)
    return string


def print_to_ascii(frame):
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    for row in frame:
        print(convert_row_to_ascii(row))


def is_raspberrypi():
    try:
        with io.open("/sys/firmware/devicetree/base/model", "r") as m:
            if "raspberry pi" in m.read().lower():
                return True
    except Exception:
        pass
    return False


if __name__ == "__main__":
    print(bash())
