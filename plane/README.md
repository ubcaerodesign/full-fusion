# Plane

Plane is the Python module running on the plane raspberry pi. It consists of driver classes to talk to sensors (gps, imu, etc), an on-board camera, and the pada drop servo. It also consists of a higher level communication class to publish sensor + image data to the server and send target gps coordinates to the pada.

## Getting started
See Codex's README/Releases on how to run Plane with everything else using Docker.

To run Plane on its own, you will need to ensure the server script is running (see [How to Connect to Server](https://docs.google.com/document/d/1o-YZdZj_K0m3nOGJ-WIMQkkOSGmo8YLEUohOpQV5k74/edit)) and that the rest of Codex's Poetry setup is complete, then it's simply `python plane/plane.py`s
