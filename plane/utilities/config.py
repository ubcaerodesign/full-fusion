import os
import yaml
from pathlib import Path
from dotenv import load_dotenv


class Config:
    def __init__(self):
        # if running with test.py (--local or otherwise), we set use_server with Popen instead of dotenv
        # this ensures test.py without local uses the remote server and test.py --local uses the local server,
        # regardless of the .env file. .env should be used when running without test.py.
        if "remote_server" not in os.environ:
            load_dotenv()
        self.load_yaml_config()
        self.remote_server = os.environ["remote_server"].lower() == "true"

    def load_yaml_config(self):
        config_path = Path(__file__).resolve().parent.parent / "config.yaml"
        with open(config_path, "r") as yml_file:
            try:
                # Config yaml object
                config = yaml.safe_load(yml_file)

                # Config settings
                self.plane = config["plane"]
                self.camera = config["camera"]
                self.sensors = config["sensors"]
                self.radio = config["radio"]
                self.servo = config["servo"]
                self.logging = config["logging"]
                
                # Data packet formats
                self.data_format = config["sensor_data_structure"]
                self.img_format = config["img_data_structure"]
                self.state_format = config["state_data_structure"]
            except yaml.YAMLError as exc:
                print(exc)
