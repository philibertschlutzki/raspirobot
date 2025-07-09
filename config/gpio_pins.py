import yaml
import os

def read_pins():
    path = os.path.join(os.path.dirname(__file__), "gpio_pins.yaml")
    with open(path, "r") as f:
        return yaml.safe_load(f)
