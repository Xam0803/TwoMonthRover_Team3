# Orange Pi Onboard Rover Software

This module handles onboard sensor acquisition and arm control execution.

## I2C Pin Mapping
- BMP388 & BNO055: Connected via shared I2C bus (SDA / SCL).

## Dependencies
pip install adafruit-circuitpython-bmp388 adafruit-circuitpython-bno055

## Running the Onboard Node
source /opt/ros/humble/setup.bash
python3 orange_pi/rover_onboard_node.py
