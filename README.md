# Sensor Integrated Pneumatic Gripper

This project envolves the design, manufacturing, software integration of a robotic gripper.

## Installation

Start by installing rosserial: http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup

After loading relevant code onto the Arduino, use the following command to remotely control the gripper from your computer: 

```bash
rosrun rosserial_python serial_node.py /dev/tty<port-connected-to-arduino>
```

## Usage

```python
from gripper_code import Gripper

#create gripper object, which also initializes relevant node, publishers, subscribers
gripper = Gripper()

#Toggle distance between gripper's fingers [mm]
gripper.toggle_dist(37.5)

#Control suction
gripper.start_suction()
gripper.stop_suction()
```

## Demo

![Basic Demonstration of Gripper](image_url)

## Project Presentation

![Basic Demonstration of Gripper](image_url)

