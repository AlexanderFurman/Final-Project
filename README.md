# Sensor Integrated Pneumatic Gripper

This project envolves the design, manufacturing, software integration of a robotic gripper.

## Installation
Start by installing rosserial: http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup

After loading relevant code onto the Arduino, use the following command to remotely control the gripper from your computer: 

```bash
rosrun rosserial_python serial_node.py /dev/tty<port-connected-to-arduino>
```
You can now integrate the gripper into any ROS Project you are working on.

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

## Gripper Version 1 Demo


<img src="https://raw.githubusercontent.com/AlexanderFurman/Final-Project/main/Demos/initial_gripper_test.gif" alt="Demonstration of Gripper 1">

## Gripper Version 2 Demo

<img src="https://raw.githubusercontent.com/AlexanderFurman/Final-Project/main/Demos/gripper_test.gif" alt="Demonstration of Gripper 2">

## Project Presentation

[Link to Project Presentation PDF](https://raw.githubusercontent.com/AlexanderFurman/Final-Project/main/Presentation/final_presentation.pdf)

