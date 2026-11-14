# ROS 2 3D Perception and Local Obstacle Avoidance for a Mobile Robot

This project develops a local obstacle-avoidance system for a mobile robot. It combines **3D perception from depth-camera images** with ROS 2 nodes that detect obstacles, assess the available space, and determine how the robot should react.

The project has two implementations:

- A **Gazebo simulation** for testing the robot and obstacle-avoidance behaviour in a virtual environment.
- A **real-robot implementation** using an HP60C depth camera, ROS 2, a Raspberry Pi, and an Arduino Mega-based mobile robot.

The system is designed to select a safe direction when possible and stop the robot when no safe direction is available.

## Project objectives

- Read depth images from the camera.
- Process depth data to identify nearby obstacles.
- Estimate which areas around the robot are free.
- Select a local direction that avoids obstacles.
- Stop the robot when the available space is not considered safe.
- Compare the system's behaviour in simulation and on the real robot.

## How the system works

The software is divided into ROS 2 nodes, with each node handling part of the process:

1. **Depth-camera input** — provides depth images to the perception node.
2. **3D perception** — `perception_3d.py` processes the depth data and publishes obstacle information.
3. **Local obstacle avoidance** — `evitement.py` uses the obstacle information to choose a movement direction or stop.
4. **Robot command** — on the real robot, `commande_arduino.py` handles the command path to the Arduino Mega, which controls the motors through the robot's motor-driver hardware.

The overall data flow is:

```text
Depth camera
     ↓
3D perception (`perception_3d.py`)
     ↓
Obstacle information (`/obstacles`)
     ↓
Local avoidance (`evitement.py`)
     ↓
Motion command (`/cmd_vel`)
     ↓
Arduino interface and mobile robot
```

The camera, perception, and avoidance components communicate through ROS 2 topics. The real-robot setup also includes an Arduino Mega interface for controlling the robot's motors.

## Simulation

The `Simulation/` directory contains the files for testing the system in Gazebo:

- `gazebo/worlds/couloir.world` — the Gazebo corridor world.
- `nodes_ros2/perception_3d.py` — processes depth data for obstacle perception.
- `nodes_ros2/evitement.py` — implements local obstacle avoidance.
- `robot_description/robot.urdf` — describes the robot's structure for simulation.
- `robot_description/meshes/` — contains 3D mesh files used by the robot description.

The simulation makes it possible to observe the robot's response to obstacles in a controlled environment before testing on the physical robot.

## Real-robot implementation

The `real_robot/` directory contains the code and camera information for the physical robot:

- `arduino/code_arduino_mega.ino` — Arduino Mega firmware.
- `camera_HP60C/` — information about the HP60C depth camera and its ROS 2 stream.
- `nodes_ros2/perception_3d.py` — processes the real camera's depth images.
- `nodes_ros2/evitement.py` — selects a safe local direction or stops the robot.
- `nodes_ros2/commande_arduino.py` — handles the ROS 2 command interface with the Arduino Mega.

The real mobile robot uses an Arduino Mega-based control system, four motors, and an L298N motor driver. A Raspberry Pi is used as the onboard computer for the ROS 2 components.

## HP60C depth camera

The real-robot implementation uses an **HP60C depth camera**. Its ROS 2 driver publishes depth images on:

```text
/ascamera_hp60c/camera_publisher/depth0/image_raw
```

The image encoding is `16UC1`. Depth values are provided in millimeters and converted to meters in `perception_3d.py`.

Start the camera driver with:

```bash
ros2 launch ascamera hp60c.launch.py
```

Check the depth-stream frequency with:

```bash
ros2 topic hz /ascamera_hp60c/camera_publisher/depth0/image_raw
```

List the available ROS 2 topics with:

```bash
ros2 topic list
```

## Repository structure

```text
.
├── Simulation/
│   ├── gazebo/
│   │   └── worlds/
│   │       └── couloir.world
│   ├── nodes_ros2/
│   │   ├── evitement.py
│   │   └── perception_3d.py
│   └── robot_description/
│       ├── meshes/
│       └── robot.urdf
├── real_robot/
│   ├── arduino/
│   │   └── code_arduino_mega.ino
│   ├── camera_HP60C/
│   └── nodes_ros2/
│       ├── commande_arduino.py
│       ├── evitement.py
│       └── perception_3d.py
└── README.md
```

## Main technologies and hardware

- ROS 2 Humble
- Python
- Gazebo
- URDF
- HP60C depth camera
- Raspberry Pi
- Arduino Mega
- L298N motor driver
- Four-motor mobile robot

## Running the project

The repository does not currently include ROS 2 launch files for starting the complete system in one command. The camera driver can be started using the command shown above. The other nodes should be run according to their ROS 2 workspace and package setup.

## Large mesh files

Some mesh files are not included because they exceed GitHub's **25 MB limit for uploading files through the website**. See the README inside `Simulation/robot_description/meshes/` for details about those files.

## Contact

For more information about this project, feel free to contact me:

- **Email:** [talebmedbouamatou4.com](mailto:talebmedbouamatou4.com)
- **Portfolio:** [Visit my portfolio](https://talebmedbouamatou.github.io)
