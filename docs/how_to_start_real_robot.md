
# How to start the real robot

## Prerequisites

You need to configure your Raspberry Pi (see [documentation](docs/config_environnement_raspi.md)) and have a basic understanding of ROS2.

Update the system and install the necessary packages:

```bash
sudo apt update && sudo apt upgrade
sudo apt install ros-humble-xacro ros-humble-teleop-twist-keyboard ros-humble-ros2-control
```


And install dependencies:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

## Building the workspaces

/!\ **I recommend compiling on your PC and then transferring the compiled files to the Raspberry Pi.**
However, sometimes it might be necessary to compile on the Raspberry Pi. 
For example, we had to compile on RPY because the `can_interface` package was not really found when we tried to compile on our PC.

Then compile the workspace with `colcon`:

YOU MUST BE IN THE `~/ros2_ws` DIRECTORY TO RUN THIS COMMAND.
```bash
colcon build --symlink-install --packages-select can_robot robot_simu can_interface my_robot_control robot_description
```

## Running the robot
Then source with `source install/setup.bash`.

Run the following command to start the robot:

```bash
ros2 launch robot_simu launch_real.launch.py
```

To control the robot, you can use the `teleop_twist_keyboard` package(SSH into the Raspberry Pi):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

You can also check some topics with `rqt`:

```bash
ros2 run rqt_topic rqt_topic
```

or easily with `ros2 topic echo /topicname`.