# How to start the Gazebo simulation

## Prerequisites

Packages should already be installed in the dev container. See `Dockerfile` for more information.

/!\ The RPy is arm64 architecture, so you cannot run the simulation on the RPy.

## Running the simulation

1. Compile the workspace with `colcon`:
```bash
colcon build --symlink-install
```

2. Source the workspace (in `~/ros2_ws`) with:
```bash
source install/setup.bash
```

3. Run the following command to start the simulation:
```bash
ros2 launch robot_simu launch_sim.launch.py
```
