# Cabrillo ROV 2023

Cabrillo College Robotics Club repo for the MATE ROV 2023 competition

## Software

ROS 2 Humble Hawksbill (Ubuntu 22.04 LTS)

[ROS Documentation](http://docs.ros.org/en/humble/index.html)

## Building

The ROS nodes are built with the `colcon` program. A `makefile` is present to help me remember what the `colcon` commands are. Build the project with:

```console
make 
```

The built artifacts are in:

- `build`
- `install`
- `log`

Clean the build with:

```console
make clean 
```

## Launching all Nodes

There are launch configurations in the `launch` directory. To run all the configured nodes on the ROV:

```console
ros2 launch ./launch/rov.launch.py  
```

To launch all the nodes on the deck:

```console
ros2 launch ./launch/deck.launch.py  
```

## Running Individual Nodes

The built nodes are a part of a local build area. Source the following script to setup your paths:

```console
source install/setup.bash # Choose one that matches your shell.
```

Run the Python examples:

```console
ros2 run seahawk_rov example_pub
ros2 run seahawk_rov example_sub
```

## Launching the deck

Do this to launch the deck nodes:

```console
ros2 launch ./launch/deck.launch.py
ros2 topic echo /drive/twist
```

## Launching the Kinematics Debug Nodes

Do this to launch the debug nodes. It also starts RViz:

```console
ros2 launch launch/kinematics_viz.launch.py
```
