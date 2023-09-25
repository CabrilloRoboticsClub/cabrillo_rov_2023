# Cabrillo ROV 2023

Cabrillo College Robotics Club repo for the MATE ROV 2023 competition

## Software

ROS 2 Humble Hawksbill (Ubuntu 22.04 LTS)

[ROS Documentation](http://docs.ros.org/en/humble/index.html)

## Getting Started 

Check out the [README](./setup/README.md) in the `setup` directory. 

## Building

The ROS nodes are built with the `colcon` program. A `makefile` is present to help me remember what the `colcon` commands are. Build the project with:

```console
make 
```

The build artifacts are in:

- `build`
- `install`
- `log`

Clean the build with:

```console
make clean 
```

## Starting the ROV and Deck ROS2 Nodes

The easiest way to launch is by `make`. To launch the deck nodes and RQT viewer run:

```console
$ make deck
```

To launch all the nodes on the ROV run: 

```console
$ make rov
```

## Launch Files 

We've migrated all the launch files to their respective ROS packages. To use the launch files you must first run the `install/local_setup.XXX` script. 

```console 
ros2 launch seahawk_rov rov.launch.py  
```

To launch all the nodes on the deck:

```console 
ros2 launch seahawk_deck deck.launch.py  
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

## Launching the Kinematics Debug Nodes

Do this to launch the debug nodes. It also starts RViz:

```console 
$ ros2 launch seahawk_deck kinematics_viz.launch.py
```

You have to separately launch the other deck nodes. 
