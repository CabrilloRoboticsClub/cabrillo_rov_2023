# Cabrillo ROV 2023

Cabrillo College Robotics Club repo for the MATE ROV 2023 competition

## Software

ROS 2 Humble Hawksbill (Ubuntu 22.04 LTS)

[ROS Documentation](http://docs.ros.org/en/humble/index.html)

## Getting Started 

Take the following steps to get started. 

1. Install necessary packages (skip this if you're using a removable drive provided by the club)
    ```console
    $ sudo apt update -y && sudo apt install -y git ansible python3-pip vim 
    ```
1. Create an SSH key if you don't have one already. 
    ```console 
    $ ssh-keygen -t ed25519
    ```
    Use the default location, set a password if you like. 
1. Copy and paste your public key into GitHub
    ```console
    $ cat ~/.ssh/id_ed25519.pub
    ```
1. Check out this repository into your home directory. 
    ```console
    $ cd 
    $ git clone git@github.com:CabrilloRoboticsClub/cabrillo_rov_2023.git
    $ cd ~/cabrillo_rov_2023
    $ git submodule init 
    $ git submodule update
    ```
1. Use `make` to install ROS2 and all the necessary packages. 
    ```console 
    $ make devbox
    ```
    This will take a while! 
1. Put ROS on your path 
    ```console
    echo 'if [ -z "$ROS_DISTRO"]; then source /opt/ros/humble/setup.bash; fi' >> ~/.bashrc 
    ```
1. **Start a new shell** and build the repository
    ```console
    $ make 
    ```

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

We've migrated all the launch files to their respective ROS packages. To run all the configured nodes on the ROV:

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

## Launching the deck

Do this to launch the deck nodes:

```console 
$ ros2 launch seahawk_deck deck.launch.py
$ ros2 topic echo /drive/twist
```

## Launching the Kinematics Debug Nodes

Do this to launch the debug nodes. It also starts RViz:

```console 
$ ros2 launch seahawk_deck kinematics_viz.launch.py
```
