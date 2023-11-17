# Cabrillo ROV 2023

Cabrillo College Robotics Club repo for the MATE ROV 2023 competition

## Setup GitHub

In order to use this repository you should have generated SSH keys and uploaded your public key to GitHub. If you have not yet created an SSH key on your machine do the following. **Skip these steps if you have an SSH key in GitHub already**. 

1. Create an SSH key if you don't have one already. 
    ```console 
    ssh-keygen -t ed25519
    ```
    Use the default location, set a password if you like. 
1. Copy and paste your public key into GitHub
    ```console
    cat ~/.ssh/id_ed25519.pub
    ```
    Full instructions for how to add a public key to GitHub can be found in [GitHub's official documentation](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account#adding-a-new-ssh-key-to-your-account).


## Getting Started (Mac and Windows)

If you have a Windows or Mac desktop (or Linux that's not Ubuntu 22.04) you can use Docker and Development Containers in vscode to do local development. This is the fastest and easiest way to get setup and eables you to run and test nodes on your computer. That's good for learning but you won't be able to use the ROS2 network or use devices like a joystick or camera.

1. [Install vscode](https://code.visualstudio.com/)
1. [Install Docker Desktop](https://www.docker.com/products/docker-desktop/)
1. Follow the [Dev Containers Tutorial](https://code.visualstudio.com/docs/devcontainers/tutorial)

Now checkout the code in this repository:

1. Check out this repository into your home directory. 

    **These commands work for Terminal on Mac and PowerShell on Windows.**
    ```console
    cd 
    git clone git@github.com:CabrilloRoboticsClub/cabrillo_rov_2023.git
    cd ~/cabrillo_rov_2023
    git submodule init 
    git submodule update
    ```
1. Start vscode int the new `cabrillo_rov_2023` directory. Vscode will prompt you to create a dev container. Follow the prompts. Building the container will take 10 to 20 minutes but you only have to do it once. 

1. Create a Terminal in your vscode window with the `Terminal -> New Terminal` menu. 
1. Build the software in your dev container: 

    **Run this command in the dev container terminal.**

    ```console 
    make 
    ```

Alternatively, if you don't want to use the container inside of VSCode, you can run
```console
docker build -t ros2-container:<tag> .devcontainer
docker run --user vscode -v .:/home/vscode/cabrillo_rov_2023 --network host -it ros2-container:<tag>
```

## Ubuntu 22.04 Desktop Setup 

If you have Ubuntu 22.04 on your desktop you don't need a dev container because you can run everything natively. Only Ubuntu machines will be able to use hardware and communicate with other machines in the ROS2 network. 

1. Install necessary packages (skip this if you're using a removable drive provided by the club)
    ```console
    sudo apt update -y && sudo apt install -y git ansible python3-pip vim 
    ```
1. Check out this repository into your home directory. 
    ```console
    cd 
    git clone git@github.com:CabrilloRoboticsClub/cabrillo_rov_2023.git
    cd ~/cabrillo_rov_2023
    git submodule init 
    git submodule update
    ```
1. Use `make` to install ROS2 and all the necessary packages. 
    ```console 
    make devbox-install
    ```
    This will take a while! 
1. Put ROS on your path 
    ```console
    echo 'if [ -z "$ROS_DISTRO" ]; then source /opt/ros/humble/setup.bash; fi' >> ~/.bashrc 
    ```
1. **Start a new shell** and build the repository
    ```console
    make 
    ```

You Desktop is now a dev box. 

## Setup a Raspberry Pi 4

If you want to setup a Pi to work like the ROV follow the instructions in the [setup directory](./setup/README.md)

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
ros2 launch seahawk rov.launch.py  
```

To launch all the nodes on the deck:

```console 
ros2 launch seahawk deck.launch.py  
```

## Running Individual Nodes

The built nodes are a part of a local build area. Source the following script to setup your paths:

```console
source install/setup.bash # Choose one that matches your shell.
```

Run the Python examples:

```console
ros2 run seahawk example_pub
ros2 run seahawk example_sub
```

## Launching the Kinematics Debug Nodes

Do this to launch the debug nodes. It also starts RViz:

```console 
$ ros2 launch seahawk_deck kinematics_viz.launch.py
```

You have to separately launch the other deck nodes. 
