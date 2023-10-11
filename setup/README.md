# Setup

This readme will describe how to setup various things.

## Development Box Setup 

Take the following steps to get started. 

1. Install necessary packages (skip this if you're using a removable drive provided by the club)
    ```console
    sudo apt update -y && sudo apt install -y git ansible python3-pip vim 
    ```
1. Create an SSH key if you don't have one already. 
    ```console 
    ssh-keygen -t ed25519
    ```
    Use the default location, set a password if you like. 
1. Copy and paste your public key into GitHub
    ```console
    cat ~/.ssh/id_ed25519.pub
    ```
    Full instructions for how to add a public key to GitHub can be found in [GitHub's official documentation](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account).
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

## Raspberry Pi (ROV) Setup Procedure

Start by creating a bootable SD card image: 

1. On your local computer download and install [Raspberry Pi Imager](https://www.raspberrypi.com/software/)
1. Attach a Micro SD card to your local computer
1. Open Raspberry Pi Imager
    * `CHOOSE OS` > `Other general-purpose OS` > `Ubuntu` > `Ubuntu Server 22.04 LTS (64-bit)`
    * Choose the Micro SD card as the storage and hit `WRITE`
1. Unplug the Micro SD card from your local computer, plug it back in to your local computer.
1. Copy the `user-data` file from `setup/image` into the `system-boot` volume that shows up (overwrite the existing file).
1. Install the SD card into the Raspberry Pi
1. Connect the Pi to an ethernet connection with internet access
1. Plug the power cable in to the Pi.

The Pi will boot, then reboot. Give it 10 minutes or so. The hostname and passwords are now set. SSH is enabled. You can connect with:

```console
ssh -A ubuntu@seahawk.local
```

Once you have SSHed into the new Pi image run the ROV specific playbooks:

1. Check out this repository into your home directory. 
    ```console
    cd 
    git clone git@github.com:CabrilloRoboticsClub/cabrillo_rov_2023.git
    cd ~/cabrillo_rov_2023
    git submodule init 
    git submodule update
    ```
1. Install make and Ansible command
    ```console
    sudo apt install make ansible
    ```
1. Use `make` to install ROS2 and all the necessary packages. 
    ```console 
    make rov-install
    ```
    **This will take a long, long time!** 30 minutes or more. 
1. Reboot the Pi 
    ```console
    sudo reboot
    ```
1. Disable the building of the `seahawk_description` package 
    ```console 
    touch src/seahawk_description/COLCON_IGNORE 
    ```
1. Put ROS on your path 
    ```console
    echo 'if [ -z "$ROS_DISTRO"]; then source /opt/ros/humble/setup.bash; fi' >> ~/.bashrc 
    ```
1. **Start a new shell** and build the repository
    ```console
    make 
    ```
