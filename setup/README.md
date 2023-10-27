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
    Full instructions for how to add a public key to GitHub can be found in [GitHub's official documentation](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account#adding-a-new-ssh-key-to-your-account).
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

## Using vscode to Connect to a Pi

The best way to access the Raspberry Pi is using vscode. You can use the access to develop and test nodes that run on the Pi or to just start the ROV during development of deck-side nodes. Assuming you have your devbox setup per the instructions above, perform the following steps to access the Pi. 

1. Install the [Remote SSH plugin](https://code.visualstudio.com/docs/remote/remote-overview) into vscode
1. Click the remote status bar on the very bottom left of the window. (You can find some tips in the [Remote SSH tutorial](https://code.visualstudio.com/docs/remote/ssh-tutorial).)

    ![](https://code.visualstudio.com/assets/docs/remote/ssh-tutorial/remote-status-bar.png)

    The Remote Status bar item can quickly show you in which context VS Code is running (local or remote) and clicking on the item will bring up the Remote - SSH commands.

    ![](https://code.visualstudio.com/assets/docs/remote/ssh-tutorial/remote-ssh-commands.png)
1. Type the ssh command into the top bar: 

    ![](https://code.visualstudio.com/assets/docs/remote/ssh-tutorial/set-user-host.png)

    > The command will look like: **ssh yourname@seachicken.local -A** 

1. Follow the prompts:
    1. Choose the first option for where to save the configuration. 
    1. Choose "Linux" type if asked. 
    1. Choose "Yes" if this is your first time connecting to this Pi.

1. You should see that you are connected:

    ![](https://code.visualstudio.com/assets/docs/remote/ssh-tutorial/ssh-status-bar.png)

1. Now that you're connected, start a new Terminal. 
1. Create a copy of the repo in your home directory: 

    ```console 
    git clone git@github.com:CabrilloRoboticsClub/cabrillo_rov_2023.git
    ```

1. Configure git:

    ```console 
    git config --global user.name "Your Name"
    git config --global user.email "you@you.com"
    ```

1. Add ROS to your startup scripts:

    ```console 
    echo 'if [ -z "$ROS_DISTRO" ]; then source /opt/ros/humble/setup.bash; fi' >> ~/.bashrc 
    ```

1. Exit your current terminal and start a new one. 
1. Build the ROS project

    ```console 
    cd ~/cabrillo_rov_2023
    make
    ```

Vscode will remember everything for next time. You should only have to do this once. 

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
    git clone https://github.com/CabrilloRoboticsClub/cabrillo_rov_2023.git
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
    echo 'if [ -z "$ROS_DISTRO" ]; then source /opt/ros/humble/setup.bash; fi' >> ~/.bashrc 
    ```
1. **Start a new shell** and build the repository
    ```console
    make 
    ```
