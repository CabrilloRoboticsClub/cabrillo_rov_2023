# Setup

**TODO:** flesh this out with description of the contents of this folder

## Raspberry Pi Setup procedure

1. On your local computer download and install [Raspberry Pi Imager](https://www.raspberrypi.com/software/) or other imaging software of your choice(`dd`, Balena Etcher, etc.)
1. Attach a Micro SD card to your local computer
1. Open Raspberry Pi Imager
    * `CHOOSE OS` > `Other general-purpose OS` > `Ubuntu` > `Ubuntu Server 22.04 LTS (64-bit)`
    * Choose the Micro SD card as the storage and hit `WRITE`
1. Unplug the Micro SD card, plug it back in, and copy the `user-data` file into the `BOOT` volume that shows up (overwrite the existing file).
1. Install the SD card into the Raspberry Pi, connect the Pi to ethernet, and plug in the power to the Pi.

## Ubuntu Devbox Setup Procedure 

1. Install Ubuntu on a fresh computer or a removable drive. 
    1. Install a regular system with proprietary software (which may be needed for robot video encode/decode)
1. On first boot install needed packages:

    ```console
    $ sudo apt install -y git ansible 
    ``` 

1. Use Ansible to run the `devbox.yaml` file locally.

    ```console
    $ sudo ansible-pull -U https://github.com/CabrilloRoboticsClub/cabrillo_rov_2023.git --inventory 127.0.0.1, --connection local setup/devbox.yaml
    ```

    > This will take a long time!

1. Generate an SSH key for your development box. Follow the instructions in the [GitHub documentation](https://docs.github.com/en/authentication/connecting-to-github-with-ssh).

1. Clone the Cabrillo ROV repository:

    ```console
    $ cd 
    $ git clone git@github.com:CabrilloRoboticsClub/cabrillo_rov_2023.git
    ``` 

1. Start `vscode` 

    ```console
    $ cd cabrillo_rov_2023
    $ code . 
    ``` 

   1. In order to develop on the Pi, install the `Remote SSH` extension. 
   1. Install the Python extension

### Keeping up with Project Updates 

Your development box has a command `cabrillo-update`. Running the command will update your box with the latest changes from the infrastructure team. 

`<this guide is incomplete>`

