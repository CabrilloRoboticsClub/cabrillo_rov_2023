# Setup

**TODO:** flesh this out with description of the contents of this folder

---

## Raspberry Pi (ROV) Setup procedure

1. On your local computer download and install [Raspberry Pi Imager](https://www.raspberrypi.com/software/)
1. Attach a Micro SD card to your local computer
1. Open Raspberry Pi Imager
    * `CHOOSE OS` > `Other general-purpose OS` > `Ubuntu` > `Ubuntu Server 22.04 LTS (64-bit)`
    * Choose the Micro SD card as the storage and hit `WRITE`
1. Unplug the Micro SD card from your local computer, plug it back in to your local computer.
1. Copy the `user-data` file from `setup/roles/rov/files` into the `system-boot` volume that shows up (overwrite the existing file).
1. Install the SD card into the Raspberry Pi
1. Connect the Pi to an ethernet connection with internet access
1. Plug the power cable in to the Pi.

---

## DECK BOX Setup Procedure

1. On your local machine download [ubuntu-server 22.04 LTS](https://releases.ubuntu.com/22.04.2/ubuntu-22.04.2-live-server-amd64.iso) installer iso.
1. Flash the installer iso to a flash drive.
1. Boot the installer iso on the deck box.
1. install ubuntu server.

### language

* english

### keyboard

Layout:

* English (US)

Variant:

* English (US)

### Choose type of install

Base for the installation:

* Ubuntu Server

Additional Options:

* enable search for third party drivers

### Network Connections

* DHCPv4

### Configure Proxy

* n/a

### Configure Ubuntu Archive Mirror

* use default

### Guided Storage Configuration

* (X) Use Entire Disk
* (X) Set up this disk as an LVM group
* no encryption

### Storage Configuration

just hit done

### Profile Setup

Your name:

* ubuntu

Your Server's Name:

* seahawk-deck

Pick a username:

* ubuntu

Choose a password:

* Cabri11o

### Upgrade to ubuntu pro

* skip for now

### SSH Setup

YES install OpenSSH Server

SSH Identity

* From GitHub

GitHub Username:

* cabrillorobotics

YES allow password authentication over ssh

### Featured Server Snaps

none

---

## Ansible Deploy Playbook (First Time setup)

Use this playbook to deploy (or upgrade) all the devices in the robot to the latest system configuration.

(Run this from a DevBox in the `cabrillo_rov_2023` folder)

```bash
ansible-playbook setup/playbook-deploy.yaml -i setup/hosts.yaml
```

---

## Ansible Build Procedure (Build New Software)

This playbook will build code from the branch of your choosing (on this repository) to all the rov devices

(Run this from a DevBox in the `cabrillo_rov_2023` folder)

```bash
ansible-playbook setup/playbook-build.yaml -i setup/hosts.yaml
```

## Ubuntu Devbox Setup Procedure

1. Install Ubuntu on a fresh computer or a removable drive.
    1. Install a regular system with proprietary software (which may be needed for robot video encode/decode)
1. On first boot install needed packages:

    ```console
    sudo apt install -y git ansible 
    ```

1. Use Ansible to run the `devbox.yaml` file locally.

    ```console
    sudo ansible-pull -U https://github.com/CabrilloRoboticsClub/cabrillo_rov_2023.git --inventory 127.0.0.1, --connection local setup/devbox.yaml
    ```

    > This will take a long time!

1. Generate an SSH key for your development box. Follow the instructions in the [GitHub documentation](https://docs.github.com/en/authentication/connecting-to-github-with-ssh).

1. Clone the Cabrillo ROV repository:

    ```console
    cd 
    git clone git@github.com:CabrilloRoboticsClub/cabrillo_rov_2023.git
    ```

1. Start `vscode`

    ```console
    cd cabrillo_rov_2023
    code . 
    ```

   1. In order to develop on the Pi, install the `Remote SSH` extension.
   1. Install the Python extension

### Keeping up with Project Updates

Your development box has a command `cabrillo-update`. Running the command will update your box with the latest changes from the infrastructure team.

---

`<this guide is incomplete>`
