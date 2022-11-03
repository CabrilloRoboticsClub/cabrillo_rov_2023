# Development VM Setup

I've decided to use Oracle VirtualBox for the VM tooling because it is relatively easy to use, and exists on all major platforms. This is *not* a general purpose guide for using VirtualBox, or a guide for getting started with Linux. Ideally, this will be a lits of procedures that can be understood by someone with zero experience with either so that they can begin working on the robot code as fast as possible. **If a wider understanding is needed, this is the wrong document.**

## Preparing for the VM installation

You will need to have the following:

- Oracle VirtualBox [main page here](https://www.virtualbox.org/)
- Ubuntu 22.04 LTS installer .iso [main page here](https://ubuntu.com/)
    - Most likely, you're looking for an x86_64 image. Sometimes called AMD64 (don't confuse it with `ARM64`. it's easy to do)

> If you happen to be an an M1/M2 Apple Silicon system, you want the `ARM64` image. In this case, the process *should* be identical, but the EFI suppport may or may not cause issues.
ROS2 Humble depends on Ubuntu 22.04, and so we'll be using that. Since this is a guide for setting up the Cabrillo ROV dev VM, we're of course using the setup scripts from the Cabrillo ROV (2023) code repository.

## Installing the Virtual Machine

Hit "new" and we're greeted with the configuration panel with the following properties:

1. Name
    * The name can be anything you like. It's only for VirtualBox to have something to call this machine and is separate from the "hostname" of the guest.
2. Folder
    * The folder is the location in which to store the guest's virtual harddrive file.
3. ISO Image
    * Next is the "ISO Image." For us, this will be the Ubuntu installation image downloaded from the Ubuntu website.
4. Edition
5. Type
6. Version
7. Skip Unattended Installation

> A note about the `Folder` line: Make sure this file is put somewhere you can access it. Flash drives are fine if you want to take it with you, just remember to power down the VM before trying to unplug it. Best case: you lose a bit of logs or something, and the system recovers on next boot. But it *could* cause filesystem corruption, and you'll have to blow it all away and stat over.  Also consider the read/write speed (it can be slow!)

#### Name, ISO, and no-unattended install

At this point, VirtualBox grays out the "Edition", "Type", and "Version" fields (filling in the last two for us), and gives us a notice that the "Detected OS type: Ubuntu (64-bit)... ...can be installed unattendedly." VirtualBox is smart and knows that it can take advantage of some tooling in the installation image we have. This process, however, requires files that may or may not exist in your install of VirtualBox (mine, an Arch Linux host, has chosen to separate them into an optional package.) For this reason __select the "Skip Unattended Installation"__ box. 

> I can't possibly account for all hosts, so instead we'll just do the normal "attended" installation process. If you've ever installed Ubuntu before, this will be the same process as that. More familiar this way!

#### RAM and CPU configuration (and EFI mode)

This next screen is our hardware configuration. We get to pick the amount of RAM, and number of CPU's we have, as well as the boot mode of our virtual machine. __I recommend 4GB of RAM, and 2 CPU's and to enable EFI.__ If you need, you can get away with less, and if you can afford it you are of course free to give it more.

> `Whole CPU's?` : A "CPU" is usually called a "core" since the whole assembly is called the "CPU." 

> `What's EFI?` : (U)EFI is the boot mechanism that replaced BIOS, and has been around since 2005. VirtualBox says "special OS's only" but it's actually the case that *most* systems can boot in this mode.

#### Storage for the VM

Finally we get to set up our storage file. VirtualBox __defaults to a size of 25GB, and this should be plenty.__ If you decide on a larger I recommend *against* higher than 64GB, though. Ubuntu doesn't need nearly that much space, and our code lives on a Raspberry Pi. If anything gets to be that big, it'll never work on the ROV anyway.
> `VDI file size` : These virtual disk files are flexible in their size, so if you make a 25GB file and see that it's only a few megabytes, don't panic! They don't need to be one giant file that's 25GB in size where most of it is blank. Instead they're "thin allocated" and only take as much data as they actually contain.

Finally we are given a summary screen. If we're happy with all the settings, press "Finish" and we can move on to the Ubuntu installation itself!

### Installing Ubuntu

> Installing Ubuntu? But we just installed the VM? WHy do we need to install it again?

We have given VirtualBox a VM configuration, and a disk image to boot as the OS installer. Now we need to power on the VM and do the OS installation. Power up the VM by double clicking it. This will open a new window that shows the GRUB boot menu for Ubuntu. We want to "Try or Install Ubuntu". This is the default option, and will start automatically after a few seconds. At this point, you can do the stanard Ubuntu installation process. There's nothing special that we need to do during this part. If needed, you can find a guide online.

### Installing ROS2

With a working Ubuntu 22.04 install, you can get all the tooling we'll be using for our ROV. The big one is ROS2. Unfortunately, the install process is a bit involved. Fortunately, it's already automated as part of our setup scripts!

You'll need to install the following from `apt`:

- git
- python3
- python3-pip
