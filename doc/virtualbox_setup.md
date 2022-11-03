# Development VM Setup

I've decided to use Oracle VirtualBox for the VM tooling because it is relatively easy to use, and exists on all major platforms. This is *not* a general purpose guide for using VirtualBox, or a guide for getting started with Linux. Ideally, this will be a lits of procedures that can be understood by someone with zero experience with either so that they can begin working on the robot code as fast as possible. If a wider understanding is needed, this is the wrong document.

### Preparing for the VM installation

You will need to have the following:

- Oracle VirtualBox [main page here](https://www.virtualbox.org/)
- Ubuntu 22.04 [main page here](https://ubuntu.com/)
    - Either an x86_64 or ARM64 .iso will work. If you're not sure what this means, you're probably looking for the x86_64 (sometimes called `AMD64` and not to be confused with `ARM64`).
 - cabrillo_rov_2023 repository [github here](https://github.com/CabrilloRoboticsClub/cabrillo_rov_2023)

ROS2 Humble depends on Ubuntu 22.04, and so we'll be using that. Since this is a guide for setting up the Cabrillo ROV dev VM, we're of course using the setup scripts from the Cabrillo ROV (2023) code repository.

### Installing the Virtual Machine

With all the parts, we can begin installing the virtual machine. Start up VirtualBox, and you'll find a screen with a handful of buttons on it. The one of interest is the "new" button towards the center of the window. Pressing on this you'll be asked for a bunch of information about the guest virtual machine.

- Name
- Folder
- ISO Image
- Edition
- Type
- Version
- Skip Unattended Installation

The name can be anything you like. It's only for VirtualBox to have something to call this machine and is separate from the "hostname" of the guest.
The folder is the location in which to store the guest's virtual harddrive file.

> A note about the `Folder` line: Make sure this file is put somewhere you can access it. Flash drives are fine if you want to take it with you, just remember to power down the VM before trying to unplug it. Best case: you lose a bit of logs or something, and the system recovers on next boot. But it *could* cause filesystem corruption, and you'll have to blow it all away and stat over.  Also consider the read/write speed (it can be slow!)

Next is the "ISO Image." For us, this will be the Ubuntu installation image downloaded from the Ubuntu website.

At this point, VirtualBox grays out the "Edition", "Type", and "Version" fields (filling in the last two for us), and gives us a notice that the "Detected OS type: Ubuntu (64-bit)... ...can be installed unattendedly." VirtualBox is smart and knows that it can take advantage of some tooling in the installation image we have. This process, however, requires files that may or may not exist in your install of VirtualBox (mine, an Arch Linux host, has chosen to separate them into an optional package. I can't possibly account for all hosts, so instead I'll just do the normal "attended" installation process. If you've ever installed Ubuntu before, this will be the same process as that. More familiar this way!

Check the "Skip Unatteded Installation" box, and continue forward!

This next screen is our hardware configuration. We get to pick the amount of RAM, and number of CPU's we have. I recommend 4GB of RAM, and 2 CPU's as a minimum. If you need, you can get away with less, and if you can afford it you are of course free to give it more.

> `Whole CPU's?` : A "CPU" is usually called a "core" since the whole assembly is called the "CPU." 

There's one more checkbox here to enable the EFI mode. Ubuntu 22.04 supports (U)EFI mode, so you can check this box. Mark it, and move forward!

> `What's EFI?` : (U)EFI is the boot mechanism that replaced BIOS, and has been around since 2005. VirtualBox says "special OS's only" but it's actually the case that *most* systems can boot in this mode.

Finally we get to set up our storage file. VirtualBox defaults to a size of 25GB. This should be plenty if you're concerned about using too much of your host's storage. As always, you can increase if you want. I recommend __against__ higher than 64GB, though. Ubuntu doesn't need nearly that much space, and our code lives on a Raspberry Pi. If anything gets to be that big, it'll never work on the ROV anyway. These virtual disk files are flexible in their size, so if you make a 25GB file and see that it's only a few megabytes, don't panic! They don't need to be one giant file that's 25GB in size where most of it is blank. Instead they're "thin allocated" and only have as much data as they actually contain.

Finally we are given a summary screen. If we're happy with all the settings, press "Finish" and we can move on to the Ubuntu installation itself!

### Installing Ubuntu

> Installing Ubuntu? But we just installed the VM? WHy do we need to install it again?

We have given VirtualBox a VM configuration, and a disk image to boot as the OS installer. Now we need to power on the VM and do the OS installation. Power up the VM by double clicking it. This will open a new window that shows the GRUB boot menu for Ubuntu. We want to "Try or Install Ubuntu". This is the default option, and will start automatically after a few seconds. At this point, you can do the stanard Ubuntu installation process. There's nothing special that we need to do during this part. If needed, you can find a guide online.

### Installing ROS2

With a working Ubuntu 22.04 install, you can get all the tooling we'll be using for our ROV. The big one is ROS2. Unfortunately, the install process is a bit involved. Fortunately, it's already automated as part of our setup scripts!

You'll need to install the following from `apt`:

- 
