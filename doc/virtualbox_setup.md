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

At this point, VirtualBox grays out the "Edition", "Type", and "Version" fields (filling in the last two for us), and gives us a notice that the "Detected OS type: Ubuntu (64-bit)... ...can be installed unattendedly." VirtualBox is smart and knows that it can take advantage of some tooling in the installation image we have. This means the next screen will look a little different than it would otherwise. If you want to see that process, you can check the "Skip Unattended Installation" box, and continue. I'll be using the unattended install process for this guide, however, and I recommend that you do the same.

The next screen has 3 boxes

- Username and Password
- Additional Options
- Guest Additions

Pick out a username and password. This is a personal development tool, so you don't need to consider sharing either with another person. The `Additional Options` pane contains a greyed-out product key (for OS's that have those. But we do not), a hostname, and a domain name. The hostname can be whatever you like, and the domain should probably be blank. If you have a domain, and you'd like to have this machine claim to be a member, then by all means. Most people do not, and it's bad form to claim to be from another domain. This pane also has the "Install in background" checkbox, but we're going to ignore it.
