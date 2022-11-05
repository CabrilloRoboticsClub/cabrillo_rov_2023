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

## Installing Ubuntu

#### The major beats

1. Welcome
    - Install Ubuntu (not try)
2. Keyboard layout
    - English US (unless, ofc, you're not English US)
3. ???
4. Updates and other software
    - Select Minimal Installation (We don't really want or need *Solitaire* and the like)
    - Download updates
        - You *could* postpone this, but you'll do it eventually anyway. May as well do it here.
5. Installation Type
    - Erase disk and install Ubuntu
    - Hit "install now" and It'll ask if you're sure. Yes. Yes you are.
6. Where are you?
    - Pick your timezone (Cabrillo is Los Angeles)
7. Who are you?
    - Name: Your full name (or nothing, if you really want)
    - Your Computer's Name: Name your computer! Only dashes as special characters, though.
    - Username: The username for your account.
    - Password: I highly recommend something short.
        - this is a virtual guest that already lives behind your normal password. A weak password here is *probably fine.* ~~This is not legal advice. I will not be held accountable for your actions.~~
    - Automatic Login: Whichever you like
        - auto/manual login from the graphical login panel. This doesn't let people remote into your VM unauthenticated.
    - ActiveDirectory server: No. We don't have one of those.
8. Final:
    - Beware the Software Updater popup
        - It will show up shortly after the system hits the desktop.  When it does, click "Remind me later.' We'll handle that in a bit.
    - Online Accounts: You can sign in to any and all accounts you want to. Including, of course, none at all.
    - Livepatch: Fancy feature that we don't need, and wont pay for. Just press "next."
    - Help Improve Ubuntu: No, don't send system info
    - Welcome to Ubuntu -> Location services: No
    - Ready to go: Done and done!

Congratulations! Ubuntu 22.04 LTS is now installed into your virtual machine

## Installing ROS2

With a working Ubuntu 22.04 install, you can get all the tooling we'll be using for our ROV. The big one is ROS2. Unfortunately, the install process is a bit involved. Fortunately, it's already automated as part of our setup scripts!

### Using apt for package installation

Before we continue, a note on package installation: On Ubuntu one can use the `apt` tool to manage software packages. The important commands you need to know are as follows

- `apt update`
- `apt upgrade`
- `apt install [packages...]`

where the `[packages...]` is a space-separated list of packages you want installed.

e.g.:
```
apt install git python3
```
instructs `apt` to install `git` and `python3`

**Ensure your package lists are up to date** by running `apt update`. This will fetch the lists of packages available from the Ubuntu repositories so you can see if there are updates available (there probably are! that's what the Software Updater popup was asking about earlier)

> A side effect of using an outdated package list is that issuing install commands collects an old(er) copy of the package. This isn't likely to be a problem, but it does mean you'll end up waiting on a re-download and re-upgrade later on when the setup script does a full upgrade.

### Installing the setup utilities

Open your terminal (`ctrl+t` or) "Show Applications" in the bottom left, and then select "Terminal." From here you can use the `apt` tool to install the following:

- git
- python3
- ansible

### Configuring Git

With the `git` tool installed, you can now clone the repository. However, you wont have any ability to push your changes back up to GitHub without some additional configuration.

#### Name and Email

To commit changes, you need to tell git a name and email to attach. Run the following:

`git config --global user.name "Your Name"`
`git config --global user.email "your.email@host.com"`

Of course, replace the fake name and email with your own. **You should try to make these match the info on your GitHub account.*

> There isn't anything that has a hard requirement on making your git config match your GitHub. It would just look a bit strange if you had commits with one email attached, pushed them up to GitHub, and then merged the branch(es) using a persona with a different email -- Or name, or other metadata. You *should* try to match them, but you're not *required* to do so.

#### SSH Key (optional, recommended)

GitHub lets you upload an SSH key file for authentication when pushing changes. Doing this means you don't need to enter a username and password every time you want to push your changes to the remote repository (to GitHub). For the VM, we'll be generating a new key, and uploading the **public key** to GitHub. **Do not upload the private key.**

1. Generate key pair: `ssh-keygen`
    * Choose save location (default of $HOME/.ssh/id_rsa) is fine.
    * Enter passphrase (I recommend none)
2. Upload public key to GitHub
    * Navigate to your $HOME/.ssh/ folder, and locate the `id_rsa.pub` file.
    * Go to GitHub settings page
    * SSH and GPG keys
    * New SSH key
    * Add key:
        1. Title: Name your key ("ROS VM key" or something)
        2. Key Type: Authentication Key (this is for pushing changes, not signing them)
        3. Key: Paste the contents of your `$HOME/.ssh/id_rsa.pub` file here. **Make sure it's the one with a .pub suffix for the public key**
        4. Press "Add SSH key"


### Cloning the Cabrillo ROV 2023 code repository

Still from within your terminal, grab a clone of the club code repository:

`git clone git@github.com:CabrilloRoboticsClub/cabrillo_rov_2023.git`

This will attempt to reach the repo over ssh, so you'll be prompted to check a host fingerprint. Accept the fingerprint, and it'll continue connecting. **If the connection fails** it's because you don't have an SSH key that GitHub will accept. If you've never set up a key, then the problem is quite obvious: There is no key! If you *did* then you will need to make sure you have that key on your VM (or to set up another just for the VM). If you don't want to do this, you can use option two:

`git clone https://github.com/CabrilloRoboticsClub/cabrillo_rov_2023.git`

...use the HTTPS protocol! (notice the `https://` out front)

This will clone the git repository into a folder named "cabrillo_rov_2023" in the current directory. By default, this will be your home directory. You are free to place this git repo folder wherever you like. It isn't important that it lives in any particular place

> I like to have mine in a `$HOME/projects` folder. It's not a document so "$HOME/Documents" doesn't quite make sense, so I make a new thing. You can do this, or leave it just there in `$HOME` where you were when cloning it. Folder organization is definitely outside the scope of this document, though, so this will be an exercise for the reader.

### Running Ansible for the remaining setup

Finally, we get to the automated bit! With the repository cloned, and ansible installed, we can use the playbook in our repository to finish setting up the system with all the parts we need for writing code.

`ansible 
