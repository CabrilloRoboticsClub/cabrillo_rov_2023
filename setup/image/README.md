# Ubuntu Development Drive Preparation 

This is the construction and customization of the developer disks.

## Download the 2024 Image 

An [Ubuntu image](https://drive.google.com/file/d/1PHmEwpXliKuy4DiJp5DwF8t5_xciZzM5/view?usp=sharing) is available for download. It was built using the procedure in the next part. 

## Create an Ubuntu Image 

This procedure is how the downloadable image was generated. The process assumes you're working on Ubuntu 22.04 Desktop with `virt-manager` installed. 

1. Download the installer ISO. 

    ```console
    $ wget https://releases.ubuntu.com/26.04.1/ubuntu-26.04.1-desktop-amd64.iso
    ```

1. Create a VM using the installer. 

    ```console
    $ virt-install --name=devbox \
    --boot uefi \
    --vcpus=4 \
    --memory=8192 \
    --cdrom=./ubuntu-26.04.1-desktop-amd64.iso \
    --disk ./devbox.qcow2,size=16,sparse,format=qcow2 \
    --osinfo detect=on,name=ubuntu24.04
    --qemu-commandline="-machine q35,smm=on -global driver=cfi.pflash01,property=secure,value=on -device uefi-vars-x64,jsonfile=vars.json"
    ```

    **Note: The `osinfo` is advisory. Setting it to 24.04 means you can do 
    this procedure on an earlier version of Ubuntu** 
    
1. Bootstrap the image with the following settings:
    1. Interactive Installation
    1. Default Selection
    1. Enable third party drivers
    1. Enable multimedia
    1. User: `robot`
    1. Password: *Funny Cabrillo* 
    
1. On first boot:
    1. Setup the user's desktop 
    1. Install packages:
        ```console
        $ sudo apt update -y && sudo apt install -y git ansible python3-pip vim cloud-init
        ```
    1. Install vscode 

1. Configure `cloud-init` to expand on subsequent boot
    1. Make the directory `/boot/efi/cloud-init`
    1. Paste this into `/boot/efi/cloud-init/user-data`
        ```yaml
        #cloud-config
        growpart:
            mode: auto
            devices: ["/"]
            ignore_growroot_disabled: false
        ```
    1. Paste this into `/boot/efi/cloud-init/meta-data`
        ```yaml
        #cloud-config
        ```
    1. Configure GRUB:
        1. Update `/etc/default/grub`:
            ```
            GRUB_CMDLINE_LINUX="ds=nocloud\;s=file:///boot/efi/cloud-init/"
            ```
        1. Run:
            ```console
            $ sudo update-grub 
            ```
1. Make sure cloud-init will re-run:
    ```console 
    $ sudo cloud-init clean --logs
    ```

## Burning the QCOW2 Image 

Make sure the device is unmounted after you plug it in: 

```console
$ sudo umount /dev/sdX 
```

The `qcow2` image format is sparse. Burning it onto a flash drive can't be done
with `dd`. You have to use `qemu-img` to convert it as it copies. 

```console 
sudo qemu-img convert -f qcow2 -O raw devbox.qcow2 /dev/sdX
```

You can use QEMU to validate the image on disk without booting it. 

```conosle 
$ sudo qemu-system-x86_64 \
    -bios /usr/share/ovmf/OVMF.fd -cpu host -accel accel=kvm \
    -drive file=/dev/sdX,format=raw \
    -m 8192 -smp 4 \
    -boot c
```

The image should automatically expand to fit the whole disk. The DOS partition
on the imaged disk exposes the `cloud-init` files. A user could customize them
before first boot if they wanted to. 
