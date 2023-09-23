# Ubuntu Development Drive Preparation 

This is the construction and customization of the developer disks.

## Create an Ubuntu Image 

This procedure assumes you're working on Ubuntu 22.04 Desktop. 

1. Download the installer ISO. 

    ```console
    $ wget https://releases.ubuntu.com/22.04.3/ubuntu-22.04.3-desktop-amd64.iso
    ```

1. Create a VM using the installer. 

    ```console
    $ virt-install --name=devbox \
    --boot uefi \
    --vcpus=4 \
    --memory=8192 \
    --cdrom=./ubuntu-22.04.3-desktop-amd64.iso \
    --disk ./devbox.qcow2,size=16,sparse,format=qcow2 \
    --os-variant=ubuntu22.04 
    ```

1. Bootstrap the image with the following settings:
    1. Minimal image 
    1. Default partitioning 
    1. No extra drivers, no 3-rd party software
    1. User: `student`
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

The `qcow2` image format is sparse. Burning it onto a flash drive can't be done with `dd`. You have to use `qemu-img` to convert it as it copies. 

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

The image should automatically expand to fit the whole disk. The DOS partition on the imaged disk exposes the `cloud-init` files. A user could customize them before first boot if they wanted to. 
