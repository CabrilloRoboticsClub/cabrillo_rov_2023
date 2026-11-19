# Setup

This directory contains setup infrastructure that's designed to make it easy for
students to get started.

The machines used by the club are configured with [Ansible](https://docs.ansible.com/).
There are two playbooks: one for the student development boxes and one for the
Raspberry Pi that runs on the ROV. Both playbooks share a pair of roles that
install ROS2 and the common packages that Seahawk depends on.

## Running the Playbooks

The playbooks are run against the local machine, so you run them *on* the box
you want to configure. The `makefile` in the repository root wraps the
`ansible-playbook` command so you normally don't call `ansible-playbook`
directly.

The playbooks need to have `make`, `ansible`, and `pipx` installed:

```console
$ sudo apt install make ansible pipx
$ pipx ensurepath
```

Then run the target that matches the machine you're setting up:

| Command | Playbook | Use on |
|---------|----------|--------|
| `make devbox-install` | [`devbox.yaml`](devbox.yaml) | A student development box (desktop/laptop/VM). |
| `make rov-install` | [`rov.yaml`](rov.yaml) | The Raspberry Pi on the ROV. |

Both targets run with `sudo` because the playbooks become `root` to install
packages and edit system files. `make rov-install` additionally creates the
project's `uv` virtual environment and syncs dependencies after the playbook
finishes.

It's safe to re-run after a failure or to pick up new packages.

## Build a Bootable USB Drive

It might be preferable for students to boot from an external drive, rather than
nixing Windows or using a devcontainer. The [`image/`](image) directory
documents how to use QEMU on Linux to creata a bootable image that can be
transfered to a USB drive. 

