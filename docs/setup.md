# Setup
The following file contains directions for setting up a TCU or ROV. This guide can be used for setting up production systems, or test hardware.

While the following directions are thorough, they do not detail every step of the process. Processes like flashing USB drives are subsumed into one step. If you have questions, reach out to a senior software team member.

## Test Setups
The ideal test setup is as follows:

- Ubuntu 24.04.1 LTS Desktop running natively on a laptop
- Ubuntu 24.04.1 LTS Server running on a testbench.

If this is unattainable, there are several other ways to simulate topside.

* Ubuntu 24.04.1 LTS in WSL on Windows

* Ubuntu 24.04.1 LTS in a VMWare Virtual Machine on Windows

* Ubuntu 24.04.1 LTS in a UTM Virtual Machine on MacOS

Booting (or dual-booting) Ubuntu natively is always preferred, because network interfaces can be difficult in virtual machines and WSL. For anyone attempting to use WSL, VMWare, or any other Windows alternative, you can merge networks with the Windows tool Hyper-V. 

UTM seems to be a seamless virtualization experience on MacOS, though. VMWare is relatively untested for the purposes of TCU virtualization, and WSL users will need to keep in mind that all of their text editing must be done through a terminal text editor like [NeoVim](https://neovim.io/) or with the [WSL Extension for VSCode](https://code.visualstudio.com/docs/remote/wsl).

## Setting up a TCU
### Part 1 (Production Hardware)
* Flash a USB drive with [Ubuntu 24.04.1 LTS Desktop](https://ubuntu.com/download/desktop/thank-you?version=22.04.3&architecture=amd64) using [Balena Etcher](https://etcher.balena.io/).
* Plug the USB drive into the laptop.
* Access BIOS (Press F2 during power-on)
  * Disable Secure Boot.
  * If necessary, edit the boot order to prioritize the thumb-drive over Windows.
  * Save your changes and exit.

### Part 1 (Dual-Booting)
* [Shrink a disk partition](https://learn.microsoft.com/en-us/windows-server/storage/disk-management/shrink-a-basic-volume) by 25-35 GB.
* If you have bitlocker encryption, disable it.
* Flash a USB drive with [Ubuntu 24.04.1 LTS Desktop](https://ubuntu.com/download/desktop/thank-you?version=22.04.3&architecture=amd64) using [Balena Etcher](https://etcher.balena.io/).
* Plug the USB drive into the laptop.
* Access your laptop BIOS (Button varies by manufacturer)
  * (OPTIONAL) Disable Secure Boot.
    * Try to complete these steps without disabling secure boot first.
    * If that fails, disable secure boot and then re-enable it after the installation process.
  * If necessary, edit the boot order to prioritize the thumb-drive over Windows.
  * Save your changes and exit.

### Part 1 (WSL)
* [Install WSL](https://www.microsoft.com/p/ubuntu/9pdxgncfsczv) from the Microsoft Store.
* Launch WSL.

### Part 1 (Virtualization)
* Download an [Ubuntu 24.04.1 LTS Desktop](https://ubuntu.com/download/desktop/thank-you?version=22.04.3&architecture=amd64) ISO.
* Create a virtual machine with the ISO.
  * Steps will vary based on virtualization software. Use of VirtualBox is not recommended.
* Boot the virtual machine and begin the installation process. 

### Part 2
* Proceed with the installation process as normal.
  * Customize settings to your liking, with the exception of those specified in the following directions.
* If you are dual booting, when prompted, choose to install Ubuntu alongside Windows.
  * Create a Linux partition on the 35GB you freed up by shrinking a disk partition.
* Opt for a minimal installation, without installing third-party software.
* When you reach the "Who are you?" screen:
  * Your name: JHSRobo
  * Your computer's name: topside
  * Pick a username: jhsrobo
  * Choose a password: JHSRobo

### Part 3
* Install git.
  * `sudo apt install git`
* Clone the corews repository in your home directory.
  * `git clone https://github.com/JHSRobo/corews`
* Run the setup command.
  * `sudo bash ~/corews/scripts/tcu/tcu_setup.bash`
* Source .bashrc.
  * `source ~/.bashrc`

## Setting up an ROV
* Download the [Raspberry Pi Imager](https://www.raspberrypi.com/software/).
* Flash a micro-sd card reader with Ubuntu Server 24.04.1 LTS (64-bit).
  * Listed under the "Other general-purpose OS" category.
  * Use the following settings (all others can remain default):
    * Enable SSH with password authentication.
    * Username: jhsrobo
    * Password: JHSRobo
    * IMPORTANT: Keep "Configure wireless LAN" disabled.
* Slot the micro-sd card into a testbench.
* SSH to the testbench.
  * You will need to check its IP on the router page.
* Set the new hostname to bottomside.
  * `sudo hostnamectl set-hostname bottomside` 
* Clone the corews repository in your home directory.
  * `git clone https://github.com/JHSRobo/corews`
* Run the setup command.
  * `sudo bash ~/corews/scripts/rov/rov_setup.bash`
* Source .bashrc.
  * `source ~/.bashrc`
---
Written by randallj24@student.jhs.net.
Updated by fringsj26@student.jhs.net

If we're in college by the time you're reading this, feel free to reach out.

Last Updated 12/02/24
