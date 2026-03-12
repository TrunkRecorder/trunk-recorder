---
sidebar_label: 'Linux Install'
sidebar_position: 1
---

# Linux Installs

This page covers installing Trunk Recorder on a Linux box. I test everything on Ubuntu, but other flavors of Linux are supported. Instructions are also included for Arch Linux.

## Install Prerequisites
To get started, install all of the required packages. Instructions for different versions are below:


### Ubuntu 23.04

```bash
sudo   apt-get install -y apt-transport-https build-essential ca-certificates fdkaac git gnupg gnuradio gnuradio-dev gr-osmosdr libuhd-dev libboost-all-dev libcurl4-openssl-dev libgmp-dev libhackrf-dev liborc-0.4-dev libpthread-stubs0-dev libssl-dev libuhd-dev libusb-dev pkg-config software-properties-common cmake libsndfile1-dev sox
```

If you are using a HackRF:

```bash
sudo apt install -y hackrf libhackrf-dev libhackrf0
```

If you are using an IIO device (Pluto, LibreSDR, fmcomms2/3/4, etc):
```bash
sudo apt install -y libad9361-dev libiio-dev libiio-utils libgnuradio-iio*
```

### Ubuntu 22.04

```bash
sudo   apt-get install -y apt-transport-https build-essential ca-certificates fdkaac git gnupg gnuradio gnuradio-dev gr-osmosdr libuhd4.1.0 libuhd-dev libboost-all-dev libcurl4-openssl-dev libgmp-dev libhackrf-dev liborc-0.4-dev libpthread-stubs0-dev libssl-dev libuhd-dev libusb-dev pkg-config software-properties-common cmake libsndfile1-dev sox
```

If you are using a HackRF:

```bash
sudo apt install -y hackrf libhackrf-dev libhackrf0
```

If you are using an IIO device (Pluto, LibreSDR, fmcomms2/3/4, etc):
```bash
sudo apt install -y libad9361-dev libiio-dev libiio-utils libgnuradio-iio*
```


### Ubuntu 21.04

```bash
sudo   apt-get install -y apt-transport-https build-essential ca-certificates fdkaac git gnupg gnuradio gnuradio-dev gr-osmosdr libuhd3.15.0 libuhd-dev libboost-all-dev libcurl4-openssl-dev libgmp-dev libhackrf-dev liborc-0.4-dev libpthread-stubs0-dev libssl-dev libuhd-dev libusb-dev pkg-config software-properties-common cmake libsndfile1-dev sox
```

If you are using a HackRF:

```bash
sudo apt install -y hackrf libhackrf-dev libhackrf0
```

If you are using an IIO device (Pluto, LibreSDR, fmcomms2/3/4, etc):
```bash
sudo apt install -y libad9361-dev libiio-dev libiio-utils libgnuradio-iio*
```

### Ubuntu 20.x

```bash
sudo apt install -y gr-osmosdr osmo-sdr libosmosdr0 libosmosdr-dev libuhd3.15.0 libuhd-dev gnuradio-dev libgnuradio-uhd3.8.1 libgnuradio-osmosdr0.2.0 gcc cpp cmake make build-essential libboost-all-dev  libusb-dev fdkaac sox openssl libssl-dev curl libcurl4 libcurl4-openssl-dev pkg-config liborc-0.4-dev git
```

If you are using a HackRF:

```bash
sudo apt install -y hackrf libhackrf-dev libhackrf0
```

If you are using an IIO device (Pluto, LibreSDR, fmcomms2/3/4, etc):
```bash
sudo apt install -y libad9361-dev libiio-dev libiio-utils libgnuradio-iio*
```

### Older Ubuntu Versions...

These instructions should work on Ubuntu 16.x to 17.x, including Debian 9 and 10. 

- For **Ubuntu 18.04** add bionic universe before updating the available packages list

```bash
sudo add-apt-repository "deb http://archive.ubuntu.com/ubuntu bionic universe"
sudo apt-get update
```

Ubuntu 
```bash
sudo apt install gnuradio gnuradio-dev libuhd-dev libgnuradio-uhd3.7.11
```
Debian 
```bash
sudo apt install gnuradio gnuradio-dev libuhd-dev libgnuradio-uhd3.7.13
```

For all SDR based devices...
```bash
sudo apt install gr-osmosdr libosmosdr0
```

If using HackRF or one of its derivatives...
```bash
sudo apt install hackrf libhackrf0 libhackrf-dev
```

```bash
sudo apt install git cmake make build-essential libboost-all-dev libusb-1.0-0.dev libaacs0 libcppunit-dev libcppunit-1.14-0 libssl-dev openssl curl fdkaac sox libcurl3-gnutls libcurl4 libcurl4-openssl-dev
```


### Arch Linux

#### Install from AUR
Update and dependencies
```bash
sudo pacman -Suy
sudo pacman -S --needed base-devel git
```

Clone the package
```bash
git clone https://aur.archlinux.org/trunk-recorder.git
```

Install trunk-recorder
```bash
cd trunk-recorder
makepkg -si
```

Configure `/etc/trunk-recorder/config.json` and start with `systemctl start trunk-recorder.service`. The service runs as trunk-recorder so ensure needed devices(SDR) and files(Media/Talkgroups) are usable(Read or Read/Write) by the trunk-recorder user. 

#### Install with AUR helper
```bash
yay -S trunk-recorder
```
Configure `/etc/trunk-recorder/config.json` and start with `systemctl start trunk-recorder.service`. The service runs as trunk-recorder so ensure needed devices(SDR) and files(Media/Talkgroups) are usable(Read or Read/Write) by the trunk-recorder user. 

#### Install from source

It is suggested to make sure your installed packages are up to date and to review the [Arch Linux documentation regarding upgrades](https://wiki.archlinux.org/index.php/System_maintenance#Upgrading_the_system):
```bash
sudo pacman -Syyu
```

Install the packages required to build Trunk Recorder:
```bash
sudo pacman -S --needed base-devel git cmake boost gnuradio gnuradio-osmosdr libuhd fdkaac sox
```

Continue with Building Trunk Recorder below

## Building Trunk Recorder

In order to keep your copy of the Trunk Recorder source code free of build artifacts created by the build process, it is suggested to create a separate "out-of-tree" build directory. We will use `trunk-build` as our build directory.

Assuming you are in the desired directory to place both trunk-recorder and trunk-build folders to, perform the following...

```bash
mkdir trunk-build
git clone https://github.com/TrunkRecorder/trunk-recorder.git
cd trunk-build
cmake ../trunk-recorder
make
sudo make install
```

## Configuring the UHD for Ettus SDRs

If you haven't setup UHD yet there are a few extra steps you need to take:

Install the UHD drivers:

```bash
sudo apt-get install libuhd-dev uhd-host
```

Download the firmware images. The location of the downloader is different than the error message:

```bash
dpkg -L uhd-host | grep "downloader"
```
The run the downloader at the location identified.

Setup the udev rules so any user can access the USB, as documented [here](https://files.ettus.com/manual/page_transport.html#transport_usb_udev):

```bash
cd /lib/uhd/utils
sudo cp uhd-usrp.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## Configure the RTL-SDR drivers

RTL-SDR dongles were originally meant to be used for DTV reception. There are default drivers that sometimes get loaded up for this and need to be blocked so you can use the SDR functionality. 

Edit the blocklist:
```bash
sudo nano /etc/modprobe.d/blocklist-rtlsdr.conf
```

and add in the following:

```bash
# Blacklist host from loading modules for RTL-SDRs to ensure they
# are left available for the Docker guest.
blacklist dvb_core
blacklist dvb_usb_rtl2832u
blacklist dvb_usb_rtl28xxu
blacklist dvb_usb_v2
blacklist r820t
blacklist rtl2830
blacklist rtl2832
blacklist rtl2832_sdr
blacklist rtl2838
blacklist rtl8192cu
blacklist rtl8xxxu
```

Now restart to make sure these drivers are not loaded.


## Configuring Trunk Recorder

The next step is to [configure Trunk Recorder](../CONFIGURE.md) for the system you are trying to capture.

## Running trunk recorder. 

If all goes well you should now have the executable named `trunk-recorder`, and created the `config.json` configuration file as described in the [Wiki](https://github.com/TrunkRecorder/trunk-recorder/wiki/Configuring-a-System) and [README](https://github.com/TrunkRecorder/trunk-recorder/blob/master/README.md#configure).

From your build directory (e.g. `trunk-build`) you can now run
`./trunk-recorder`

### Runtime options

Trunk Recorder will look for a *config.json* file in the same directory as it is being run in. You can point it to a different config file by using the *--config* argument on the command line, for example: `./trunk-recorder --config=examples/config-wmata-rtl.json`.
