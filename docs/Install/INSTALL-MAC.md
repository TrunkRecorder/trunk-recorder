---
sidebar_label: 'Mac Install'
sidebar_position: 4
---
# MacOS
There are two main "package managers" used on MacOS: [Homebrew](#using-homebrew) and [MacPorts](#using-macports). Trunk-recorder can be installed with dependencies from one or the other

## Using Homebrew
Tested on macOS Sequoia 15.5 with the following packages:

- homebrew 4.5.3
- cmake 4.0.2
- gnuradio 3.10.12.0_1
- uhd 4.8.0.0_1
- pkgconfig 2.4.3
- cppunit 1.15.1
- openssl 3.5.0
- fdk-aac-encoder 2.0.3
- sox 14.4.2_6
- pybind11 2.13.6_1
- six 1.17.0

#### Install Homebrew
See [the Brew homepage](https://brew.sh) for more information.
```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install.sh)"
```

#### Installing using the Trunk Recorder Tap

Trunk Recorder can be installed easily by adding the repository to homebrew. More information can be found on [this github repo](https://github.com/TrunkRecorder/homebrew-install). Install can be done easily with the following commands.
```
brew tap trunkrecorder/install
brew install trunk-recorder
```

If you prefer to install packages individually or without adding the repository, it can be done using the instructions below.

#### Install GNURadio and other dependencies
```bash
brew install gnuradio uhd cmake pkgconfig cppunit openssl fdk-aac-encoder sox pybind11 six
```
#### Install the OsmoSDR Package for GNURadio
See the gr-osmosdr [homepage](https://osmocom.org/projects/gr-osmosdr/wiki/GrOsmoSDR) for more information.
```bash
git clone git://git.osmocom.org/gr-osmosdr
cd gr-osmosdr
mkdir build && cd build
cmake ..
make -j
sudo make install
sudo update_dyld_shared_cache
```
Before continuing with the build instructions, note that you will need to specify the location of the Homebrew libssl libraries during `cmake` with `-DOPENSSL_ROOT_DIR=`, or you will receive an error from CMake about not finding libssl, or a linking error from `make` about not having a library for `-lssl`.  

This path in Homebrew will differ by system (Apple Silicon:`/opt/homebrew/opt/openssl@3` or macOS Intel:`/usr/local/opt/openssl@3`), but it can be located automatically as used below:

#### Building Trunk Recorder
```bash
mkdir trunk-recorder && cd trunk-recorder
git clone https://github.com/robotastic/trunk-recorder.git source
mkdir build && cd build
cmake ../source -DOPENSSL_ROOT_DIR=$(brew --prefix openssl@3)
make -j
sudo make install
```
Continue to [Configuring Trunk Recorder](#configuring-trunk-recorder).

## Using MacPorts
### These instructions should work on OS X 10.10, OS X 10.11, and macOS 10.12.

#### Install MacPorts 

Follow [the instructions from the MacPorts project to install the appropriate version of MacPorts](https://www.macports.org/install.php) for your version of macOS.

If you have already installed MacPorts, make sure your ports tree is up to date:
```bash
sudo port selfupdate
```

*(7/24/21) Note: this has been tested and works on an M1 based Mac. Some dependencies for **gr-osmosdr** do not support ARM64 yet and can be removed by adding a -, eg: -docs*

#### Install GNU Radio

The preferred method for [installing GNU Radio](https://wiki.gnuradio.org/index.php?title=MacInstall) on macOS is: 
 
```bash
sudo port install gnuradio uhd gr-osmosdr
```

#### Install tools to compile Trunk Recorder
```bash
sudo port install cmake boost libusb cppunit
```

#### Install tools for OpenMHz
If you are interested in uploading recordings to OpenMHz, install FDK-AAC and Sox  to convert the Wav files to M4a.
```bash
sudo port install sox
```

Download and make [libfdk-aac](https://github.com/mstorsjo/fdk-aac).
extract the source, and cd to the source directory

```bash

autoreconf -i
./configure
make
sudo make install
```

Download and make the command line [**fdkaac** program](https://github.com/nu774/fdkaac).
extract the source, and cd to the source directory
```bash
autoreconf -i
./configure
make
sudo make install
```

#### Building Trunk Recorder
```bash
mkdir trunk-recorder && cd trunk-recorder
git clone https://github.com/TrunkRecorder/trunk-recorder.git source
mkdir build && cd build
cmake ../source -DOPENSSL_ROOT_DIR=/usr/local/opt/openssl
make -j
sudo make install
```

## Configuring Trunk Recorder

The next step is to [configure Trunk Recorder](../CONFIGURE.md) for the system you are trying to capture.

## Running trunk recorder. 

If all goes well you should now have the executable named `trunk-recorder`, and created the `config.json` configuration file as described in the [Wiki](https://github.com/TrunkRecorder/trunk-recorder/wiki/Configuring-a-System) and [Configuring Trunk Recorder](https://github.com/TrunkRecorder/trunk-recorder/blob/master/docs/CONFIGURE.md).

From your build directory (e.g. `trunk-build`) you can now run
`./trunk-recorder`

### Runtime options

Trunk Recorder will look for a *config.json* file in the same directory as it is being run in. You can point it to a different config file by using the *--config* argument on the command line, for example: `./trunk-recorder --config=examples/config-wmata-rtl.json`.
