#!/bin/bash

set -e

### Save working directory
cwd=$(pwd)
function cleanup {
  echo "Installation failed"
  cd "${cwd}"
  exec $SHELL
}
trap cleanup ERR

### Set workspace
read -p "Set pupil workspace [$HOME/devel]: " PUPIL_WS
export PUPIL_WS=$(eval echo "${PUPIL_WS:-$HOME/devel}")

mkdir -p "${PUPIL_WS}"

### Install dependencies
sudo apt install -y pkg-config git cmake build-essential nasm wget \
  libusb-1.0-0-dev libglew-dev libglfw3-dev libtbb-dev

# ffmpeg >= 3.2
# TODO maybe install through conda
sudo apt install -y libavformat-dev libavcodec-dev libavdevice-dev \
  libavutil-dev libswscale-dev libavresample-dev ffmpeg x264 x265 \
  libportaudio2 portaudio19-dev

# 3D Eye model dependencies
sudo apt install -y libgoogle-glog-dev libatlas-base-dev libeigen3-dev
sudo apt install -y libceres-dev

# turbojpeg
cd "${PUPIL_WS}"
wget -O libjpeg-turbo.tar.gz https://sourceforge.net/projects/libjpeg-turbo/files/1.5.1/libjpeg-turbo-1.5.1.tar.gz/download
tar xvzf libjpeg-turbo.tar.gz
cd libjpeg-turbo-1.5.1
./configure --enable-static=no --prefix=/usr/local
sudo make install
sudo ldconfig

# uvc
cd "${PUPIL_WS}"
git clone https://github.com/pupil-labs/libuvc
cd libuvc
mkdir build
cd build
cmake ..
make && sudo make install

echo 'SUBSYSTEM=="usb",  ENV{DEVTYPE}=="usb_device", GROUP="plugdev", MODE="0664"' | sudo tee /etc/udev/rules.d/10-libuvc.rules > /dev/null
sudo udevadm trigger
