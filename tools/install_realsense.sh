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

# Set OS version correctly for mint, ubuntu
if [ -f /etc/os-release ]; then
	source /etc/os-release
	export UBUNTU_VERSION=$UBUNTU_CODENAME
	echo $UBUNTU_VERSION
	export EXTRA=''
else
	export UBUNTU_VERSION=$(lsb_release -sc)
	export EXTRA=' -u'
fi

### install SDK
sudo apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE || \
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE

sudo add-apt-repository \
"deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo ${UBUNTU_VERSION} main {$EXTRA}"

sudo apt-get update

sudo apt-get install -y \
 librealsense2-dkms librealsense2-utils librealsense2-dev \
 librealsense2-udev-rules
