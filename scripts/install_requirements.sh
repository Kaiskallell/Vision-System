# @copyright Copyright (c) 2022 Gerhard Schubert GmbH - All Rights Reserved
#
# Unauthorized copying of this file, via any medium is strictly prohibited
# Proprietary and confidential

#!/bin/bash -i
#
# install all dependencies and libs which are needed to execute VisionSystem
#

#set -e # Exit immediately if a command exits with a non-zero status

apt_install(){
	sudo apt install $1 -y
	if [ $? -gt 0 ]
	then
		echo -e "\e[1;31mfailed to install $1, exiting \e[0m"
		exit 1
	else
		echo -e "\e[1;32minstalled $1 \e[0m"
	fi
}

pip_install(){
	python3 -m pip install $1
	if [ $? -gt 0 ]
	then
		echo -e "\e[1;31mfailed to install $1, exiting \e[0m"
		exit 1
	else
		echo -e "\e[1;32minstalled $1 \e[0m"
	fi
}

setup_vnc(){
	# Enable the VNC server to start each time you log in
	mkdir -p ~/.config/autostart
	cp /usr/share/applications/vino-server.desktop ~/.config/autostart

	# Configure the VNC server
	gsettings set org.gnome.Vino prompt-enabled false
	gsettings set org.gnome.Vino require-encryption false

	# Set a password to access the VNC server
	# Replace thepassword with your desired password
	gsettings set org.gnome.Vino authentication-methods "['vnc']"
	gsettings set org.gnome.Vino vnc-password $(echo -n 'thepassword'|base64)

	# Reboot the system so that the settings take effect
	#sudo reboot
	echo "Reboot the system so that the vnc settings take effect"
}

install_cmake(){
  sudo apt purge cmake -y # uninstall old cmake
  cp -r ../resources/installFiles/cmake-3.25.1-linux-aarch64 ~/Downloads/
  rm -r ../resources/installFiles/cmake-3.25.1-linux-aarch64
  if [ $? -gt 0 ]
	then
		echo -e "\e[1;31mfailed to copy cmake, exiting... \e[0m"
		exit 1
	else
		echo -e "\e[1;32mcopied cmake to ~/Downloads \e[0m"
  fi
  cmakePath=$(realpath ~/Downloads/cmake-3.25.1-linux-aarch64/bin) # folder where the cmake binaries are located
  PATH=${PATH}:${cmakePath} # rest of this script can find cmake
  echo 'export  PATH=${PATH}:'${cmakePath} >> ~/.bashrc # every new terminal will find cmake
}

install_pcl(){
  sudo apt purge libpcl-dev -y # uninstall old pcl
  cd ~/Downloads
  git clone --branch pcl-1.12.1 https://github.com/PointCloudLibrary/pcl.git
  mkdir pcl/build
  cd pcl/build
  cmake -DCMAKE_BUILD_TYPE=Release ..
  make -j8
  if [ $? -gt 0 ]
	then
		echo -e "\e[1;31mfailed to install pcl, exiting \e[0m"
		exit 1
	else
		echo -e "\e[1;32minstalled $1 \e[0m"
  fi
  
  sudo make install
  cd ../../
  sudo rm -r pcl/
  cd $installScriptDir
}

install_boost(){
	tar -zxvf ../resources/installFiles/boost_1_71_0.tar.gz --directory ~/Downloads/
	rm -r ../resources/installFiles/boost_1_71_0.tar.gz
	cd ~/Downloads/boost_1_71_0
	./bootstrap.sh
	sudo ./b2 --with=all -j8 install
	echo -e "\e[1;32minstalled boost library \e[0m"
	cd ..
	sudo rm -r boost_1_71_0/ # clean up to save disk space 
	cd $installScriptDir # go back to VisionSystem/scripts folder
}

installScriptDir=$(pwd)
username="cobot"
if [ "$USER" != "$username" ]
then
   echo "Username is not how it supposed to be. Change it to '$username' "
   echo "Make a new user with the name 'cobot' then log out and delete the old 'nvidia' user"  
   exit 1 # do not install anything if username is not correct
fi
echo "User name is: $username" 

# apt stuff
{ sudo apt update && echo -e "\e[1;32mapt update successful \e[0m"; } || { echo -e "\e[1;31m apt update failed, exiting \e[0m" && exit 1; }

echo "install cuda"
apt_install "cuda-toolkit-11-4"

echo "setting up cuda"
command -v nvcc >/dev/null 2>&1
if [ $? -gt 0 ]
then
	echo -e "\e[1;32madding nvcc to path \e[0m"
	echo "# Add 32-bit CUDA library & binary paths:" >> ~/.bashrc
	echo "export  PATH=/usr/local/cuda-11.4/bin:$PATH" >> ~/.bashrc
	{ source ~/.bashrc && echo -e "\e[1;32msuccessfully sourced bashrc \e[0m"; } \
	|| { echo -e "\e[1;31mfailed to source bashrc \e[0m" && exit 1; }
	{ command -v nvcc >/dev/null 2>&1 && echo -e "\e[1;32msuccessfully added nvcc to PATH \e[0m"; } \
	|| { echo -e "\e[1;31mfailed to add nvcc to path, exiting \e[0m"  && echo "you need to open another terminal and start the script again, This is a todo to improve install script" && exit 1; }
else
	echo -e "\e[1;32mnvcc is already on PATH, no need to add \e[0m"
fi

if [ -f "/usr/lib/libcudart.so" ]
then
	echo -e "\e[1;32mlibcudart symlink exists, no need to create\e[0m"
else
	{ sudo ln -s /usr/local/cuda/lib64/libcudart.so /usr/lib/libcudart.so && echo -e "\e[1;32msuccessfully created cudart symlink \e[0m"; } \
	|| { echo -e "\e[1;31mfailed to create cudart symlink \e[0m" && exit 1; }
fi

echo "removing unnecessary packages"

{ sudo apt purge libreoffice-* -y && echo -e "\e[1;32mremoved libreoffice \e[0m"; } || { echo -e "\e[1;31mfailed to remove libreoffice, exiting \e[0m" && exit 1; }
{ sudo apt purge thunderbird -y && echo -e "\e[1;32mremoved thunderbird \e[0m"; } || { echo -e "\e[1;31mfailed to remove thunderbird, exiting \e[0m" && exit 1; }
{ sudo apt autoremove -y && echo -e "\e[1;32mautoremoved unused packages \e[0m"; } || { echo -e "\e[1;31mfailed to autoremove, exiting \e[0m" && exit 1; }

echo "installing dependencies of VisionSystem"

apt_install "git"
apt_install "libprotobuf-dev"
apt_install "protobuf-c-compiler"
apt_install "protobuf-compiler"
apt_install "redis"
apt_install "libhiredis-dev"
apt_install "libflann-dev"
apt_install "libeigen3-dev"
apt_install "libhdf5-dev"
apt_install "libpcl-dev" # does install boost-all-dev, too which is needed for missionController, udp etc.
install_boost
install_cmake
install_pcl

apt_install "libcudnn8"
apt_install "libcudnn8-dev"
apt_install "tensorrt"

apt_install "vino" # vnc server
apt_install "./../resources/installFiles/ids-peak-linux-arm64-2.3.0.0.deb" # ids camera driver
apt_install "strace" # for attaching to stdcout (fd/0) of running process
apt_install "clang-format-10" # formatting code when using ide
apt_install "clang-tidy-10" # clean code standards
apt_install "libssl-dev" # sha1 sum in http server

# python stuff
apt_install "python3-pip"
sudo apt install -y python3-parted

# dummy video driver for vnc without monitor (needed in jetpack 5.0.2)
apt_install "xserver-xorg-video-dummy"

{ python3 -m pip install --upgrade pip && echo -e "\e[1;32mupgraded pip \e[0m"; } || { echo -e "\e[1;31mfailed to upgrade pip, exiting \e[0m" && exit 1; }

{ sudo -H pip3 install -U jetson-stats && echo -e "\e[1;32msuccessfully installed jetson-stats \e[0m"; } || { echo -e "\e[1;31mfailed to install jetson-stats, exiting \e[0m" && exit 1; }

# setup Baumer explorer
echo "Installing Baumer Explorer"
cd ~/Documents/git/VisionSystem/resources/installFiles
chmod u+x Baumer_Camera_Explorer_3.3.2_lin_aarch64.deb
sudo dpkg -i u+x Baumer_Camera_Explorer_3.3.2_lin_aarch64.deb
cd ~/Documents/git/VisionSystem/scripts

# invoke opencv install script 
# needed because opencv which comes with jetpack
# does not contain charuco stuff which is in opencv_contrib 
chmod u+x install_opencv.sh
./install_opencv.sh

# build VisionSystem
mkdir ../build
cd ../build/
cmake ..
make -j8
if [ $? -gt 0 ]
	then
		echo -e "\e[1;31mfailed to install Vision System, exiting \e[0m"
		exit 1
	else
		echo -e "\e[1;32minstalled Vision System \e[0m"
fi
cd ../scripts/

# copy tool to put every thing on a bigger ssd
cp -r ../resources/installFiles/sbts-boot-from-SSD-main ~/Downloads/
rm -r ../resources/installFiles/sbts-boot-from-SSD-main

# make Vision System start as systemd service on startup
cd ~/Documents/git/VisionSystem/scripts
chmod u+x addToAutostart.sh
./addToAutostart.sh

# make vnc work
setup_vnc
chmod u+x adjustXorgConfForVNC.sh
./adjustXorgConfForVNC.sh

# set rtc correctly
chmod u+x preserveSystemTime.sh
./preserveSystemTime.sh

# set the wallpaper to be the cobot design
gsettings set org.gnome.desktop.background picture-uri "file:///home/cobot/Documents/git/VisionSystem/resources/jetson-config/Wallpaper.jpg"

echo "This script will exit"

