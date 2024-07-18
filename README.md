# Introduction 
Repository for Cobot Vision System Software.
This repo is organised like this:

1. `config/`: configuration files needed to do inference, for example camera configurations, ip adresses, ...
2. `doc/`: documentation needed to understand the goal of the Vision System and to understand the tools and methodes used for example how to make a camera calibration, ...
3. `format/`: here every format is going to be stored. In the folder `0/` are stored templates for guiding the user with configuring other formats. In the folder `999/` is a template for guiding a user to do the calibrations.
4. `libs/`: libraries needed in the project.
5. `licenses/`: as the name suggests, are here the different licenses of the project stored.
6. `resources/`: jetson configuration as network configuration and startup files. We store here also different neural networks that are going to be used. 
7. `scripts/`: installation scripts for semi-automatically preparing the jetson after purchase from delivery.
8. `src/`: source code of the Vision System.
9. `tests/`: source code of unit and integration tests of the Vision System.

When starting working with the project you should first read the documentaion `VisionSystem/doc/spec.md` of the Vision System to understand the functionalities and the main goal of this project. When you have a jetson in front of you, and you need to see how to connect the jetson with other hardware like the robot or cameras, please refere to `VisionSystem/doc/networkconfiguration.md`.

In this document we are going to show you first how to install and setup the jetson for use and second usefull commands are going to be shown, to do specific works within the project.

# Installation
For installation there are two methodes:
1. `Deploying a syslogic image`: the jetson where the Vision System is running is a syslogic IPC. When purchasing a new syslogic computer, you need first to flash the jetson with the syslogic image and then run a script to download, make the installation of the dependencies and finally setup the jetson. This methode can be usefull, if you want to install stuffs by yourself or make new installation environment (new dependencies) and check if everything is ok. 
2. `Deploying a production image`: This methode is usefull for gaining time or producing in quantity jetsons for robots that are going to be shipped to customers. Especially the `hardware fertigung` of `schubert` is going to use this methode, to install the newest stable version of the Vision System.

Notice that we are using `Jetpack 5.0.2` and we assume that you can use `nvidia sdk manager` for installing a `jetson dev-kit`. If not please refere to here](https://docs.nvidia.com/jetson/l4t/index.html#page/Tegra%20Linux%20Driver%20Package%20Development%20Guide/flashing.html#wwpID0E0MQ0HA).

## Deploying a syslogic image

1. Preparing for cloning/flashing syslogic IPCs:

Cloning/flashing of syslogic jetson is very similar to cloning/flashing a regular jetson as described [here](https://docs.nvidia.com/jetson/l4t/index.html#page/Tegra%20Linux%20Driver%20Package%20Development%20Guide/flashing.html#wwpID0E0MQ0HA).

The syslogic flashing procedure is described on their [webpage](https://www.syslogic.com/deu/edge-inference-ai-computer-rml-a3-104329.shtml?c4=0&total=). We use Jetpack 5.0.2. The password is: wIgHwvPVhm1 \
There you find a readMe.txt with the following content:

```
On the Ubuntu 20.04 host computer:

1.	Before a Syslogic Device can be flashed, install the NVidia SDK Manager and download JetPack 5.0.2, if asked, skip flashing (not installing) of the target device. 
	Do not check box "download now, install later" as this would not install everything necessary for flashing.

2. 	Copy the install script and the Linux_for_Tegra.tar.gz (inside the SYSLOGI_L4T_JP5.0.2.tar.gz archiv) into the JetPack 5.0.2 Folder (normally under ~/nvidia/nvidia_sdk/). 
	The folder name is
	for XAVIER AGX:
	JetPack_5.0.2_Linux_JETSON_AGX_XAVIER_TARGET

	for XAVIER NX:
	JetPack_5.0.2_Linux_JETSON_XAVIER_NX

3.	After, run the install.bash script to untar the syslogic configuration files into the Linux_for_Tegra folder.

4.	To flash a syslogic device, go into the Linux_for_Tegra folder and execute the flash_syslogic.sh script.
	If a prepared image should be used, copy the system.img (for example taken from CNF_L4TJP502-A3.img.zip) into the bootloader folder as "bootloader/system.img" before executing the script.
	Otherwise the image can be freshly build - for the first boot, oem configuration will be necessary,
```

Connect Jetson boards to the x86 host via the mircoUSB and put the device into recovery mode.\
		- Press and hold the REC button (S100). // Anmerkung(aschaefer): unterer Knopf\
		- While pressing the REC button, press and release the RESET button (S101). // Anmerkung(aschaefer): oberer Knopf \
		- Wait 2 seconds and release the REC button.
		
2. Preparing the environment:

After flashing the jetson, connect it with a display via the display-port. Notice to see the jetson you need a display-port to a display-port connection, otherwise it will not work. Make the installation of `L4t` like every `Ubuntu` computer. These settings are needed:
1. `User Name`: cobot
2. `password`: schubertKobold!
3. `language`: english
4. `keyboard`: english
5. `location and time yone`: berlin

After that you need to connect the just flashed jetson to the internet. Please do not install new stuffs like `chromium`. Check `autologin` otherwise `VNC-server` is not going to work. 

3. Installation of the Vision System:
First Binaries of external librarys are stored in the Cobi network storage: `file:///Y:/21%20-%20Software/02_VisionSystem/01_VS_resources/VisionSystem/resources/installFiles` . This folder needs to be copied to `VisionSystem/resources`. Second run the following script `scripts/install_requirements.sh` on your terminal, which installs all libraries and packages which are needed for running VisionSystem (OpenCV, pcl etc.). Now you can use the Vision System and run application. Notice that in this step the Vision System is not in autostart. This will be discussed on the next chapter.

4. boot from ssd (if you want to clone a new image, please do not do this step):
Every jetson in production should contain an internal ssd and should boot from their. If you have deployed an image properly you should now enable the booting from internal ssd. You can access to device with display and mouse or with ssh. If you would like to access to the device with ssh, you should type from your host computer connected to the jetson with LAN cable via `LAN2 interface of the jetson` with the following password `schubertKobold!`:
```
ssh cobot@192.168.52.120
```

On the device you can type the following commands to boot from internal ssd:

```
cd ~/Downloads
cd sbts-boot-from-SSD
sudo ./sbts_install_boot_from_SSD.sh
```

## Deploying a production image
After following the steps in [Preparing for cloning/flashing syslogic IPCs](##Deploying-a-syslogic-image), execute the following commands to deploy the image

Copy your image and raw image into the `bootloader` folder in you `Linux_for_Tegra` folder and change it's name to `system.img` and `system.img.raw`. There may already be a file called `system.img` and `system.img.raw` from previously flashing a jetson device. Back it up if you would like. Then, execute the following to deploy the image.

```
sudo ./flash_syslogic.sh
```
Press 'Y' to ensure that you use the created image.

After that you can also do the boot from ssd, as described above.

# Usefull commands

## Creating a backup image or a production image
After following the steps in [Preparing for cloning/flashing syslogic IPCs](##Deploying-a-syslogic-image), execute the following command to clone the image. Notice that creating a backup only from a jetson which does not boot from internal ssd.

```
sudo ./flash.sh -r -k APP -G backup.img brla3 mmcblk0p1
```

This will create the `backup.img` backup image as well as the `backup.img.raw` sparse image 

## Autostart
We use `systemd` to start all of the Vision System programs as services. `systemd` allows us to start, stop, enable, dusable, and query the status of services easily using `systemctl`
```
sudo systemctl start missionControl_jetpack5
sudo systemctl enable missionControl_jetpack5
```
The output of the services is logged by `journalctl` and can be viewed using
```
journalctl  -fu <service>
```
The option `-f` means you follow the output, i.e. the logs update on the console as they are written by the service </br>
The option `-u` is used to specify the unit (i.e. service) that we want the logs for.

## Camera Calibration
In `VisionSystem/doc/CameraCalibration.md` you can find a explaination of how to calibrate the cameras connected to the jetson.

## Manual start of VisionSystem
If you want to start the VisonSystem manually you first need to stop the processes which are already running: 
`sudo systemctl stop missionControl_jetpack5`
Then you can go to the folder:
`cd ~/Documents/git/VisionSystem/build/bin` \
And the execute:
`./missionController`

If you are debugging and only want to execute the detection part of VisionSystem you can do so with this comand:
`./detectPickableObjects -networksConfigPath="/home/cobot/Documents/git/VisionSystem/format/XX/YY.json"`

