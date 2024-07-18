# Guide to Camera Calibration on the Tog519

Camera calibration using a 3D Aruco-board is an essential step to correct distortions and inaccuracies in the images captured by your camera. This is particularly crucial to achieve high precision in the Cobot's picking and placing of products. Here is a guide on how to perform camera calibration:

# Required Hardware:

1. **3D-Aruco-Board:** This is a specialized device consisting of a square pattern with Aruco markers used for calibration. Currently we use a board with 49 markers.
2. **Siemensstern:** A Siemensstern is a test pattern used to evaluate a camera's sharpness and resolution by demonstrating the camera's ability to depict fine details of a radial pattern.
3. **Camera:** To perform calibration, the camera must already be securely installed in its intended position on the Cobot. If there is a plan to reposition the camera later, recalibration will be required afterward.
4. **Laptop:** Additionally, you will need a Schubert laptop to establish a connection with the Cobot. You will require specific permissions to adjust your computer's IP address to connect to the Cobot. Furthermore, you will need the appropriate software to establish an RDP (Remote Desktop Protocol) connection to the Cobot.

# Steps for Camera Calibration:

## 1. Preparing the 3D-Aruco-Board:

The required Aruco board is usually found among the tools and accessories located with the Cobots in Hall 1 North. Before using the Aruco board, it should be verified that it is the current version of the board and free from any damage or deformities.

## 2. Connecting to he Cobot:

To connect to the Cobot (**IP: 192.168.52.120**), first, configure the Ethernet interface on the laptop being used. For this purpose, you can use the IP address 192.168.52.119 with a subnet mask of 255.255.255.0, for example. It is recommended to save this network configuration as a permanent option to avoid having to repeat this step multiple times. Now, you can use VNC software (e.g., Remmina) to access the Syslogic computer inside the Cobot. The required passwords for this should be known.

## 3. Camera Settings and Capturing Images:

The cameras installed in the Cobot can only be used exclusively by the Vision System (VS) or the BaumerCam software, which is used to capture calibration photos. Since the VS starts by default when the Cobot is powered on, it must be terminated first:

**`sudo systemctl stop missionControl_jetpack5.service`**

To access the cameras in use, we use the Baumer Camera Explorer. The software is launched from the “/opt/baumer-camera-explorer/bin/” directory with the command "./bexplorer".

**`cd /opt/baumer-camera-explorer/bin`**

**`./bexplorer`**

Now, in the graphical interface of the software, you can select the desired camera. 
  <center>

  <div style="width: 800px">
  
![](resources/Calibration_select_camera_in_bexplorer.png)
  </div>

   </center>
  

A window will open, displaying the camera's output.
If no ouput image from the camera is visible, this means that the camera is set on hardware trigger mode. To change this, you can change the trigger mode as displayed in the following images.

  <center>
  <div style="display: flex; justify-content: center;">

  <div style="width: 800px">

  ![](resources/Calibration_activate_software_trigger_1.png)
  </div>
  <div style="width: 800px">

  ![](resources/Calibration_activate_software_trigger_2.png)
  </div>

  </center>

Trigger mode must be set to `Off`.
<p style='color:red'>* Note: When finished with the calibration, it is important to set the trigger mode back to the value that you found.</p>



Once you can see the desired camera output, it is advisable to first check if the camera is focused. To do this, place the Siemens star on the conveyor belt under the camera and assess how well the test pattern is visible. If the test pattern is poorly displayed, you can loosen the fine screws on the camera lens and then optimize the image sharpness by adjusting the lens.

  <center>
  <div style="display: flex; justify-content: center;">

  <div style="width: 300px">


  ![](resources/Siemens_star.jpg)
  Siemens Star
  </div>
  <div style="width: 200px">
  </center>

Once the test pattern is clearly visible, you can begin capturing calibration images. 
But first you need to define the folder where the images will be saved:

  Click `file` -> `settigs` -> `Image` -> `Image Folder` and select the desired folder.
  
   <center>
  <div style="display: flex; justify-content: center;">

  <div style="width: 800px">

  ![](resources/Calibration_select_images_folder_1.png)
  </div>
  <div style="width: 800px">

  ![](resources/Calibration_select_images_folder_2.png)
  </div>

  </center>


A series of images from various angles of the 3D Aruco board should be taken. 
To save an image click on this symbol ![](resources/Calibration_take_image_symbol.png).
You can find it in the bottom left corner of the window.


The distance (within the camera's focus range) and, in particular, the orientation towards the camera should vary. It is recommended to take around 20 shots.
The exposure time should be selected so that the brightness of the board in the displayed image looks like in the attached photo of the documentation. (In case of doubt, rather a little brighter than darker).

## 4. Prepare the calibration environment:

The **"singleCameraCalibration.json"** file integrated into the calibration software, located in the directory **“/home/cobot/Documents/git/VisionSystem/format/999/”** needs to be adjusted before starting the calibration process.
To do this, open the JSON file in an editor. You need to specify the image folder under "calibrationConfigs/ImagePath:" with an absolute path. Here where the images were stored. Further you need to specify also the serial number of the camera under "cameraConfigs/camera/serialNumbers" and specify the Pose Image under "calibrationConfigs/ImagePath:" with an absolute path.

 ## 5. Applying Calibration:

To initiate the actual calibration process, navigate to the directory "/home/cobot/Documents/git/VisionSystem/build/bin/" of the Vision System in the terminal. Then, execute the calibration using the following command:

**`cd /home/cobot/Documents/git/VisionSystem/build/bin`**

**`./calibration -networksConfigPath=/home/cobot/Documents/git/VisionSystem/format/999/singleCameraCalibration.json`**


# Steps for Hand Eye Calibration:

We can do the handeye calibration of a camera, only when the camera was well calibrated.

# 1.Take pose image and pose of the robot:

You need to take the Pose Image (an image of the calibration board with the robot arm attached to it and all markers clearly visible). Additionally, the corresponding orientation of the robot arm is required for the Pose Image.


You can obtain this information from the PS (Programming System) accessed via

http://192.168.52.89:5000/ps/home

Once the PS main page is open, you can click on "DIAGNOSE" at the bottom right of the screen to obtain the robot coordinates. A simple method here would be to take a photo of the current position with your smartphone. The steps for taking an image with the camera were already described above and here are not be going to be repeated.

## 2. Prepare the calibration environment:

The **"handEyeCalibration.json"** file integrated into the calibration software, located in the directory **“/home/cobot/Documents/git/VisionSystem/format/999/”** needs to be adjusted before starting the calibration process.
To do this, open the JSON file in an editor. You need to specify the image folder under "calibrationConfigs/ImagePath:" with an absolute path. Here where the images were stored. Further you need to specify also the serial number of the camera under "cameraConfigs/camera/serialNumbers" and specify the Pose Image under "calibrationConfigs/ImagePath:" with an absolute path. Also you need to write the pose of the robot, this should be done under "calibrationConfigs/poses/calibPoses:". The coordinates should be written in mm.

 ## 3. Applying Calibration:
To initiate the actual calibration process, navigate to the directory "/home/cobot/Documents/git/VisionSystem/build/bin/" of the Vision System in the terminal. Then, execute the calibration using the following command:

**`cd /home/cobot/Documents/git/VisionSystem/build/bin`**

**`./calibration -networksConfigPath=/home/cobot/Documents/git/VisionSystem/format/999/handEyeCalibration.json`**


# Steps for Stereo Calibration:

We can do the stereo camera, only when the cameras were well calibrated.

# 1. Take one image from each camera:

You need to take image (an image of the calibration board where all markers clearly visible) from each camera. The board should be at a fixed position for both images.

The steps for taking an image with the camera were already described above and here are not be going to be repeated.

## 2. Prepare the calibration environment

The **"stereoCalibration.json"** file integrated into the calibration software, located in the directory **“/home/cobot/Documents/git/VisionSystem/format/999/”** needs to be adjusted before starting the calibration process.
To do this, open the JSON file in an editor. You need to specify the image path under "stereoCalibration/pathToImgOfLeftCam:" (for the left camera) and "stereoCalibration/pathToImgOfRightCam:" (for the right camera) with an absolute path. Here where the images were stored. 

Further you need to specify also the serial number of the cameras, under "stereoCalibration/serialNbrOfLeftCam" (for the left camera) and under "stereoCalibration/serialNbrOfRightCam" (for the right camera).

Additionally you need to adjust the target resolution for the depth estimation. These parameters are to be found under "stereoCalibration/targetSize/width" and "stereoCalibration/targetSize/height". Generally we use a resolution of "832x832".

In addition we need also to specify the cropping value of both images. These parameters are to be found under "stereoCalibration/uOffset" and "stereoCalibration/vOffset".

Finally we need to name the file where the result of the stereocalibration are going to be stored. This is under "stereoCalibration/outputFileName". Every stereocamera should have a unique associated file.

 ## 3. Applying Calibration:
To initiate the actual calibration process, navigate to the directory "/home/cobot/Documents/git/VisionSystem/build/bin/" of the Vision System in the terminal. Then, execute the calibration using the following command:

**`cd /home/cobot/Documents/git/VisionSystem/build/bin`**

**`./calibration -networksConfigPath=/home/cobot/Documents/git/VisionSystem/format/999/stereoCalibration.json`**

