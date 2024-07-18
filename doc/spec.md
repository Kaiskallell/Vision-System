# Vision System Specification

**Notice:** This document should be continually processed.

02.06.2022 <br />

- [Vision System Specification](#vision-system-specification)
  - [1. Introduction](#1-introduction)
    - [1.1 Purpose of this Document](#11-purpose-of-this-document)
    - [1.2 Document Conventions](#12-document-conventions)
    - [1.3 Product Scope](#13-product-scope)
      - [1.3.1 Goals](#131-goals)
      - [1.3.2 Assumptions to understand this Document](#132-assumptions-to-understand-this-document)
    - [1.4 Out of Scope](#14-out-of-scope)
    - [1.5 Open Questions](#15-open-questions)
    - [1.6 Reference to other Documents](#16-reference-to-other-documents)
  - [2. Overall Description](#2-overall-description)
    - [2.1 Purpose of the desired System](#21-purpose-of-the-desired-system)
    - [2.2 Product Functions](#22-product-functions)
    - [2.3 Components](#23-components)
    - [2.3 Operating Environment](#23-operating-environment)
  - [3. Requirements](#3-requirements)
    - [3.0 Formating Project](#30-formating-project)
    - [3.1 Camera interface](#31-camera-interface)
      - [3.1.1 General Camera Interface](#311-general-camera-interface)
      - [3.1.2 Stereo Depth Estimation](#312-stereo-depth-estimation)
    - [3.2 Neural Networks](#32-neural-networks)
    - [3.3 Calculate 6D pose of each detected product](#33-calculate-6d-pose-of-each-detected-product)
    - [3.4 Calculate 6D pose of each tray where we place products and track them](#34-calculate-6D-pose-of-each-tray-where-we-place-products-and-track them)
    - [3.5 Quality Gate](#35-quality-gate)
    - [3.6 Classification extension](#36-classification-extension)
    - [3.7 Calibration](#37-calibration)
    - [3.8 Communication with path planner (VMS) and PS](#38-communication-with-path-planner-vms-and-ps)
      - [3.8.1 Encoding of Data via Protocol Buffers](#381-encoding-of-data-via-protocol-buffers)
      - [3.8.2 Sending Poses to path planner (VMS)](#382-sending-poses-to-path-planner-vms)
      - [3.8.3 Changing Format](#383-changing-format)
    - [3.9 Simulation Mode](#39-simulation-mode)
      - [3.9.1 Simulating Tasks](#391-simulating-tasks)
      - [3.9.2 Simulating Cameras](#392-simulating-cameras)
      - [3.9.3 Simulating PS](#393-simulating-ps)
      - [3.9.4 Simulating VMS](#394-simulating-vms)
      - [3.9.5 Simulating TC3](#395-simulating-tc3)
    - [3.10 Logging](#310-logging)
    - [3.11 Google Test](#311-google-test)
    - [3.12 Auto Start](#312-auto-start)
    - [3.13 Scripting Installation](#313-scripting-installation)
    - [3.14 Clone and Recover](#314-clone-and-recover)
    - [3.15 Build Pipeline](#315-build-pipeline)

## 1. Introduction

### 1.1 Purpose of this Document

In the last version of the vision system some refactorings were made to organize the repo and to improve the structures. Unit and integration tests were introduced and trials to document the source code were made. It is not perfect, that's why the focus should remain the same in the actual version of the vision system: **DEVELOP A WELL WORKING SOFTWARE PACKAGE**. This specification aims also to define new requirements of the vision system, as well to keep the focus on improving the source code for further development. In this version, is from a big importance to introduce a software deployment pipeline. The introduction of automation in a deployment pipeline will allow us as a development team to focus more on innovating and improving the end product for the user. By reducing the need for any manual tasks, we will be able to deploy new code updates much quicker and with less risk of any human error. The time line is until the end of the year 2024.

### 1.2 Document Conventions

**Abbreviations and Short Cuts:**

* PS - Programming System: with PS can user program his application, create formats and load them at runtime.
* VS - Vision System: AI enabled computer vision.
* protobuf - Google Protocol Buffer: Protocol buffers are Google's language-neutral, platform-neutral, extensible mechanism for serializing
             structured data, think XML, but smaller, faster, and simpler.
* VMS - Verpackungsmaschinensteuerung: Path planning system for Schubert machines, this is used in the cobot for moving it. 
* TC3 - Beckhoff TwinCAT3: EtherCAT-Master Gateway for controlling directly the hardware of the cobot.
* IDS - A german camera manufactor for 2D and 3D cameras, we use IDS 2D cameras.
* Baumer - A swiss camera manufactor for 2D, we use Baumer 2D cameras.
* Product - is an object which is going to be picked like chocalate candy.
* Tray - is an object where a product is going to be placed like carton compartments.

### 1.3 Product Scope

#### 1.3.1 Goals

1. Improve code maintainability
2. Increase testing
3. Improve simulation
4. Increase automation of workflows
5. Reduce hardware costs for cameras
6. Increase Hardware independcies for cameras
7. Increase usibility for the end customer
8. Introduce updates
9. Introduce new capabilities like: placing and tracking a tray, classification extension and quality gate

#### 1.3.2 Assumptions to understand this Document

* The robot is picking objects from a conveyor belt which is shiped with the robot.
* The robot is placing objects on a conveyor belt which can be developed from the customer and is not under our control.
* The robot is picking the object from the picking conveyor and moves it (immediately or via a turning station or a classification station) to the placing conveyor. The robot is waiting there until it receives a new pose of a new pickable object.
* A pickable object is when the whole shape is visible within an image from the camera.
* When using a placing cameras trays should be detected and tracked to be able to place more than a picked object into the tray. This mean we need to give a unique id for each tray that is going to be seen.
* We can do a quality check of an object (product or tray) before picking it. The quality gate is here the measuring of the dimension of the object between different points and compare these distances with their ideal values. If one dimension is lower that expected the class of an object should be written as bad and send to the robot planer as bad.
* After picking an object the robot can go to a classification station to classify more informations about the product. With some products we are not able to see the whole features for right manipulating the product that is why a further camera is needed to collect those informations.
* A hardware trigger is used to capture images. Normally we can use Free Run mode for capturing images. This is needed to avoid images on which the robot arm obstructs the view of the picking area. The trigger is controlled by the hardware-IO component of the robot (TC3). There is a module in TC3 that is responsible for releasing the hardware trigger when the robot is outside the picking area.
  * Sometimes this can not be well understood why we need this hard real time. For small objects like chocolate candy we need to have +/-1 mm accuracy for picking the candy in a moving conveyor belt at a speed of 13 m/min. It implies that we have 0.216 mm for each 1 msec jitter.
* If no poses can be found or the poses found are corrupt, the picking conveyor belt moves forward. The purpose is to create a new scene for the Vision System. This is done directly in TC3.
* We need the ability to pick at least 80 objects per minute. 
  * Because picking one object may change the poses of the other objects, we need to run the detection before every pick. 
  * This does not mean that a procesing rate of 80 per minute is sufficient. We need to provide the pose of a pickable object when the robot is done placing the previous object. Therefore, we capture an image as soon as the robot arm leaves the scene which is inside the angle of view of the camera. Summarized we need an inference time of 300 msec from image aquisition to delivering poses to the robot (assuming that there is always at least one moveable pose in each image). By now we have 100 msec as inference time.
  * Sometimes we cannot drive to an object for the following reasons. First, the delivered object may be outside the robot's work area. Seconds the delivered pose could be wrong, for example, due to a wrong depth estimation. Third the movement of the belt could make the object no longer accessible. For these reasons we try to send all possible pickable objects to the robot with a picture so that it has more chances of approaching one.
* Because we have a bin picking, we need depth estimation in the scene. An AI based stereo depth estimation was on the last release introduced. Why we are using are own depth estimation because 3D cameras are very expensive or can not work well for all kind of products (homogenous texture or reflecting texture) or too slow. When training our own AI with are own objects we can be very accurate in comparaison of 3D cameras on the market. Also we achieved a higher performance in comparaison with 3D cameras. 
* The vision system should be able to provide 6D poses of objects in at least two moving belts. The image trigger of different camera sets should be coordinated.

### 1.4 Out of Scope

* We only deliver a list of 6D poses in one common coordinate system (robot base coordinate system). We make no special provision coordinating multiple vision systems or multiple robots.
* The Vision System is not able to have a memory for refinding products from one image to the next image.
* The Vision System does not deliver informations to the robot saying how to pick objects.
* The Vision System in this version can not do tracking of products and deliver their velocities.
* Track individual compartments in which to place picked objects through sequential detections and calculate their velocity.
* We develop this only for very specific (already known) objects, not general purpose pick and place tasks (unknown products). The customer has to tell us how the imprint of the objects will look like.
* We don't have any capability for online refinement or for quantification of the quality of neural networks.

### 1.5 Open Questions

1. Do we continue using Redis + protobuf + udp + tcp for communications? Can we use a middleware instead to unify/simplify communications? This largely depends on how we solve the robot controller -> We switch to a more appropriate middleware. We are going to use a introduce a middleware instead of redis, until now we can not introduce the middleware on other components of the robot. But a development of our own robot control is started, where a unified middleware is a must.
2. How to calibrate neural networks for using int8 in serie level? With real images? With synthetic images? How to deal with it?
3. Hard Real Time capability of the Vision System? How to trigger IOs directly with the Vision System?
4. Is there any software to automatically check the quality of an open source project, or its maintability?
5. How to achieve modularity for cameras and formats?
6. Is not better to trigger camera within the camera interface? Is not better to have the UDP inside the camera interface?

### 1.6 Reference to other Documents

* [VS and PS/ST Communication specification](https://scteam.grips.world/sites/cobot/Vision%20System/03_Software/05_Vision_System_tog519/02_Spezifikationen/01_VS2Controller_protobuf_communication/181130_SpezGrob_SchnittstelleBV_Rev0090.pdf)

* [HTTP Server specification](doc/specHttpCom.md)

* [DDS Middleware specification](doc/specDDSmiddleware.md). Removing the Redis Database with the DDS middleware.

* [Jetson IPC documentaion](https://scteam.grips.world/sites/cobot/Vision%20System/04_Hardware/02_Carrierboard/03_SysLogic). This is our hardware for the Vision System.

* [Joel Spolsky 12 rules](https://www.joelonsoftware.com/2000/08/09/the-joel-test-12-steps-to-better-code/). This will guide us in our development of this version of Vision System.

## 2. Overall Description

### 2.1 Purpose of the desired System

The vision system enables the robot to see its working environment. The premise is that the robot is placed along a conveyor belt carrying objects that are unorganized. The vision system detects individual objects and calculates their 6D poses so that the robot can pick them. The vision system detects also trays and track them, these are needed to place picked objects. It is also possible to have more than one picking area. For each area (picking or placing) exists one camera (3D stereo camera or mono cmera). Also the vision system can with a classification extension and station retrieve further informations about a product. This is needed because on the picking area because of occlusion the camera can not see all neeeded informations, for example the openning of a cover can not be well seen.

### 2.2 Product Functions

The Vision System has the following requirements:

* Determine all pickable objects from one image. This should be done using AI. The AI needs to deliver for a pickable object: the 2D bounding box surrounding the object, keypoints of the object and the class associated to the object. For the class the AI needs to differentiate between different types of objects (all of which are known a priori).
* Check if the dimensions of an object are correct. If not classify this object as bad.
* At least two picking areas should be tracked. The simple way is always using the module twice for each picking area. This should not lead to performance problems. That means we should not use heavy neural networks.
* Determine the 6D poses of objects on a conveyor belt. The objects for this version are deformable pouches, bottles, covers, pumpes, and bottles on a tray, doeses in a box. For pouches we need to differentiate between different sides of an object (all of which are known a priori)
* Enable picking at a rate of 80 objects per minute.
* Detect, determine the 6D poses of a tray on a conveyor belt where to place products.
* Classify other features of a product in a classification station.
* Functional configurability: enable fast and easy changing of the target (object) to be picked.
* Functional configurability: individual functions of the vision system can be turned on/off depending on whether they're needed.
* Hardware configurability: enable fast and easy changing of cameras used. By camera producers there is a huge problem for delivering cameras, that's why we need to be able to use any kind of a 2D camera with GenICam standard, to be independent from the producers. For this version we need to use at least Baumer and IDS cameras. 
* Using directly compressed networks in the laboratory in the .engine format, we do not need to compress networks in the robot.
* Operate with simulated/pre-recorded data.
* Operate with simulated counter parts: PS, VMS and TC3.
* Provide easy hand-eye calibration.
* Logs informations to the PS.
* Scripting the installation of the vision system.
* Possibility to backup and to recover the vision system. Creating a workflow for making updates to customers, using recover. For this purpose, in combination with the PS we need to backup the format configurations in the vision system.
* Download new neural networks for customers directly from the PS. No need to be connected to the vision system via remote desktop services.
* Introduce a pipeline to drive software development through a path of building, testing, and deploying code, also known as CI/CD. By automating the process, the objective is to minimize human error and maintain a consistent process for how software is released.

### 2.3 Components

The vision system has 5 distinct operational tasks that will be performed by 5 executables: 

1. `detectPickableObjects`
2. `placeConveyorTracking`
3. `classificatorExtension`
4. `calibration`
5. `VsCommunication`
6. `vsHttpServer`

All of these tasks can be coordinated within the service provided with the `missionController`.

`detectPickableObjects` will use RGB cameras and neural networks to detect individual objects and calculate their poses. The camera will be easily replaceable in order to take advantage of newer (better or cheaper) camera technologies. We also need a way to specify camera configuration parameters, both general (applicable to all cameras). The camera is hardware triggered. We need hardware trigger for time synchronising the whole system, because the conveyor belt is moving. The triggering of the camera is made within TC3 wich sends to `detectPickableObjects` after triggering the image number and the encoder value of the moving belt over UDP.
The detections and the image informations (image number and the encoder value) will be sent to `VsCommunication` via a middleware. The AI pipeline will be easily replaceable or reconfigurable so that we can take advantage of future advancements in deep learning. At the starting of this task, the configuration must be read. The configuration is: what type is the used camera, what are the intrinsic and extrinsic parameters of the camera, what are the specific camera parameters (here for example gain), what is the region where the camera needs to focus on, what are the needed neural networks and their parameters. At least we can use two `detectPickableObjects` in paralell. The depth is needed here to be estimated, for this purpose we are going to use a 3D Stereo camera.

`placeConveyorTracking` will use RGB cameras and neural networks to detect individual trays and calculate their 6D poses. The camera is hardware triggered. We need hardware trigger for time synchronising the whole system, because the conveyor belt is moving. The triggering of the camera is made within TC3 wich sends to `placeConveyorTracking` after triggering the image number and the encoder value of the moving belt over UDP.
The detections and the image informations (image number and the encoder value) will be sent to `VsCommunication` via a middleware. At the starting of this task, the configuration must be read. The configuration is: what type is the used camera, what are the intrinsic and extrinsic parameters of the camera, what are the specific camera parameters (here for example gain), what is the region where the camera needs to focus on, what are the needed neural networks and their parameters.

`classificatorExtension` will use RGB cameras and neural networks to detect further features of a product. The camera is hardware triggered. We need hardware trigger for time synchronising the whole system, because the conveyor belt is moving. The triggering of the camera is made within TC3 wich sends to `classificatorExtension`. At the starting of this task, the configuration must be read. The configuration is: what type is the used camera, what are the intrinsic and extrinsic parameters of the camera, what are the specific camera parameters (here for example gain), what is the region where the camera needs to focus on, what are the needed neural networks and their parameters.


`Calibration` calibrates the cameras, do the stereo calibration if we are using 3D cameras and the hand eye calibration. After success of the calculation the resulting transformation between both coordinate systems (camera and robot) will be saved automatically in the config folder of the Vision System within the serial number of choosen camera.

`VsCommunication` reads poses from `detectPickableObjects` or `placeConveyorTracking` via middleware and sends them to the robot controller (here VMS) with the specific image informations (image number and encoder value). This is required because the vision system does not run on the same physical machine as the robot controller (here VMS). If it is a `product` we need to send an `objectId` as `product` and if it is a `tray` we need to send the `objectId` as a `tray`. This is needed from VMS to understand if it is a picking object or placing object. To communicate to the robot controller (here VMS) we are currently sending messages via TCP, serialised using the google protobuf library. The advantage is that the interface is specified through the `*.proto` files, which makes the colaboration with Lachmann and Rink (developer of VMS software) easier. The `missioncontroller` is communicating to the PS via `VsCommunication` to take commands, to start and stop or to change a specific task and to load the adequate object specification that needs to be detected. `VsCommunication` is responsible for the version control with VMS and PS.

**Notice:** the UDP communication will be obsolete when we achieve the development of the robot controller V3.0. Triggering images and sending the presence of a product will be done directly within `VsCommunication` and the robot controller V3.0.

`HTTPServer` The current vision system loads neural networks from its disk to detect products. These networks need to be manually copied from a usb device by a developer. Futhermore there is no automated way to transfer log data from the Vision System computer to an other computer. For maintainability reasons we need to implement a way to transfer big amounts of data (~100Mb-5Gb) from the VisionSystem. That is why we are going to implement a `HTTPServer` for these 3 uses: Transfer Neural Networks to the vision system, Transfer config files for Neural Networks to the vision system and Transfer log data from the vision system to the PS. The specification of the interface could be found [using the following link](doc/specHttpCom.md).

### 2.3 Operating Environment

The Vision System is operating on a Nvidia Jetson Xavier AGX (32Gb) with
a Linux for Tegra OS. It needs to use Jetpack 5.0.2. The industrial version is made by Syslogic GmbH (Syslogic IPC/RMLA3K22-B203S).
With Jetpack 5.0.2 we have the ability to boot the jetson with an SD card and to clone an image of the Jetson. The SD card needs to have a memory of 500 Gb. Also with JetPack 5.0.2, they released the new version of TensorRT 8. TensorRT 8 supports more neural network architectures for accelerating the inference time of AI.

## 3. Requirements

### 3.0 Formating Project

The Vision System project must be configured as agreed in our [Software-Guidelines](https://decrtfs/GRIPS/Cobot/_git/getting-started?path=%2FSoftwareDevelopmentWorkflows.md&_a=preview).
For that we are using this project tree:

- config
- doc
- format
- libs
- licenses
- resources
- scripts
- src
- tests

We also support `clang-format` and `clang-tidy`.
clang-format is a tool to automatically format C/C++/Objective-C code, so that developers don't need to worry about style issues during code reviews. The clang-tidy is a clang-based C++ ''linter'' tool. Its purpose is to provide an extensible framework for diagnosing and fixing typical programming errors, like style violations, interface misuse, or bugs that can be deduced via static analysis. The configuration can be found [here](../resources/cppCodeQualityTools)

### 3.1 Camera interface

#### 3.1.1 General Camera Interface

Currently the vision system implements camera interfaces for multiple different cameras, which implement a uniform api. This makes it very easy to integrate a new camera. We need to support cameras with GenICam standard. At this point we support only 2D cameras. The depth will be estimated using Stereo Depth.

* physical camera present & connected
* interface to camera driver/library
  * -> open/close camera
  * -> manage resources, e.g. memory for frames
* configure camera according to settings
* fetch images from camera
* return frames

#### 3.1.2 Stereo Depth Estimation

Depth estimation is a crucial step towards inferring scene geometry from 2D images. In stereo vision, we use two stereo images from two cameras to perform disparity estimation and, next, compute depth.

### 3.2 Neural Networks

We are using two different neuronal networks. The first one is called YoloV5, which returns class information as well as bounding boxes.
The second network is called DeepLabCut and outputs keypoints, which are used to determine the right orientation of the product.


`detectPickableObjects` and `placeConveyorTracking` 
* take input image
* run yoloV5 NN
* run deeplabcut NN on the well found masks and bounding boxes
* return results
* configuration: location of the neural networks

For NN we need to calibrate each network at the first run. This calibration is needed for quanitizing the networks into int8 format. This
why we need to code this quantization. The quantization and building engine files should be done directly in the lab not on the robot. The vision system should be able to run engine files with int8 or with Float16.

### 3.3 Calculate 6D pose of each detected product

The mask of each product is used to find the relevant pixels in the depth image. These distance values are then used to calculate the coordinates in camera coordinate system. After that a RANSAC algorithm is used to fit a plane in the point cloud. With the retrieved plane and the keypoints we can calculate a direction vector which is used as x-axis and drawn in red color in the image. The z-axis is the normal vector of the plane. y-axis is calulated via cross product.

* mask out depth image
* calculate coordinates in camera coordinate system
* perform plane estimation
* calculate rotation matrix from plane and keypoints
* calculate euler angles in robot coordinate system from rotation matrix

The coordinate origin of the x/y plane is located at the anchor point of the upper arm. The coordinate origin of the x/y or x/z plane is located at the level of the lowest position 
for a product pickup (e.g. height of the lowest product belt). Thus, as a rule, only 
positive z-coordinates should result.

![](resources/coordinatesystem.png)

The algorithm must be able also to calculate the 6D pose of also homogenous non symmetrical products with the principal of `Long Short Edges`. In addition we need to be able also to see if all keypoints combinations that retrieve the x-axis of a product, are conform to eachone. If not we do not need to send the pose of the product to the vms.

### 3.4 Calculate 6D pose of each tray where we place products and track them

The center of the bounding box is the point that should be sent to the robot. The high of the tray is a parameter in the configuration of the format, we do not need to estimate the depth at first. We are focusing first of all in only x-y plane. That is why is only the orientation around the z axis relevant.
With accessing the encoder value of the belt via Tc3, we need to calculate the possible placement of trays between two images. We need to take many images as possible to track the trays movement accurately.

### 3.5 Quality Gate

We need to check for every object (product or tray) if dimensions between different keypoints are ideal. If one dimension is lower than expected the object should be marked as `bad`. This prevent the robot for picking strong deformed products. 

### 3.6 Classification extension

After picking a product we can go to a classification station where the product is going to be seen from another camera in a different perspective to retrieve more or better feature. After new classification of the product, the new informations should be sent to VMS to work with these new informations and to handle the product adequately.

### 3.7 Calibration

You must perform camera model calibration for each camera in the system before performing stereo calibration. Creating a camera model involves acquiring multiple images, usually of a calibration grid, in multiple planes. For each plane, a camera model provides a set which consists of rotation and translation matrices. Corresponding sets, computed for the left and right camera based on the same plane, provide the information required to compute the spatial relationship between the two cameras. Stereo calibration returns a single rotation and translation matrix (R, T) that relates relative real-world coordinates for the left and right cameras.

In robotics, the hand eye calibration problem (also called the robot-sensor or robot-world calibration problem) is the problem of determining the transformation between a robot and a camera or between a robot base and the world coordinate system. In our work we need to calibrate the robot base with the camera. Many different methods and solutions developed to solve the problem, broadly defined as either separable, simultaneous solutions. Each type of solution has specific advantages and disadvantages as well as formulations and applications to the problem. A common theme throughout all of the methods is the common use of quaternions to represent rotations. We can use opencv c++ for calculating the hand eye calibration. See the following [link](https://docs.opencv.org/4.5.3/d9/d0c/group__calib3d.html#gad10a5ef12ee3499a0774c7904a801b99) for more informations. Because we are going to use a 3D plate wih aruco markers we need only one image of the board to do the hand eye calibration. We need to update our pipeline for doing the hand eye calibration for different cameras. Using the PS we need to be able to choose the camera on which the hand eye calibration is going to be performed. 

**Notice:** The Hand-Eye-Calibration is not done from a customer. It is done from a specialist from the cobot department.

Please refer to the following documentation [link](CameraCalibration.md) to know how to do the calibrations.

### 3.8 Communication with path planner (VMS) and PS

`VsCommunication` will act as a server (it only can respond to incomming messages). All the messages are defined in `.proto` files and Google Protobuf is used to serialize the data. Every proto file should contain comments on why these messages are necessary and how it should work.
To handle multiple incomming request all requests are put into a queue. It's items are then processed one after another.

![](resources/communication_overview.png)

There are multiple commands with which the path planner and the PS can control which actions the Vision System should perform. The commands are:

* VERSION_CHECK
* GET_STATUS
* FORMATS_AVAILABLE
* SET_FORMAT
* GET_FORMAT

By version check we must define a unique version for the interface independent of the definition of the version of the vision system. With any change occuring in the interface, the version should be actualised.

#### 3.8.1 Encoding of Data via Protocol Buffers 

The transmission of data should be realized via [Protocol Buffers Framework](https://developers.google.com/protocol-buffers/).
It allows to define different messages in  a meta language.
A special compiler can generate serialization and deserialization function for different programming languages  (C#, C++ etc.).
The serialized data should be sent over a TCP-stream.
To simplify the receiving process we define a unified structure of the data in the TCP-stream. We distinguish between requests (from VMS or PS to VS) and responses (from VS to VMS or PS).

**Request**

|          | StartId    | sequence number | commandId  | Length of Message [bytes] | Encoded Message |
| -------- | ---------- | --------------- | ---------- | ------------------------- | --------------- |
| Datatype | uint32_t   | uint32_t        | uint32_t   | uint32_t                  | uint8_t[]       |
| Expample | 0x10101010 | 23              | 0x00010001 | 0x000000020 (32 byte)     |                 |

StartId:
- Allows to determine the start of the command and ensures that no misinterpretation of 
of data occurs 

sequence number: 
- For assigning messages to requests or responses. 

CommandId: 
- Here you can decide which deserialization function is used. In the upper 16 
bits the system is to be coded and in the lower 16 bits the command type

| System | CommandId  |
| ------ | ---------- |
| VMS    | 0x0001XXXX |
| VS     | 0x0002XXXX |
| PS     | 0x0003XXXX |

Length of Messages:
- This can be used to determine the position of the following start identifier in the stream. For the case, 
that a command does not have any further data in the form of a message, a length of 0 
is allowed

Encoded Message: 
- Message coded via protocol buffers. The data area must be deserialized according to the 
command identifier.

**Response**

|          | StartId    | sequence number | commandId  | ACK/BUSY/NACK | Length of Message [bytes]   | Encoded Message |
| -------- | ---------- | --------------- | ---------- | ------------- | --------------------------- | --------------- |
| Datatype | uint32_t   | uint32_t        | uint32_t   | uint32_t      | uint32_t                    | uint8_t[]       |
| Expample | 0x10101010 | 23              | 0x00010001 | ACK (1)       | 0x000000010 (16 byte)?? hä? |                 |


Start identifier: 
- As in the request. 
Sequence number: 
- As in the request. In the response, the sequence number is returned, so that it is clear to which 
to which command the response refers. 
Command identifier: 
- As in the request. In the response, the identifier is returned so that it is clear to which command the response refers. 
command the response refers to. 
ACK / NACK / BUSY: 
- The field describes the status of the request. The following 3 values are distinguished:
  * NACK (0x00000000): The command was rejected (no change of the system state of VS). 
  * ACK (0x000001): The command was accepted. In the following message 
the answer can be found. 
  * BUSY (0x00000002): The command was accepted. However, time is needed until 
the results of the request are available. The request (with the same sequence number and 
same command identifier) should be repeated until a response with an 
ACK or NACK occurs. 
Length of the message: 
- As in the request. 
Coded message 
- As in the request

As a general rule:
* **every request should have a associated response**

#### 3.8.2 Sending Poses to path planner (VMS)

The pose estimation is done on the executable `detectPickableObjects` or `placeConveyorTracking`. However the sending of the poses is done in a seperate process `VsCommunication`. A middleware will be used to enable inter process communication beteween them. `VsCommunication` can only respond to requests which are issued by VMS. VMS is constantly asking: "is there a new pose?" (-> polling).

* write all poses
* in second process: read all poses and delete them
* compress information via protobuf
* send string via TCP to path planner

All message definitions are stored in this [file](../src/vs_communication/proto/STVS/GetStatus.proto). The definitions are not written in this document to avoid diverging definitions of the same message.

![](resources/getStatus.png)

#### 3.8.3 Changing Format

All message definitions are stored in this [file](../src/vs_communication/proto/VSPS/Format.proto). The definitions are not written in this document to avoid diverging definitions of the same message.

![](resources/Format.png)

### 3.9 Simulation Mode

We need to run the Vision System in simulating mode. This is done for rapid prototyping and testing functionalities on the laboratory.
We need to distinguish here between two simulation modes: `full simulation` or `free run simulation`.
If `full simulation` mode will run, these modules needs to be also simulated:

1. Tasks
2. Cameras
3. PS
4. VMS
5. TC3

In `free run simulation`, the cameras will be run in Software Trigger mode and we do not need PS, VMS and TC3 parts to be used.

#### 3.9.1 Simulating Tasks

We attend only to simulate one `detectPickableObjects` or `placeConveyorTracking`.

#### 3.9.2 Simulating Cameras

We need to capture images from a real camera and store them in a specific folder. We do not need to make this simulation exponential hard. 

#### 3.9.3 Simulating PS

The Simulated PS is only a mockup PS with user interactions within a terminal. 

1. The user can choose if he wants to verify the version of the Vision System or to see the loaded format or send new format or to see available formats.
2. For verifying the version, the user needs to write the version of PS, sends the request to the Vision System and see the actual version of the Vision System.
3. For the interaction for the format, the user can send a request seeing which formats are available, then requests a new format and finally can see if the format was really loaded.

#### 3.9.4 Simulating VMS

The Simulated VMS is only a mockup VMS with user interactions within a terminal.

1. The user can choose if he wants to verify the version of the Vision System or to use GetStatus.
2. For verifying the version, the user needs to write the version of VMS, sends the request to the Vision System and see the actual version of the Vision System.
3. Running GetStatus requests in 30msec and see if all poses of all frames are well arrived at the end of the pipeline.

#### 3.9.5 Simulating TC3

The Simulated TC3 is only a mockup TC3 with user interactions within a terminal.

1. After each TakeFrame request from the Vision System, the image number will need to be increased and a random encodervalue needs to be also sent.
2. At starting we need also simulate the sync mechanisms between Vision System and TC3.

**Notice:** It will be good to simulate also problems like connection lost. But this is out of scope for this version.

### 3.10 Logging

In this project we want to apply `spdlog library for Logging`. spdlog is a header only library. Just copy the files under include to your build tree and use a C++11 compiler.
Logging is also collecting data to our cloud. For that the Vision System needs to collect all data in a folder called `Vision System Logs`. This is outside the Vision System. This folder needs to be shared with the robot controller using the `HTTPServer`. The logs of different tasks should be saved in a chronological way on the same file. Otherwise we loose the debug capability. We could implement a routine to rearrange the logs when a request comes from PS via the `HTTPServer` to upload the logs of the vision system. 

### 3.11 Google Test

Google Test (also known as gtest) is a unit testing library for the C++ programming language, based on the xUnit architecture. This is used in our project to make our unit tests. We want to achieve in this project 70% of automated unit test code covery.

### 3.12 Auto Start

For Autostart we are using `systemd`. Also we need to start the `performance service for jetson`. The needed files for this configuration can be found under the [following link](../resources/jetson-config/systemd/system). Please refer to the `readme.md` of the Vision System to see how to put the auto start.

### 3.13 Scripting Installation

How many steps does it take to make the preparation of the hardware to host the software of the Vision System and start the production?
In good teams, there's a single script you can run that does a full installation from scratch. If the process takes any more than one step, it is prone to errors.
That's why we need to script our installation workflow. We need to maintain also in our cloud, the requirements list needed for every release version.
For preventing errors and bugs before releasing a new version of the installation script, the script needs to be commented, one time reviewed by another person and two times automatically tested. 
Also for preventing bugs we need to check if the project or the package is constantly maintained, the lifecycle of it and the feedback from the community. We introduce a golden rule for that: before installation of anything new, check these with at least one person in the team and directly with the developer or company responsible of this software.

### 3.14 Clone and Recover

Having most up to date software backup can help avoiding extended machine downtime in the event of software corruption or IPC failure. It could also streamline the ordering of an Upgrade. That is why we need in our Vision System to well define this workflow. How to do it with Jetson, how to do it with Syslogic IPC based on Jetson and how to organise our backups in the cloud. We need also to define the workflow for making updates to the customer and to save the format configurations using the PS. 

### 3.15 Build Pipeline

A CI/CD pipeline automates your software delivery process. The pipeline builds code, runs tests (CI), and safely deploys a new version of the application (CD). Automated pipelines remove manual errors, provide standardized feedback loops to developers, and enable fast product iterations. CI, short for Continuous Integration, is a software development practice in which all developers merge code changes in a central repository multiple times a day. CD stands for Continuous Delivery, which on top of Continuous Integration adds the practice of automating the entire software release process. With CI, each change in code triggers an automated build-and-test sequence for the given project, providing feedback to the developer(s) who made the change. The entire CI feedback loop should run in less than 10 minutes. Continuous Delivery includes infrastructure provisioning and deployment, which may be manual and consist of multiple stages. What's important is that all these processes are fully automated, with each run fully logged and visible to the entire team. Most software releases go through a couple of typical stages in a CI/CD pipeline:

![](resources/CI_CDPipeline.png)


