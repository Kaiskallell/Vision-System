# Http Communication between VisionSystem and ProgrammingSystem

**Notice:** This document should be continually processed.

20.05.2022 <br />

- [Http Communication between VisionSystem and ProgrammingSystem]
  - [1. Introduction](#1-introduction)
    - [1.1 Purpose of this Document](#11-purpose-of-this-document)
    - [1.2 Document Conventions](#12-document-conventions)
    - [1.3 Product Scope](#13-product-scope)
      - [1.3.1 Goals](#131-goals)
      - [1.3.2 Assumptions to understand this Document](#132-assumptions-to-understand-this-document)
    - [1.4 Out of Scope](#14-out-of-scope)
    - [1.5 Open Questions](#15-open-questions)
  - [2. Overall Description](#2-overall-description)
    - [2.1 Purpose of the desired System](#21-purpose-of-the-desired-system)
    - [2.2 Product Functions](#22-product-functions)
    - [2.3 Components](#23-components)
    - [2.3 Operating Environment](#23-operating-environment)
  - [3. Requirements](#3-requirements)

## 1. Introduction

### 1.1 Purpose of this Document
The current Vision System loads neural networks from its disk to detect products. These networks need to be manually copied from a usb device by a developer. Futhermored there is no automated way to transfer log data from the Vision System computer to an other computer. For maintainability reasons we need to implement a way to transfer big amounts of data (~100Mb-5Gb) from the VisionSystem.

### 1.2 Document Conventions

**Abbreviations and Short Cuts:**

* PS - Programming System: with PS can user program his application, create formats and load them at runtime.
* VS - Vision System: AI enabled computer vision.

### 1.3 Product Scope

#### 1.3.1 Goals
* Implementation of http server on VisionSystem
* Transfer Neural Networks to the VisionSystem
* Transfer config files for Neural Networks to the VisionSystem
* Transfer log data from the VisionSystem to an other computer

#### 1.3.2 Assumptions to understand this Document
* The VisionSystem is connecte over ethernet with the Programming System
* The PS should read Neural Networks from a usb device and send these networks and the associated config files to the VS
* The http server should run completly independend from VS
(own process)
* Neural Networks can be 100Mb-1Gb in file size
* There will be only one client at the time for the http server
* The requests are synchronous. The issuing computer (PS) does not send a second request before the response of the first request is arrived.

### 1.4 Out of Scope

### 1.5 Open Questions
1. Should we transfer .engine files, too?
2. We also need to be able to transfer int8 calibration images?
3. Should we zip every thing ( 3 networks and 3 engines and config files)?
4. Which libraries should we use for http server?
5. Do we need to check if enough disk space is available on the jetson?
6. Should we also transfer the other config files (camera configs etc.)
7. Do we need a way to get back to the old networks? Like a fail safe mechanism?


### 1.6 Reference to other Documents
* [VS specification](spec.md)


## 2. Overall Description

### 2.1 Purpose of the desired System

The http server should be able to reveive neuronal networks (.onnx) and config files (.json). The .onnx files contain the stucture and the weights of the network. The .json file contain
Furthermore the http server should be able to access the log files of the VS, read it and then send it to the PS.
All of this is done via http because the current communication works with Protobuf which is not capable of transfering big files.

### 2.2 Product Functions
* transfer .onnx file for depth estimation
* transfer .onnx file for object detection
* transfer .onnx file for keyPoint detection
* transfer .json file for networks configuration
* able to check if network is already available on VS by computing a checksum (sha1sum) in order to avoid unnecessary file transfers
* The configs are format agnostic therefore we need to encode the format-Id into the URL in order to save them in the correct folder
  
### 2.3 Components

A http communication starts with a request and a following response.
The header of the request contains an URL. The body contains the files (onnx/json).

The IP-address should be the same like for the protobuf communication. You can find it in the VS specification or in the sharepoint documentation.
The port number for the http server should be 55000.

Http version should be 1.1?

#### 2.3.1 URLs
URLs should have the form:

/format/\<formatId>/networksConfig.json <br/>
/format/\<formatId>/networksConfig.sha1 <br/>
/resources/networks/\<customerName>/\<networkName>.onnx <br />
/resources/networks/\<customerName>/\<networkName>.sha1 <br />
/getLogs <br />
/getAllNetworkPaths <br />
/getAllNetworkConfigPaths <br />

#### 2.3.2 Status codes
The standard http status codes should be used.
In case of an error a detailed description should be written in the response body as a string/text. 

### 2.3 Operating Environment
The http server should be independent from OS and hardware but
needs to run on a Nvidia Jetson AGX Xavier.

## 3. Requirements

### 3.1 Networks
The neural networks should be uploaded from the PS to VS.
For uploading the onnx files to the Vs the http method 'POST' should be used.
We also need a possibility to delete a network in order to free disk space.
This shoud be done with the http method 'DELETE'.

### 3.2 Networks configs
The configs files for the neural networks should be uploaded from the PS to VS.
For uploading the json files to the Vs the http method 'POST' should be used.
We also need a possibility to delete a network in order to free disk space.
This shoud be done with the http method 'DELETE'.

### 3.3 VS Logs
The log files of the VS should be downloadable from VS to PS.
For downloading the logs the http method (verb) 'GET' should be used.
The logs are a simple text file.

### 3.4 Checksums of big files
To prevent unnecessary file transfer a checksum should be passed first. The checksum should be sha1sum.
For downloading the checksums the http method (verb) 'GET' should be used. The checksum is a text file.
Furthermore the checksum is important because the PS can directly check if a file exists for visualization purposes. 

### 3.5 Get all network paths
To be able to delete networks we first need to know which networks are stored on the jetson.
For downloading the network paths the http method (verb) 'GET' should be used. The network paths is a text file. 
In each line is an other path to an .onnx file.

### 3.6 Get all network config paths
To be able to delete networks config files we first need to know which networks are stored on the jetson.
For downloading the network config paths the http method (verb) 'GET' should be used. The network paths is a text file. 
In each line is an other path to an .json file.

## 4. Usage
http server executable is located in `VisionSystem/build/bin`.
It can be started with:
`./httpCommunication <ip4-Adresse> <portNumber>`
z.B.:
`./httpCommunication 127.0.0.1 55000`
 