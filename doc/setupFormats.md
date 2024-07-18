# Setup Format json files

Templates of configuring format are stored in `VisionSystem/format/0` folder. Every `.json` describe a use case. There are 4 use cases that can be used: 

1-`picking products`

2-`placing products in detected and tracked trays`

3-`classification of further features of already picked product with an extra camera`

4-`calibration of cameras`

Every `.json` starts with a general description of the application and the name of the creator of the file. This is usefull to understand what is the `.json` describing and a best practice to always write this description.

`Notice 1`: the `.json` files are written as structures and lists. It is a best practice playing with these files within `Visual Code` or `Visual Studio` or `Notepad`. You can access to the parameters as foldable elements and not as infinite list of values.


## Picking products

In `VisionSystem/format/0` there are two `.json` files describing how to do the configuration for the aiming purpose. These files have the name `networksConfigArea1.json` and `networksConfigArea2.json`. Those both files can be configured almost the same, because they have the same use case. There are a little bit difference on the values of special parameters. These differences are going to be discussed later.

Each of those two `.json` are describing an area on which the robot can pick products. Because the robot control can only pick from two areas (LTS1 and LTS2), that is why there are only `networksConfigArea1.json` and `networksConfigArea2.json`.  `networksConfigArea1.json` is always combined with `LTS1` of the robot and `networksConfigArea2.json` can only be combined with `LTS2` of the robot. 

`Notice 1:` You can not configure `networksConfigArea1.json` with `LTS2` or `networksConfigArea2.json` with `LTS2`.

`Notice 2:` You can simultaneously configure both files in a format, for the use case for example that the robot is going to pick products from two different areas.

`Notice 3:` You can not change the name of a file for example `networksConfigArea1.json` to `networksConfigArea3.json` or to `networksConfig1.json`. It will not work because of parsing the adequate work.

`Notice 4:` At the cobot team we have the habit that we use `networksConfigArea1.json` for picking from the `right side` of the robot and `networksConfigArea2.json` from the `left side` of the robot.

`Notice 5:` There are parameters in the templates `networksConfigArea1.json` and `networksConfigArea2.json` should not be changed. These are the differences between both files as described above. The differences are going to be discussed on the next chapter.

### Differences between `networksConfigArea1.json` and `networksConfigArea2.json`

Starting with an example. In both files there is at the top a parameter called `areaNumber`. This parameter describes on which area the robot should pick a product (at the moment of reading this documentation and further also to place a product in a tray). As we said above the robot control have `LTS1` and `LTS2`. In `networksConfigArea1.json` the `areaNumber` should always be equal to `1` and means that robot is going to pick from `LTS1` and in `networksConfigArea2.json` the `areaNumber` should always be equal to `2` and means that robot is going to pick from `LTS2`. As noticed above we must not write in `networksConfigArea1.json` the `areaNumber = 2`. This is not going to work. 

As `areaNumber` there is a structure of parameters called `udp_configuration` which parameters values are different between `networksConfigArea1.json` and `networksConfigArea2.json`. First of all we should describe this parameters structure. This structure is responsible of configuring the communication between `VisionSystem` and `TC3` for `hardware-triggering` the cameras mounted above the picking areas. The communication is based on Udp-protocoll (see [link](https://en.wikipedia.org/wiki/User_Datagram_Protocol)). The structure is composed of those parameters:

1-`udp_enabled`: That is `bool` value and it can take `true` or `false`. `true` means, we want to hardware trigger the camera mounted above the picking area via `TC3`. This is needed for `real-time` purposes. When dealing with a real application on a real robot the value should be always set to `true`. For debug reasons we want sometimes to access the camera in a `software-triggering-mode` to see for example the detection quality whithout running the real application, we can set the value to `false`.

2-`udp_twinCatIP`: always take the value: "192.168.53.89". This value should never be changed. Because it says that the `TC3` computer has this ip-adress. It will be always the case. This is still in the parameter-list to enable a developer to make some tests with different ip-adresses if needed. But the end user must not change this value.

3-`udp_twinCatPort`: it takes always the value of `53004` in `networksConfigArea1.json` and the value of `53005` in `networksConfigArea2.json`. As above it should not be configured.

4-`udp_visionSystemIP`: always take the value: "192.168.53.120". This value should never be changed. Because it says that the `Vision System` computer has this ip-adress. It will be always the case. This is still in the parameter-list to enable a developer to make some tests with different ip-adresses if needed. But the end user must not change this value.

5-`udp_visionSystemPort`: it takes always the value of `53004` in `networksConfigArea1.json` and the value of `53005` in `networksConfigArea2.json`. As above it should not be configured.

Let's now drive to the other parameters, which can be configured from the user the same way for `networksConfigArea1.json` or `networksConfigArea2.json`. These parameters demand a process and apllication understanding. You can not configure at the first time the best application. Keep trying and enjoy your journey.

### Parameters that can be configured on the same way for both `networksConfigArea1.json` and `networksConfigArea2.json`

#### Camera configuration

Before starting any format, we need to configure the cameras that are going to be used. There is a structure called `cameraConfigs` in the `.json` which is responsible for this configuration. The structure is composed of:

1-`_type`: this is a list of supported cameras (as a comment) to help the user to choose the right camera:

1.1-`MockCamera`: it is a simulation camera, which is going to use already saved images and to process the defined detection pipeline. It is only used for development-tests and can not be used in a production environment.
    
1.2-`creStereoMock`: like in `MockCamera` it is a simulation stereo camera, which is going to be used with already saved images (left and right).
    
1.3-`creStereoBaumer`: Stereo camera of type `Baumer`. It uses the `Baumer-api` to access images from the real `baumer-hardware` mounted on the robot. Why `Stereo`? To have a `3D-Point cloud` of the scene. This is needed when the robot picks products which are in `Haufen` or from a `Box with different layers`. This is needed to estimate the `plane` where the robot is going to touch a product.
    
1.4-`creStereoGV5040FA`: like in `creStereoBaumer` expressed earlier this is a Stereo camera of type `IDS`. It uses the `IDS-api` to acess images from the real `IDS-Hardware`.
    
1.5-`BaumerCam`: this a simple mono-camera of type `Baumer`. This is used when a `3D-reconstruction` is not needed and the robot is going to pick products only in `one layer` with already known `depth`.
    
1.6-`GV5040FA`: As in `BaumeraCam` it is a simple mono-camera of type `IDS` which uses only the `IDS-api` to access images from `IDS-Hardware`.

2-`camera`: it is a list, which is not correct and should be corrected on the next updates. We only use the first element of this list. Looking at the first element of this list, it is composed of different parameters, that are going to be discussed here:
    
2.1-`polygonPoints`: a list of points in `image-frame` (that means `u` and `v` coordinates), which defines the area where the camera is going to process the image. If for example an image is bigger than the area defined within the polygonPoints, the rest of the image outside the polygonPoints are going to be drawn black. Normally we use only `4` points to define the area. If needed you can use more than `4` points, but please keep in mind that you can only define a `convex-area`. Every point has 3 values:

2.1.1-`id`: a unique id of each point.
        
2.1.2-`u`: the `u` coordinate of the point.
        
2.1.3-`v`: the `v` coordinate of the point.
    
2.2-`exposureTime`: The exposure time, respectively period of exposure is generally understood to mean the time span for which the film of a traditional camera or a sensor of a modern digital camera is actually exposed to the light so as to record a picture. The exposure time is given in micro-seconds. We generally use as exposure time the value `3000`, but maybe you are going to increase this value if the image is dark or decrease it if the image bright.
    
2.3-`binningEnabled`: There is often a focus on getting a suitable pixel size depending on the experiment, application. An easy way to change sensor pixel size is to combine pixels into larger superpixels, also known as binning. It is rarely that this parameter is set to `true`. But in the Vision System, we use a `2×2 bin`, `a square of 2×2 (4) pixels` are read out by the camera as if they were a `single pixel`.
    
2.4-`serialNumbers`: this is a list of serial numbers of the cameras mounted on a picking area. If we use only a `mono-camera` the list contains only `one element` and if it is a `stereo-camera` the list should contains `two different` serialnumbers, each one is unique, and should be read from the sensor box.
    
2.5-`type`: here we can choose the type of the camera that we are going to use. As reminder we can choose between those cameras: MockCamera, creStereoMock, creStereoBaumer, creStereoGV5040FA, BaumerCam, GV5040FA. `Notice`: a `mono-camera` can not be used when there are written `two serialnumbers` and a `stereo-camera` can not be used when there is only `one serialnumber`.
    
2.6-`triggerMode`: `Off` when we want to continiuous run the image acquisation for debugging purposes or `Hardware` for the real application. Choosing `Off` in a real application is going to lead to not picking the products (bad movement or not moving to the product). This is because the robot is not receiving the information when the image was taken.
    
2.7-`cropX` and `cropY`: The word Crop can be defined as to trim or cut back. The Crop tool in most image processing programs is used to trim off the outside edges of a digital image. Cropping can be used to make an image smaller (in pixels) and/or to change the aspect ration (length to width) of the image. Because the image used by the `object detector` of the Vision System is `1024x1024` but the image from a camera for example `Baumer` is `1440x1080` we need to set values here to have the must of an image for the detection process. 
    
2.8-`cropWidth` and `cropHeight`: always at the moment `1024x1024`. That is the image resolution that is going to be used within the `detection-process`.
    
2.9-`color`: always `false` because we are always using `mono-chrome` cameras.
    
2.10-`mockImgsPath`: path were the simulation images are stored and are going to be used with `MockCamera` or with `creStereoMock`. 
    
2.11-`reverseX` and `reverseY`: flip the camera image around the `x-axis` and respectively around the `y-axis`. `Notice`: you can not set one of these parameters to `true` if you did not do the calibration of the camera with the same setting.
    
2.12-`stereoDepth`: here we configure the stereodepth-estimation if we use a stereo-camera. It is a structure of parameters. Most of the parameters are used as a developer and we should not focus on them. Developer stuffs will be marked as `Dev` in the following explanation. The parameters and their meaning are listed below:
        
2.12.1-`mappingConfigPath`: here should we write the path where the `stereo-mapping` after the `stereo-calibration` is stored. Every camera-pair has its own mapping.
        
2.12.2-`inputLayerName0`: `Dev`. Naming the first input layer of the neural network. Only an expert can change this.
        
2.12.3-`inputLayerName1`: `Dev`. Naming the second input layer of the neural network. Only an expert can change this.
        
2.12.4-`outputLayerName1`: `Dev`. Naming the output layer of the neural network. Only an expert can change this.
        
2.12.5-`dataType`: `Dev`. Resoultion of the neural network weights. We use always networks with `Float 16-mode` to have a balance between accuracy and performance. Only an expert can change this. 
        
2.12.6-`calibImgsPath0`: `Dev`. Left images stored for making the calibration of the neural network when `INT8` in `dataType` is used. Only an expert can change this. Do not get confused between `stereocalibration` and `neural network-calibration`.
        
2.12.7-`calibImgsPath1`: `Dev`. Right images stored for making the calibration of the neural network when `INT8` in `dataType` is used. Only an expert can change this. Do not get confused between `stereocalibration` and `neural network-calibration`.
        
2.12.8-`onnxPath`: Here is stored the neural network which is going to do the depth estimation giving two images from a stereo-camera-pair. The neural network may change from the camera objectiv and the distance-range where the estimation should be done.
  
#### Stechtest

Here we need to differentiate two things. In `networksConfigArea1.json` or in `networksConfigArea2.json` we can do either `Stechtest` or `picking`. `Stechtest` is used for determining the accuracy of our mounted camera. These are the parameters listed on the structure `stechtestConfigs` that should be configured if the user is going to do a `stechtest`.

1-`active`: If set to `true` that means the format is going to do a `stechtest`. 

2-`filterByArea`: Extracted blobs have an area between minArea (inclusive) and maxArea (exclusive). Always set it to `true`.

3-`minArea`: The minimal area of a detected circle. Always set it to `100`.

4-`filterByCircularity`: Extracted blobs have circularity ( 4*PI*Areaperimeter*perimeter) between minCircularity (inclusive) and maxCircularity (exclusive). Always set it to `true` to ensure that the detector is going to detect only circles or ellipses.

5-`minCircularity`: always set it to `0.9` to always ensure that the detector is going to detect only circles or ellipses.

6-`filterByConvexity`: Extracted blobs have convexity (area / area of blob convex hull) between minConvexity (inclusive) and maxConvexity (exclusive). Always set it to `true`.

7-`minConvexity`: always set it to `0.2`.

8-`filterByInertia`: Extracted blobs have this ratio between minInertiaRatio (inclusive) and maxInertiaRatio (exclusive). Always set it to `true`.

9-`minInertiaRatio`: always set it to `0.01`.

10-`distanceToStechBoard`: The distance in m between the `robot coordinate system` and the `board` where the stechtest is going to be executed.

`Notice`: If the stechtest is activated, we do not need to look further into the parameters of `application`, as described in the file `networksConfigArea1_Template_Stechtest.json`. When using this template, please change it in a new `format-number` with the name: `networksConfigArea1.json`. Here it is a `MockCamera` in use, please configure your camera, as you need it.


#### appConfigs

First of all let us explain more general informations about the features implemented in the Vision System. Products can be detected and classified with the so called [`yolo neural network`](https://docs.ultralytics.com/). Within a detected and classified product we can extract unique feature with another neural network called [`DLC`](https://www.mackenziemathislab.org/deeplabcut#:~:text=DeepLabCut%E2%84%A2%20is%20an%20efficient,typically%2050%2D200%20frames).). These features are going to be defined as unique points on the texture of a product, the so called `keypoints`. With these `keypoints` we can extract usefull informations about the product as `orientation` in the scene. After a product was detected and its keypoints were detected, we need to estimate the pose of the product in the camera coordinate system. This methode is called `poseEstimator`. We can also within the gained informations do some usefull stuffs like `check the quality of a product` by measuring different distances in the product. That is why the configuration of the picking mode contains 5 bigger structures which are:

1-`generalConfigs`:
    
1.1-`pickMode`: here we can choose between `tray`, `conveyor` or `ipaDetection`. The latest one is not going to be discussed, because the algorithm was written only for research manner.
        
1.1.1-`tray`: here products are going to be presented to the robot in one layer inside a tray. The orientation of products are not important and the centerpoint where a product should be picked will be always in the middle of the bounding box surrounding the product. For that reason `keypointsDetection` configuration is not relevant. A sample of such a configuration can be found in the file `networksConfigArea1_Template_PickTray.json`. When using this template, please change it in a new `format-number` with the name: `networksConfigArea1.json`. Here it is a `MockCamera` in use, please configure your camera, as you need it, if you want to work with real camera.
        
1.1.2-`conveyor`: here products are going to be presented to the robot in one layer or in in chaos. But the difference to `tray` is that the orientation of products are relevant and the centerpoint of a product can be configured. As shown later in the parameter `trayArea` the image area is going to be processed at all and not coupled into two spaces.
    
1.2-`trayArea`: Specifying a smaller area inside the image area, where the products should be sent first. This is done to ensure that the robot is always picking from the end of the belt. If there is no product inside this area the belt should move a little bit until the product is reaching the end of the belt.
    
1.3-`maxAmountOfProductsPerImg`: Here you can specify the number of products that can be detected. In a normal case we can use the value of `5`. It is a good compromis between performance and detect a lot of products. In special case for example picking from a box, we need to specify write the maximum of products in a layer in box. 

2-`yolo`:
    
2.1-`generalNetworkConfigs`:
        
2.1.1-`onnxPath`: Here is stored the neural network which is going to do the object detection giving one image.
        
2.1.2-`dataType`: `Dev`. Resoultion of the neural network weights. We use always networks with `Float 16-mode` to have a balance between accuracy and performance. Only an expert can change this.
        
2.1.3-`useTrt`: `Dev`. Always set to `true`. This means that the neural network will be compressed to gain more performance.
        
2.1.4-`calibImgsPath`: `Dev`. Left images stored for making the calibration of the neural network when `INT8` in `dataType` is used. Only an expert can change this.
        
2.1.5-`inputLayerName`: Naming the input layer of the neural network. Using `yoloV5` should be set to `images` and using `yoloV8` should be set to `images`.
        
2.1.6-`outputLayerName`: Naming the output layer of the neural network. Using `yoloV5` should be set to `output` and using `yoloV8` should be set to `output0`.
        
2.1.7-`whichYoloToUse`: We can use here `yoloV5` or `YoloV8`. In general cases we use `yoloV5`.
        
2.1.8-`preprocessing`:
            
2.1.8.1-`color`: When using a `mono-chrome` camera set it to `false`.
    
2.2-`detectionConfigs`:
        
2.2.1-`confThr`: The confidence determines how certain the model is that the prediction received matches to a certain class. the threshold determines what the threshold for labeling something as something should be. lets say you have a confidence threshold of 0.6, which means the model will have to be at least 60% sure the object you're trying to classify is that object before it'll label it. Around `0.8` will be a good value.
        
2.2.2-`nmsThr`: As the name suggests, NMS means suppress the ones that are not maximum (score). We are eliminating predicted bounding boxes overlapping with the highest score bounding box. A value of `0.4` will be a good value.

2.2.3-`filterBySize`: sometimes we have small products to pick, for example `bottle in a tray`. When we have dirt on the belt, the `yolo` network is going to mess with those dirt and detect them as products. Because these dirt have a size smaller than the object, we can do a `filter by size of the bounding box` and eliminate those dirt from detection.

2.2.3.1-`enable`: `True`, means this feature is going to be enabled.

2.2.3.2-`widthMin`: the minimal width of detectable object. Every object that have a width less than this parameter is going to be ignored.

2.2.3.3-`heightMin`: the minimal height of detectable object. Every object that have a height less than this parameter is going to be ignored.

2.2.3.4-`widthMax`: the maximal width of detectable object. Every object that have a width greater than this parameter is going to be ignored.

2.2.3.5-`heightMax`: the maximal height of detectable object. Every object that have a height greater than this parameter is going to be ignored.
         
2.2.4-`classPredictionOn`: `false` means that all detected object are going to be sent as class `0`.
        
2.2.5-`desiredClasses`: If the parameter `classPredictionOn = true`, we can here choose which classes can be sent to the robot.
        
2.2.6-`desiredClassesConstZ`:
            
2.2.6.1-`enable`: For mounted `monocamera` to estimate the position of the tray we need to set the z-value of the tray on the belt. This parameter is always set to `true`.
            
2.2.6.2-`classToValuesInMeterMapping`: Mapping between `classes` and the `z-value` in `meter`.
        
2.2.7-`classToDLCIndexMapping`: As in the last parameter, each `class` is going to use different `DLC` for `Keypoints-Detection`. That is why we need to do mapping between `classes-Ids` and `DLC-Ids`.

2.2.8-`cropOffset`: Generally `10` and that means that the bounding box of a product will enlarged to prevent from inaccuracy of the object detection.

2.2.9-`sortingDirection`: here `u` or `v` the coordinate system of an image, where we need to prioritize the products to be picked.

2.2.10-`sortingRelationOperator`: here `<` or `>` the direction in the axis where we need to prioritize the products to be picked.

3-`dlcs`: This is a list of different `DLC-networks`. Each element on this list is representing one `DLC-network` for a unique class defined within the mapping discussed above on the parameter `classToDLCIndexMapping`. The element of the structure of one `DLC-network` are discussed below:
    
3.1-`generalNetworkConfigs`:
        
3.1.1-`onnxFilePath`: Here is stored the neural network which is going to do the keypoints-detection giving one image.
        
3.1.2-`calibImgsPath`: `Dev`. Images stored for making the calibration of the neural network when `INT8` in `dataType` is used. Only an expert can change this.
        
3.1.3-`dataType`: `Dev`. Resoultion of the neural network weights. We use always networks with `Float 16-mode` to have a balance between accuracy and performance. Only an expert can change this.    
        
3.1.4-`useTrt`: `Dev`. Always set to `true`. This means that the neural network will be compressed to gain more performance.
        
3.1.5-`inputLayerName`: `Dev`. Naming the input layer of the neural network. Always set to `Placeholder:0`. 
        
3.1.6-`outputProbsName`: `Dev`. Naming the first output layer of the neural network. Always set to `Sigmoid:0`. 
        
3.1.7-`outputLocName`: `Dev`. Naming the second output layer of the neural network. Always set to `pose/locref_pred/block4/BiasAdd:0`. 
        
3.1.8-`constAspectRatioCrop`: `Dev`. Always set it to `true` and means even the product size is less than `256x256Pixels` the aspect ratio of the product should be preserved when resizing the bounding box of the tray to `256x256Pixel`, which the resolution of DLC.
    
3.2-`detectionConfigs`:
        
3.2.1-`confThr`: The confidence determines how certain the model is that the prediction received matches. the threshold determines what the threshold for labeling something as something should be. lets say you have a confidence threshold of 0.6, which means the model will have to be at least 60% sure the point you're trying to detect is that point before it'll label it. Around `0.8` will be a good value.

3.2.2-`keyPointsWithHighProbThr`: If the number of detected keypoints, which have confidence greater than `confThr`, is greater than `keyPointsWithHighProbThr`, then the product should be passed to further processing. If not then, the product should ignored.
        
3.2.3-`shrinkKeyPointsToCenterEnable`: `true` means that the keypoints are going to be shrinked to the center depending on the next parameter `shrinkKeyPointsToCenterVal`. This is usefull when we have keypoints on edges of product and we want to determine their depths. Depth estimation on edges is not stable.
        
3.2.4-`shrinkKeyPointsToCenterVal`: can have value between `0.0` and `1.0`. `0.0` keypoints are going to be placed exactly at the center. `1.0` means however that the points are not going to be touched.
    
3.2-`poseEstimationConfigs`:
        
3.2.1-`centerpointConfigs`:
            
3.2.1.1-`centerpoint`: `fromKeyPoint` or `fromRect`. `fromRect` means that the centerpoint of the product will be the center of the bounding box from `yolo` (our object detector). `fromKeyPoint` will be the center of keypoints written in the next parameter `centerpointIdxs`.
            
3.2.1.2-`centerpointIdxs`: A list containing the indexes of keypoints that are going to be used to determine the centerpoint. If it is only one index, the centerpoint will be this choosen keypoint. If are here two points defined, the centerpoint will be the middle of the line defined between those two points. If are more than 2 points defined, the centerpoint will be the center of mass of these points. If this list is empty and in the previous parameter `centerpoint` is equal to `fromKeyPoint` an error will occur.
        
3.2.2-`planeEstimationConfigs`:  
            
3.2.2.1-`keypointIndicesMaskForDepth`: A list containing the indexes of keypoints that are going to be used to determine the plane of the picking. If `centerpoint` is equal to `fromRect` or `centerpoint` is equal to `fromKeyPoint` and there is only one point defined in this list, the plane will be estimated around `10Pixel` of the `pcikpoint` or in the second case around the defined point.
            
3.2.2.2-`loopOffsetDepthPoints`: How many points can be used for plane estimation. `=1` all points, `=2` half of points...

3.2.2.3-`useConvexHull`: Generally `true` to calculate the plane inside a mask. If within the mask there is a hole, we need to it to `false` to calculate the plane inside a `polylines` defined with the points in `keypointIndicesMaskForDepth`.

3.2.2.4-`maxDepth`: filter z outliers that are greater than this value.

3.2.2.5-`minDepth`: filter z outliers that are less than this value.

3.2.2.6-`layers`: When we need a const depth for products but our depth estimation is not good we can use these parameters:

3.2.2.6.1-`useLayers`: the layers approximation is set to true.

3.2.2.6.2-`layersValues`: every z coordinate of a product in robot coordinate system with a z value in the region `layersValues` +/- the next parameter which is `layersResolution` is going to be assigned to `layersValues`.

3.2.2.6.3-`layersResolution`: see the last parameter `layersValues`.
        
3.2.3-`orientationZConfigs`:
            
3.2.3.1-`keypointIndicesXAxis`: We must define two indexes of keypoints. The `x` axis of the product will start from the first point and ends in the second point.
            
3.2.3.2-`longShortConfigs`: With homogenous products we have difficulties determining the `x`-axis of a product, because the keypoints-detection is not stable. For that case we defined a special work for dealing with those kind of product. The points detected will always have a `L-Form` as shown on the following figures. If we select in the following parameters `idxOfKeyPointForAxis0` and  `idxOfKeyPointForAxis1` the two indexes `0` and `4`, will the `x`-axis of the product not consistent. That is why we use the following parametrization to deal with this problem. We assume at first that the `x`-axis is the long axis, we measure the distance between those two defined points. If the distance is less than a threshold than it means that axis should be flipped to `y`-axis.
                
3.2.3.2.1-`longShortEdgePoseEstimationEnable`: `true` means that the `longShortEdge`-estimation should be processed.
                
3.2.3.2.2-`longShortEdgeLengthInMeterThr`: the measured distance is less than this value than the axis linking the two defined points is the `y`-axis, else is the `x`-axis. The value is in `meter`.
                
3.2.3.2.3-`idxOfKeyPointForAxis0`: Index of the first keypoint.
                
3.2.3.2.4-`idxOfKeyPointForAxis1`:  Index of the second keypoint.
            
3.2.3.3-`keypointIndicesXAxisDoubleCheck`: A list of pair keypoints to double check if the `x`-axis of a product was well determined. If we see any confusion in the estimation, means that the keypoints-detection is not stable and we muss ignore the product. Empty list means that there is no double check.
                
3.2.3.3.1-`vectorHeadIdx`: Ending index for the `x`-axis.
                
3.2.3.3.2-`vectorTailIdx`: Starting index for the `x`-axis.
            
3.2.3.4-`deltaAngleBetweenAxesThresholdInDegree`: Maximum deviation `degree` between two `x`-axes. If deviation is greater than this value, the detected product should be ignored.     
    
3.3-`qualityGateDimension`: Some products are bad products, for example for a pouch, can sometime the air inside the pouch be compressed, and the pouch will be no more usefull. For that reasons we do a quality check of the dimensions of a product.
        
3.3.1-`enable`: `true` means the quality check will be executed.
        
3.3.2-`idxs`: A list of pair points and the minimal should distance between those points.
            
3.3.2.1-`idx0`: Point 1 for the check.
            
3.3.2.2-`idx1`: Point 2 for the check.
            
3.3.2.3-`distanceThrInMeter`: The minimal distance between those two points, under this threshold the product should labeled as `bad`.
    
3.4-`posePostProcessing`: For rotation-unsymetrical products in chaos, the kinematik of the robot can not handle 6D product position, that is why we should set the following two values `fixRollAngle` and `fixPitchAngle` to `true`. Such an example is `bottles` in chaos. The third parameter  `fixYawAngle` should always be set to `false`.
            
3.4.1-`fixRollAngle`:
            
3.4.2-`fixPitchAngle`:
            
3.4.3-`fixYawAngle`: always `false`.
    
3.5-`poseEstimationIPA`: `Dev` and irrelevant because it is only used for research things.


        

    

    


## Placing products in detected and tracked trays

In `VisionSystem/format/0` there is `.json` files describing how to do the configuration for the aiming purpose. The file have the name `networksConfigAreaPlace.json`. 

This `.json` is describing an area on which the robot can place products on a detected and tracked tray. Because the robot control can place into two areas (LTS3 and LTS4) but only one at a time.

`Notice 1:` You can not configure `networksConfigAreaPlace.json` with `LTS1` or `LTS2`.

`Notice 2:` You can not configure `networksConfigAreaPlace.json` with `areaNumber=1` or `areaNumber=2`.

`Notice 3:` You can not change the name of a file for example `networksConfigAreaPlace.json` to `networksConfigAreaPlace1.json` or to `networksConfig1.json`. It will not work because of parsing the adequate work.

`Notice 4:` There are parameters in the template `networksConfigAreaPlace.json` should not be changed.

### Configuring `networksConfigAreaPlace.json` with the right area

Starting with an example. In the file there is at the top a parameter called `areaNumber`. This parameter describes on which area the robot should place a product. As we said above the robot control have `LTS3` and `LTS4`. In `networksConfigAreaPlace.json` the `areaNumber` can be equal to `3` and means that robot is going to place into `LTS3` or can be equal to `4` and means that robot is going to place into `LTS4`.

As `areaNumber` there is a structure of parameters called `udp_configuration` which parameters values are different between `AreaNumber=3` and `AreaNumber=4`. First of all we should describe this parameters structure. This structure is responsible of configuring the communication between `VisionSystem` and `TC3` for `hardware-triggering` the camera mounted above the placing area.The structure is composed of those parameters:

1-`udp_enabled`: That is `bool` value and it can take `true` or `false`. `true` means, we want to hardware trigger the camera mounted above the placing area via `TC3`. This is needed for `real-time` purposes. When dealing with a real application on a real robot the value should be always set to `true`. For debug reasons we want sometimes to access the camera in a `software-triggering-mode` to see for example the detection quality whithout running the real application, we can set the value to `false`.

2-`udp_twinCatIP`: always take the value: "192.168.53.89". This value should never be changed. Because it says that the `TC3` computer has this ip-adress. It will be always the case. This is still in the parameter-list to enable a developer to make some tests with different ip-adresses if needed. But the end user must not change this value.

3-`udp_twinCatPort`: valid values are `53006` for `areaNumber=3` and `53007` for `areaNumber=4`.

4-`udp_visionSystemIP`: always take the value: "192.168.53.120". This value should never be changed. Because it says that the `Vision System` computer has this ip-adress. It will be always the case. This is still in the parameter-list to enable a developer to make some tests with different ip-adresses if needed. But the end user must not change this value.

5-`udp_visionSystemPort`: valid values are `53006` for `areaNumber=3` and `53007` for `areaNumber=4`.

Let's now drive to the other parameters, which can be configured from the user the same way for `AreaNumber=3` or `AreaNumber=4`. These parameters demand a process and application understanding. You can not configure at the first time the best application. Keep trying and enjoy your journey.

### Parameters that can be configured on the same way for both `AreaNumber=3` and `AreaNumber=4`

#### Camera configuration

Before starting any format, we need to configure the camera that is going to be used. There is a structure called `cameraConfigs` in the `.json` which is responsible for this configuration. The structure is composed of:

1-`_type`: this is a list of supported cameras (as a comment) to help the user to choose the right camera:

1.1-`MockCamera`: it is a simulation camera, which is going to use already saved images and to process the defined detection pipeline. It is only used for development-tests and can not be used in a production environment.
       
1.2-`BaumerCam`: this a simple mono-camera of type `Baumer`.
    
1.3-`GV5040FA`: As in `BaumeraCam` it is a simple mono-camera of type `IDS` which uses only the `IDS-api` to access images from `IDS-Hardware`.

`Notice`: there is no need here for a `3D-camera`.

2-`camera`: it is a list, which is not correct and should be corrected on the next updates. We only use the first element of this list. Looking at the first element of this list, it is composed of different parameters, that are going to be discussed here:
    
2.1-`polygonPoints`: a list of points in `image-frame` (that means `u` and `v` coordinates), which defines the area where the camera is going to process the image. If for example an image is bigger than the area defined within the polygonPoints, the rest of the image outside the polygonPoints are going to be drawn black. Normally we use only `4` points to define the area. If needed you can use more than `4` points, but please keep in mind that you can only define a `convex-area`. Every point has 3 values:

2.1.1-`id`: a unique id of each point.
        
2.1.2-`u`: the `u` coordinate of the point.
        
2.1.3-`v`: the `v` coordinate of the point.
    
2.2-`exposureTime`: The exposure time, respectively period of exposure is generally understood to mean the time span for which the film of a traditional camera or a sensor of a modern digital camera is actually exposed to the light so as to record a picture. The exposure time is given in micro-seconds. We generally use as exposure time the value `3000`, but maybe you are going to increase this value if the image is dark or decrease it if the image bright.
    
2.3-`binningEnabled`: There is often a focus on getting a suitable pixel size depending on the experiment, application. An easy way to change sensor pixel size is to combine pixels into larger superpixels, also known as binning. It is rarely that this parameter is set to `true`. But in the Vision System, we use a `2×2 bin`, `a square of 2×2 (4) pixels` are read out by the camera as if they were a `single pixel`.
    
2.4-`serialNumbers`: this is a list of serial numbers of the cameras mounted on a placing area. The list contains only one element as serial number because we only use a `mono-camera`,and should be read from the sensor box.
    
2.5-`type`: here we can choose the type of the camera that we are going to use. As reminder we can choose between those cameras: MockCamera, BaumerCam, GV5040FA.
    
2.6-`triggerMode`: `Off` when we want to continiuous run the image acquisation for debugging purposes or `Hardware` for the real application. Choosing `Off` in a real application is going to lead to not placing the products (bad movement or not moving to the tray). This is because the robot is not receiving the information when the image was taken.
    
2.7-`cropX` and `cropY`: The word Crop can be defined as to trim or cut back. The Crop tool in most image processing programs is used to trim off the outside edges of a digital image. Cropping can be used to make an image smaller (in pixels) and/or to change the aspect ration (length to width) of the image. Because the image used by the `object detector` of the Vision System is `1024x1024` but the image from a camera for example `Baumer` is `1440x1080` we need to set values here to have the must of an image for the detection process. 
    
2.8-`cropWidth` and `cropHeight`: always at the moment `1024x1024`. That is the image resolution which is going to be used within the `detection-process`.
    
2.9-`color`: always `false` because we are always using `mono-chrome` cameras.
    
2.10-`mockImgsPath`: path were the simulation images are stored and are going to be used with `MockCamera`. 
    
2.11-`reverseX` and `reverseY`: flip the camera image around the `x-axis` and respectively around the `y-axis`. `Notice`: you can not set one of these parameters to `true` if you did not do the calibration of the camera with the same setting.
  
#### appConfigs

First of all let us explain more general informations about the features implemented in the Vision System. Trays can be detected with the so called [`yolo neural network`](https://docs.ultralytics.com/). Within a detected tray we can extract unique feature with another neural network called [`DLC`](https://www.mackenziemathislab.org/deeplabcut#:~:text=DeepLabCut%E2%84%A2%20is%20an%20efficient,typically%2050%2D200%20frames).). These features are going to be defined as unique points on the texture of a tray, the so called `keypoints`. With these `keypoints` we can extract usefull informations about the tray as `orientation` in the scene. After a tray was detected and its keypoints were detected, we need to estimate the pose of the tray in the camera coordinate system. This methode is called `poseEstimator`. We can also within the gained informations do some usefull stuffs like `check the quality of a tray` by measuring different distances in the tray. The detected tray should also be `tracked` in the next images and always give it a unique ID, this is to always keep the full quantity of a tray consistent with the logic inside the robot.

2-`yolo`:
    
2.1-`generalNetworkConfigs`:
        
2.1.1-`onnxPath`: Here is stored the neural network which is going to do the object detection giving one image.
        
2.1.2-`dataType`: `Dev`. Resoultion of the neural network weights. We use always networks with `Float 16-mode` to have a balance between accuracy and performance. Only an expert can change this.
        
2.1.3-`useTrt`: `Dev`. Always set to `true`. This means that the neural network will be compressed to gain more performance.
        
2.1.4-`calibImgsPath`: `Dev`. Images stored for making the calibration of the neural network when `INT8` in `dataType` is used. Only an expert can change this.
        
2.1.5-`inputLayerName`: Naming the input layer of the neural network. Using `yoloV5` should be set to `images` and using `yoloV8` should be set to `images`.
        
2.1.6-`outputLayerName`: Naming the output layer of the neural network. Using `yoloV5` should be set to `output` and using `yoloV8` should be set to `output0`.
        
2.1.7-`whichYoloToUse`: We can use here `yoloV5` or `YoloV8`. In general cases we use `yoloV5`.
        
2.1.8-`preprocessing`:
            
2.1.8.1-`color`: When using a `mono-chrome` camera set it to `false`.
    
2.2-`detectionConfigs`:
        
2.2.1-`confThr`: The confidence determines how certain the model is that the prediction received matches to a certain class. the threshold determines what the threshold for labeling something as something should be. lets say you have a confidence threshold of 0.6, which means the model will have to be at least 60% sure the object you're trying to classify is that object before it'll label it. Around `0.8` will be a good value.
        
2.2.2-`nmsThr`: As the name suggests, NMS means suppress the ones that are not maximum (score). We are eliminating predicted bounding boxes overlapping with the highest score bounding box. A value of `0.4` will be a good value.

2.2.3-`filterBySize`: sometimes we have small tray where to place. When we have dirt on the belt, the `yolo` network is going to mess with those dirt and detect them as trays. Because these dirt have a size smaller than the object, we can do a `filter by size of the bounding box` and eliminate those dirt from detection.

2.2.3.1-`enable`: `True`, means this feature is going to be enabled.

2.2.3.2-`widthMin`: the minimal width of detectable object. Every object that have a width less than this parameter is going to be ignored.

2.2.3.3-`heightMin`: the minimal height of detectable object. Every object that have a height less than this parameter is going to be ignored.

2.2.3.4-`widthMax`: the maximal width of detectable object. Every object that have a width greater than this parameter is going to be ignored.

2.2.3.5-`heightMax`: the maximal height of detectable object. Every object that have a height greater than this parameter is going to be ignored.
                
2.2.4-`classPredictionOn`: `false` means that all detected object are going to be sent as class `0`.
        
2.2.5-`desiredClasses`: If the parameter `classPredictionOn = true`, we can here choose which classes can be sent to the robot.
        
2.2.6-`desiredClassesConstZ`:
            
2.2.6.1-`enable`: For mounted `monocamera` to estimate the position of the product we need to set the z-value of the product on the belt, by setting this parameter to `true`.
            
2.2.6.2-`classToValuesInMeterMapping`: For example, a 3D-Product like `cover` of a `shampoo-bottle` has 3 different faces. We need with `yolo` to classify these 3 faces. Each face will have a unique ID. However the `z-value` of the picking point is different from each face. That is why we need to do the mapping between `classes` and the `z-value`.
        
2.2.7-`classToDLCIndexMapping`: As in the last parameter, each `class` (representing the faces of the cover) is going to use different `DLC` for `Keypoints-Detection`. That is why we need to do mapping between `classes-Ids` and `DLC-Ids`.

2.2.8-`cropOffset`: Generally `10` and that means that the bounding box of a product will enlarged to prevent from inaccuracy of the object detection.

2.2.9-`sortingDirection`: For placing products in a tray this parameter is irrelevant.

2.2.10-`sortingRelationOperator`: For placing products in a tray this parameter is irrelevant.

3-`dlcs`: This is a list of different `DLC-networks`. Each element on this list is representing one `DLC-network` for a unique class defined within the mapping discussed above on the parameter `classToDLCIndexMapping`. The element of the structure of one `DLC-network` are discussed below:
    
3.1-`generalNetworkConfigs`:
        
3.1.1-`onnxFilePath`: Here is stored the neural network which is going to do the keypoints-detection giving one image.
        
3.1.2-`calibImgsPath`: `Dev`. Left images stored for making the calibration of the neural network when `INT8` in `dataType` is used. Only an expert can change this.
        
3.1.3-`dataType`: `Dev`. Resoultion of the neural network weights. We use always networks with `Float 16-mode` to have a balance between accuracy and performance. Only an expert can change this.    
        
3.1.4-`useTrt`: `Dev`. Always set to `true`. This means that the neural network will be compressed to gain more performance.
        
3.1.5-`inputLayerName`: `Dev`. Naming the input layer of the neural network. Always set to `Placeholder:0`. 
        
3.1.6-`outputProbsName`: `Dev`. Naming the first output layer of the neural network. Always set to `Sigmoid:0`. 
        
3.1.7-`outputLocName`: `Dev`. Naming the second output layer of the neural network. Always set to `pose/locref_pred/block4/BiasAdd:0`. 
        
3.1.8-`constAspectRatioCrop`: `Dev`. Always set it to `true` and means even the product size is less than `256x256Pixels` the aspect ratio of the product should be preserved when resizing the bounding box of the product to `256x256Pixel`, which the resolution of DLC.
    
3.2-`detectionConfigs`:
        
3.2.1-`confThr`: The confidence determines how certain the model is that the prediction received matches. the threshold determines what the threshold for labeling something as something should be. lets say you have a confidence threshold of 0.6, which means the model will have to be at least 60% sure the point you're trying to detect is that point before it'll label it. Around `0.8` will be a good value.

3.2.2-`keyPointsWithHighProbThr`: If the number of detected keypoints, which have confidence greater than `confThr`, is greater than `keyPointsWithHighProbThr`, then the tray should be passed to further processing. If not then, the tray should ignored.
        
3.2.3-`shrinkKeyPointsToCenterEnable`: `true` means that the keypoints are going to be shrinked to the center depending on the next parameter `shrinkKeyPointsToCenterVal`. Always set it `false`. It is not usefull for placing application
        
3.2.4-`shrinkKeyPointsToCenterVal`: can have value between `0.0` and `1.0`. `0.0` keypoints are going to be placed exactly at the center. `1.0` means however that the points are not going to be touched.
    
3.2-`poseEstimationConfigs`:
        
3.2.1-`centerpointConfigs`:
            
3.2.1.1-`centerpoint`: `fromKeyPoint` or `fromRect`. `fromRect` means that the centerpoint of the tray will be the center of the bounding box from `yolo` (our object detector). `fromKeyPoint` will be the center of keypoints written in the next parameter `centerpointIdxs`.
            
3.2.1.2-`centerpointIdxs`: A list containing the indexes of keypoints that are going to be used to determine the centerpoint. If it is only one index, the centerpoint will be this choosen keypoint. If are here two points defined, the centerpoint will be the middle of the line defined between those two points. If are more than 2 points defined, the centerpoint will be the center of mass of these points. If this list is empty and in the previous parameter `centerpoint` is equal to `fromKeyPoint` an error will occur.

3.2.2-`orientationZConfigs`:
            
3.2.2.1-`keypointIndicesXAxis`: We must define two indexes of keypoints. The `x` axis of the tray will start from the first point and ends in the second point.
                           
3.2.2.2-`keypointIndicesXAxisDoubleCheck`: A list of pair keypoints to double check if the `x`-axis of a tray was well determined. If we see any confusion in the estimation, means that the keypoints-detection is not stable and we muss ignore the tray. Empty list means that there is no double check.
                
3.2.2.2.1-`vectorHeadIdx`: Ending index for the `x`-axis.
                
3.2.2.2.2-`vectorTailIdx`: Starting index for the `x`-axis.
            
3.2.2.3-`deltaAngleBetweenAxesThresholdInDegree`: Maximum deviation `degree` between two `x`-axes. If deviation is greater than this value, the detected tray should be ignored.     
    
3.3-`qualityGateDimension`: Some trays are bad trays, for example a tray can sometimes be deformed, and the tray will be no more usefull. For that reasons we do a quality check of the dimensions of a tray.
        
3.3.1-`enable`: `true` means the quality check will be executed.
        
3.3.2-`idxs`: A list of pair points and the minimal should distance between those points.
            
3.3.2.1-`idx0`: Point 1 for the check.
            
3.3.2.2-`idx1`: Point 2 for the check.
            
3.3.2.3-`distanceThrInMeter`: The minimal distance between those two points, under this threshold the tray should labeled as `bad`. 

4-`tracking`: Here we are going to describe the parameters used for tracking the trays between images purposes:

4.1-`distanceThresholdHunAlgo`: This value is in `meter`, if the distance between one track and new detected tray exceeds this value, then the new object is not this track.

4.2-`nFramesExpirationThreshold`: `Dev`. Should always be `10000`.

4.3-`removeTrkIfCoordExceedsVal`: The position in `meter` along the synchron-axis, where the track should be ignored. `Notice`: this position should always be less than the `Ausschleusposition` defined on the robot `LTS3` or `LTS4`.

4.4-`conveyorVelocityDirection`: `1.0` is positive or `-1.0` negative direction.

4.5-`conveyorVelocityFeedingFactor`: `Vorschub`.

4.6-`conveyorEncoderIncrements`: Encoder-Resolution. Now generally `65535.0`.

4.7-`whichAxis`: takes the value: `xAxis` or `yAxis`. That is the direction of the synchron-axis where the trays are moving.





## Classification of further features of already picked product with an extra camera

In `VisionSystem/format/0` there is `.json` files describing how to do the configuration for the aiming purpose. The file have the name `networksConfigClassificatorExtension.json`. 

This `.json` is describing an area on which the robot can a further classification of a product in a seperate camera.

`Notice 1:` You can not configure `networksConfigClassificatorExtension.json` only with `areaNumber=7` or `areaNumber=8`.

`Notice 2:` You can not change the name of a file for example `networksConfigClassificatorExtension.json` to `networksConfigClassificatorExtension1.json` or to `networksConfigClassificatorExtension1.json`. It will not work because of parsing the adequate work.

`Notice 3:` There are parameters in the template `networksConfigClassificatorExtension.json` should not be changed.

### Configuring `networksConfigClassificatorExtension.json` with the right area

Starting with an example. In the file there is at the top a parameter called `areaNumber`. This parameter describes on which area the robot should make the further classification. In `networksConfigClassificatorExtension.json` the `areaNumber` can be equal to `7` or can be equal to `8`.

As `areaNumber` there is a structure of parameters called `udp_configuration` which parameters values are different between `AreaNumber=7` and `AreaNumber=8`. First of all we should describe this parameters structure. This structure is responsible of configuring the communication between `VisionSystem` and `TC3` for `hardware-triggering` the camera mounted above the extension.The structure is composed of those parameters:

1-`udp_enabled`: That is `bool` value and it can take `true` or `false`. `true` means, we want to hardware trigger the camera mounted above the placing area via `TC3`. This is needed for `real-time` purposes. When dealing with a real application on a real robot the value should be always set to `true`. For debug reasons we want sometimes to access the camera in a `software-triggering-mode` to see for example the detection quality whithout running the real application, we can set the value to `false`.

2-`udp_twinCatIP`: always take the value: "192.168.53.89". This value should never be changed. Because it says that the `TC3` computer has this ip-adress. It will be always the case. This is still in the parameter-list to enable a developer to make some tests with different ip-adresses if needed. But the end user must not change this value.

3-`udp_twinCatPort`: valid values are `53008` for `areaNumber=7` and `53009` for `areaNumber=8`.

4-`udp_visionSystemIP`: always take the value: "192.168.53.120". This value should never be changed. Because it says that the `Vision System` computer has this ip-adress. It will be always the case. This is still in the parameter-list to enable a developer to make some tests with different ip-adresses if needed. But the end user must not change this value.

5-`udp_visionSystemPort`: valid values are `53008` for `areaNumber=7` and `53009` for `areaNumber=8`.

Let's now drive to the other parameters, which can be configured from the user the same way for `AreaNumber=7` or `AreaNumber=8`. These parameters demand a process and application understanding. You can not configure at the first time the best application. Keep trying and enjoy your journey.

### Parameters that can be configured on the same way for both `AreaNumber=7` and `AreaNumber=8`

#### vms configuration

There is a structure called `vms_configuration`. Within this structure there is a parameter called `vms_enabled`. If this parameter is equal to `true` then the communication will occure only between `vms` and `Vision System`. Else the communication for determining the further features will work only between `TC3` and `Vision System`. This now needed, because the feature for the communication directly between `vms` and `Vision System` is still not working. That is why we need to support both.

#### Camera configuration

Before starting any format, we need to configure the camera that is going to be used. There is a structure called `cameraConfigs` in the `.json` which is responsible for this configuration. The structure is composed of:

1-`_type`: this is a list of supported cameras (as a comment) to help the user to choose the right camera:

1.1-`MockCamera`: it is a simulation camera, which is going to use already saved images and to process the defined detection pipeline. It is only used for development-tests and can not be used in a production environment.
       
1.2-`BaumerCam`: this a simple mono-camera of type `Baumer`.
    
1.3-`GV5040FA`: As in `BaumeraCam` it is a simple mono-camera of type `IDS` which uses only the `IDS-api` to access images from `IDS-Hardware`.

`Notice`: there is no need here for a `3D-camera`.

2-`camera`: it is a list, which is not correct and should be corrected on the next updates. We only use the first element of this list. Looking at the first element of this list, it is composed of different parameters, that are going to be discussed here:
    
2.1-`polygonPoints`: a list of points in `image-frame` (that means `u` and `v` coordinates), which defines the area where the camera is going to process the image. If for example an image is bigger than the area defined within the polygonPoints, the rest of the image outside the polygonPoints are going to be drawn black. Normally we use only `4` points to define the area. If needed you can use more than `4` points, but please keep in mind that you can only define a `convex-area`. Every point has 3 values:

2.1.1-`id`: a unique id of each point.
        
2.1.2-`u`: the `u` coordinate of the point.
        
2.1.3-`v`: the `v` coordinate of the point.
    
2.2-`exposureTime`: The exposure time, respectively period of exposure is generally understood to mean the time span for which the film of a traditional camera or a sensor of a modern digital camera is actually exposed to the light so as to record a picture. The exposure time is given in micro-seconds. We generally use as exposure time the value `3000`, but maybe you are going to increase this value if the image is dark or decrease it if the image bright.
    
2.3-`binningEnabled`: There is often a focus on getting a suitable pixel size depending on the experiment, application. An easy way to change sensor pixel size is to combine pixels into larger superpixels, also known as binning. It is rarely that this parameter is set to `true`. But in the Vision System, we use a `2×2 bin`, `a square of 2×2 (4) pixels` are read out by the camera as if they were a `single pixel`.
    
2.4-`serialNumbers`: this is a list of serial numbers of the cameras mounted on a classification area. The list contains only one element as serial number because we only use a `mono-camera`, and should be read from the sensor box.
    
2.5-`type`: here we can choose the type of the camera that we are going to use. As reminder we can choose between those cameras: MockCamera, BaumerCam, GV5040FA.
    
2.6-`triggerMode`: `Off` when `vms_enabled=true` or in `debug`-mode . `Hardware` when we use `TC3`.
    
2.7-`cropX` and `cropY`: The word Crop can be defined as to trim or cut back. The Crop tool in most image processing programs is used to trim off the outside edges of a digital image. Cropping can be used to make an image smaller (in pixels) and/or to change the aspect ration (length to width) of the image. Because the image used by the `object detector` of the Vision System is `1024x1024` but the image from a camera for example `Baumer` is `1440x1080` we need to set values here to have the must of an image for the detection process. 
    
2.8-`cropWidth` and `cropHeight`: always at the moment `1024x1024`. That is the image resolution which is going to be used within the `detection-process`.
    
2.9-`color`: always `false` because we are always using `mono-chrome` cameras.
    
2.10-`mockImgsPath`: path were the simulation images are stored and are going to be used with `MockCamera`. 
    
2.11-`reverseX` and `reverseY`: flip the camera image around the `x-axis` and respectively around the `y-axis`. `Notice`: you can not set one of these parameters to `true` if you did not do the calibration of the camera with the same setting.
  
#### appConfigs

First of all let us explain more general informations about the features implemented in the Vision System. Further features of a product, or a new classification of a product can be detected with the so called [`yolo neural network`](https://docs.ultralytics.com/).

2-`yolo`:
    
2.1-`generalNetworkConfigs`:
        
2.1.1-`onnxPath`: Here is stored the neural network which is going to do the object detection giving one image.
        
2.1.2-`dataType`: `Dev`. Resoultion of the neural network weights. We use always networks with `Float 16-mode` to have a balance between accuracy and performance. Only an expert can change this.
        
2.1.3-`useTrt`: `Dev`. Always set to `true`. This means that the neural network will be compressed to gain more performance.
        
2.1.4-`calibImgsPath`: `Dev`. Images stored for making the calibration of the neural network when `INT8` in `dataType` is used. Only an expert can change this.
        
2.1.5-`inputLayerName`: Naming the input layer of the neural network. Using `yoloV5` should be set to `images` and using `yoloV8` should be set to `images`.
        
2.1.6-`outputLayerName`: Naming the output layer of the neural network. Using `yoloV5` should be set to `output` and using `yoloV8` should be set to `output0`.
        
2.1.7-`whichYoloToUse`: We can use here `yoloV5` or `YoloV8`. In general cases we use `yoloV5`.
        
2.1.8-`preprocessing`:
            
2.1.8.1-`color`: When using a `mono-chrome` camera set it to `false`.
    
2.2-`detectionConfigs`:
        
2.2.1-`confThr`: The confidence determines how certain the model is that the prediction received matches to a certain class. the threshold determines what the threshold for labeling something as something should be. lets say you have a confidence threshold of 0.6, which means the model will have to be at least 60% sure the object you're trying to classify is that object before it'll label it. Around `0.8` will be a good value.
        
2.2.2-`nmsThr`: As the name suggests, NMS means suppress the ones that are not maximum (score). We are eliminating predicted bounding boxes overlapping with the highest score bounding box. A value of `0.4` will be a good value.

2.2.3-`filterBySize`: sometimes we have small products to see. When we have dirt on the belt, the `yolo` network is going to mess with those dirt and detect them as products. Because these dirt have a size smaller than the object, we can do a `filter by size of the bounding box` and eliminate those dirt from detection.

2.2.3.1-`enable`: `True`, means this feature is going to be enabled.

2.2.3.2-`widthMin`: the minimal width of detectable object. Every object that have a width less than this parameter is going to be ignored.

2.2.3.3-`heightMin`: the minimal height of detectable object. Every object that have a height less than this parameter is going to be ignored.

2.2.3.4-`widthMax`: the maximal width of detectable object. Every object that have a width greater than this parameter is going to be ignored.

2.2.3.5-`heightMax`: the maximal height of detectable object. Every object that have a height greater than this parameter is going to be ignored.
                
2.2.4-`classPredictionOn`: `true` the whole feature is used for classification.
        
2.2.5-`desiredClasses`: If the parameter `classPredictionOn = true`, we can here choose which classes can be sent to the robot.

2.2.6-`cropOffset`: Generally `0` and that means that the bounding box of a product will enlarged to prevent from inaccuracy of the object detection.
        
2.2.9-`sortingDirection`: For classification this parameter is irrelevant.

2.2.10-`sortingRelationOperator`: For classification this parameter is irrelevant.



## calibration of cameras

We do 3 types of calibration:

1-`single camera calibration`:

2- `Stereo-Calibration`:

3- `Hand-Eye-Calibration`:

A described methode for calibration can be found under [this link](CameraCalibration.md).
