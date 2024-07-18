# List of Redis Keys

## Variables used
* `system` in {0, 1, 2, ...} was designed as a way for multiple vision systems to work together, so that a unique `system` is assigned to each vision system
* (TBC) `program` in {???} is a unique id for every module/app?
* (TBC) `camera` in {???} is a unique for every camera
* (TBC) `region` in {???} is a unique id within a vision system. I don't know what crackpipe idea this was supposed to be. Is a region one of many cameras in a vision system or is it a region in a single cameras image?
* (TBC) `app` in {???} is a unique id for an app. how is this different from `program`???
* (TBC) `parameter` in {???} is a list of 
* (TBC) `constraint` in {???} wtf is this even?

Is `program` the same as `app`? If not, what's the difference?

## Global Namespace
* reloadFinishedPlaceConveyor (**string**): Used to halt `detectPickableObjects` until `verifyObjectPlaced` is up and running

## Namespace processing
* ids (**long long**): (TBC) This is the number of vision systems that are active. -> `system` in {0, 1, 2, ..., processing:ids - 1}
* `system` (**map**): wtf is this used for? This is only ever set, never read, so it's not used at all?
  * Key: processingID
  * Key: liveImage
  * Key: programID

### Namespace processing:`system`
* calibration_processing (**string**): wtf is this used for? This is only ever set, never read, so it's not used at all?
* productList (**list**): (TBC) This list stores the unique IDs of all detected objects
* productPresent (**string**): (TBC) This variable indicates whether any products were detected at all. What do we need this for, can't we deducte this from the length of the list _productList_?
* frameCounter (**string**): (TBC) this variable contains the frame number from the camera for `detectPickableObjects`
* capture_frame (**string**): wtf is this used for? This is only ever set, never read, so it's not used at all?
* caibration_values (**map**): wtf is this used for? This is only ever read, never set. How the fuck does this even work?
  * Key: error
  * Key: captured_frames

#### Namespace processing:`system`:product
* j for j in {0, 1, 2, ...} (**map**): There is a map for every detection made (individual objects could be detected multiple times). 
  * Key: pose:x
  * Key: pose:y
  * Key: pose:z
  * Key: pose:roll
  * Key: pose:pitch
  * Key: pose:yaw
  * Key: pose:encoder
  * Key: product:id
  * Key: id
  * Key: timestamp.seconds
  * Key: timestamp.nano
  * Key: priority
  * Key: error
* count (**long long**): (TBC) This is a total of all detections made by the system. It's only ever set, never read, so it's useless?

#### Namespace processing:`system`:program
* workchain (**list**): list of modules/apps used
* `program` (**map**): description of each program?
  * Key: id
  * Key: name

##### Namespace processing:`system`:program:workchain
* changed (**string**): used to signal when the workchain has changed

#### Namespace processing:`system`:workspaceMarker
* count (**string**): wtf is this used for? This is only ever read, never set. How the fuck does this even work?
* `region` (**map**): wtf is this used for? This is only ever set, never read, so it's useless?
  * Key: id
  * Key: value
  * Key: x
  * Key: y
  * Key: z

#### Namespace processing:`system`:app:`app`
* parameter (**string**): the number of parameters for given `app`
* constraintFront (**string**): wtf is this used for? wtf are constraints?
* constraintBeyond (**string**): wtf is this used for? wtf are constraints?

##### Namespace processing:`system`:app:`app`:constraintFront
* `constraint` (**map**): wtf is this used for? wtf are constraints?
  * Key: constraint
  * Key: id

##### Namespace processing:`system`:app:`app`:constraintBeyond
* `constraint` (**map**): wtf is this used for? wtf are constraints?
  * Key: constraint
  * Key: id

##### Namespace processing:`system`:app:`app`:parameter
* `parameter` (**map**): (TBC) this is a map for a given `parameter` for a given `app`. 
  * Key: name
  * Key: type
  * Key: value
  * Key: minimum
  * Key: maximum
  * Key: category
  * Key: text_id
  * Key: maximumLength

## Namespace format

### Namespace format::1
I don't know what the 1 is for, but it's hard coded
* active (**string**): (TBC) this holds the id of the active format

## Namespace version
* communication (**string**): (TBC) this holds the version of the communication system. It is only ever set, never read, so it's useless?
* `system` (**map**): 
  * Key: Version

## Namespace communication
* debug (**string**): if set, enable extra logging

## Namespace app
* 1 (**list**): wtf is this used for? It is only ever set, never read, so it's useless?

## Namespace sensor
* `camera` (**map**): (TBC) this is supposed to turn on/off a live image transmission. However, it is only ever set, not read, and live image was never implemented, so it's useless?
  * Key: liveImage

