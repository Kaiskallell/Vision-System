# Instructions to create an AzureDevOpsPipeline

## 1. Bring Jetson into the right network
The Azure DevOps Server is located in a special network (172.16.81.* or 'vlan 81' ).
Therefore, the Jetson, which is to serve as a build server, must also be placed in this network.
If the Jetson is in this network, it cannot access the internet anymore, because it is now behind the firewall. 
Therefore it is important to install everything you need there beforehand.
In vlan 81 there is also no DHCP server so you need a static IP address. There is also no DNS server so you always need to know the IP address of the server as URL. So if you try to reach "https://decrtfs/tfs/GRIPS/Cobot/" you get a "This site cant be reached. DNS_PROBE_FINNISHED_NXDOMAIN" error.
So you have to enter "https://172.16.81.11/tfs/GRIPS/Cobot/" instead.

## 2. Install/configure Azure software on the Jetson.
<center>

![](resources/getAgent.png "getAgent")

</center>
Move to:
Project Settings -> Pipelines -> Agent Pools -> Cobot -> New Agent.
You have to download the software for the corresponding architecture and install it on the Jetson.
When trying to run `config.sh` the error message appears: "The remote certificate is invalid according to the validation procedure.". Because of certificate problems (self-signing). Therefore you have to disable ssl: 
`config.sh --sslskipcertvalidation `. It's not nice but there is no other way or I don't know any other way.
You also need special permissions ( https://stackoverflow.com/a/65913781/ ). It is not enough to be admin on project level. You have to be admin on organization/collection level.
One can now set a name for the build server and must enter a PAT (Personal Access Tokens) that one has previously created in Azure DevOps. You only have to run `config.sh` once, after that you can reset the permissions.

<center>

![](resources/pat.png "pat")

</center>

After that run ``run.sh``. Now the Jetson should appear as an agent in the agent pool.

## 3. Create build pipeline
To create the build pipeline you need special permissions.
<center>

![](resources/queueBerechtigung.png "queueBerechtigung")

</center>
Necessary are: 

- Add builds to the queue 
- Manage build queue
- View build pipeline
- Edit build pipeline
- Delete build pipeline

Or best of all, have all permissions enabled:
<center>

![](resources/pipelineBerechtigungenAll.png "pipelineBerechtigungenAll.png")

</center>

Now you can set up a new pipeline via Pipelines -> Pipelines -> New Pipeline.

The pipeline YAML file for a c++/cmake project looks like this:
```
stages:
- stage: Build
  jobs:
    - job: 'MyBuildJob'
      pool:
        name: Cobot
        demands:
          - agent.name -equals jetsonDevKit
      steps:
      - script: |
         mkdir build
         cd build
         cmake ..
         make -j8
```

The terms "stage", "job", "step" are explained here:
https://learn.microsoft.com/en-us/azure/devops/pipelines/get-started/key-pipelines-concepts?view=azure-devops


## 4. Create test pipeline
To create a test pipeline another stage must be introduced.
The TestStage should only be executed if the BuildStage was completed successfully (` condition: succeeded('BuildStage')` ).
In this stage cmake must be executed with the test flag, so that all tests are compiled. Then call the desired test executables. This can be done in several steps to get a nicer representation in DevOps.
Note that no .onnx/.jpg/.png files should be tracked in git.
However, to run the tests you need these files. Therefore they were stored locally on the Jetson under the path `~/Downloads/00_unitTestPipeline/`.

The Yaml file now looks like this:
```
stages:
- stage: BuildStage
  jobs:
    - job: 'MyBuildJob'
      pool:
        name: Cobot
        demands:
          - agent.name -equals jetsonDevKit
      steps:
      - script: |
         mkdir build
         cd build
         cmake ..
         make -j8
          
- stage: TestStage
  dependsOn: BuildStage
  condition: succeeded('BuildStage')
  jobs:
    - job: 'MyTestJob'
      pool:
        name: Cobot
        demands:
          - agent.name -equals jetsonDevKit
      steps:
      - script: |
         mkdir build
         cd build
         cmake -D BUILD_TESTS=ON ..
         make -j8
         cd tests
         ./TEST_Yolov5
```