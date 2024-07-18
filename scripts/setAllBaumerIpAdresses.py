# @copyright Copyright (c) 2022 Gerhard Schubert GmbH - All Rights Reserved
#
# Unauthorized copying of this file, via any medium is strictly prohibited
# Proprietary and confidential

#!/bin/python3

import subprocess
import sys
import time
import json

class Camera:
    def __init__(self, serialNumber: str, ipAddress: str, subnetmask: str, nic: str):
        self.serialNumber = serialNumber
        self.ipAddress = ipAddress
        self.subnetmask = subnetmask
        self.nic = nic

def askUserForInput(jsonFilePath):
    print("")
    print("JsonFilePath: ", jsonFilePath)
    input("Please adjust the serialNumbers in the json file according to the connected cameras. And then press 'Enter' .")

def readJsonFile(jsonFilePath):

    f = open(jsonFilePath)
    jsonFile = json.load(f)
    camList = []
    for camData in jsonFile["cameras"]:
        serialNumber = camData["serialNumber"]
        desiredIpAddress = camData["desiredIpAddress"]
        desiredSubnetmask = camData["desiredSubnetmask"]
        nic = camData["nic"]
        camList.append(Camera(serialNumber,desiredIpAddress,desiredSubnetmask, nic))
    f.close()
    return camList
    
# set all ips in visible address range (gevipconfig -a)
cmdGevipconfig = "./../libs/external/Baumer_neoAPI_1.2.0_lin_aarch64_cpp/tools/gevipconfig"
subprocess.run(["sudo", cmdGevipconfig , "-a"], stdout=subprocess.PIPE, stderr=subprocess.PIPE) # force all cameras in jetson default subnet (only temporarily)
subprocess.run([cmdGevipconfig], stdout=sys.stdout, stderr=subprocess.PIPE) 

# ask user to modify the json file according to the already printed serialNumbers
jsonFilePath = "../config/baumerCameraIpAddress.json"
askUserForInput(jsonFilePath)

# read serialNumber, desiredIpAddress, desiredSubnetmask from json
camList = readJsonFile(jsonFilePath)

# set all ipAddresses and subnetmasks
for cam in camList:
    resultBaumerInfos = subprocess.run([cmdGevipconfig, "-c",cam.serialNumber, "--ip",cam.ipAddress, "--subnet",cam.subnetmask,"--persistent"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    print("Restart nic",  cam.nic)
    subprocess.run(["sudo", "ifconfig", cam.nic, "down"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    time.sleep(2)  # wait for poe connection to shot down and all capacitors in camera are empty
    subprocess.run(["sudo", "ifconfig", cam.nic, "up"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

print("Wait 10sec for rebooting of baumer cams ...")
time.sleep(10)  # baumer cam needs some time to boot
#user can now check if changes are correct
subprocess.run([cmdGevipconfig], stdout=sys.stdout, stderr=subprocess.PIPE) 

print("Done. If ip address of camera is not set like you expected then please try to plug out and plug in the camera.")

