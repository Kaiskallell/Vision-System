# @copyright Copyright (c) 2023 Gerhard Schubert GmbH - All Rights Reserved
#
# Unauthorized copying of this file, via any medium is strictly prohibited
# Proprietary and confidential

import os
import json
import pathlib
import sys

def query_yes_no(question, default="no"):
    """Ask a yes/no question via raw_input() and return their answer.

    "question" is a string that is presented to the user.
    "default" is the presumed answer if the user just hits <Enter>.
            It must be "yes" (the default), "no" or None (meaning
            an answer is required of the user).

    The "answer" return value is True for "yes" or False for "no".
    """
    valid = {"yes": True, "y": True, "ye": True, "no": False, "n": False}
    if default is None:
        prompt = " [y/n] "
    elif default == "yes":
        prompt = " [Y/n] "
    elif default == "no":
        prompt = " [y/N] "
    else:
        raise ValueError("invalid default answer: '%s'" % default)

    while True:
        sys.stdout.write(question + prompt)
        choice = input().lower()
        if default is not None and choice == "":
            return valid[default]
        elif choice in valid:
            return valid[choice]
        else:
            sys.stdout.write("Please respond with 'yes' or 'no' " "(or 'y' or 'n').\n")
        

# iterate through all format folders and read all networkconfig.jsons
listOfAllNetworkConfigPaths = []
formatFolder = "../format"
for root, dirs, files in os.walk(formatFolder):
    for file in files:
        listOfAllNetworkConfigPaths.append( os.path.join(root, file))

# and list all networkpaths which are specified in json files
networkPathsList = []
for networksConfigFilePath in listOfAllNetworkConfigPaths:
    #print(networksConfigFilePath)
    f = open(networksConfigFilePath)
    # returns JSON object as 
    # a dictionary
    jsonFile = json.load(f)
    if 'stereoDepth' in jsonFile:
        value = jsonFile['stereoDepth']['onnxPath']
        if value != "":
            networkPathsList.append( value)   
    if 'yolo' in jsonFile:   
        value = jsonFile['yolo']['onnxPath']
        if value != "":
            networkPathsList.append( value)    
    if 'dlc' in jsonFile:
        value = jsonFile['dlc']['onnxFilePath']
        if value != "":
            networkPathsList.append( value)   

    # Closing file
    f.close()

#print(networkPathsList)

# make all paths relative to script folder or leave it global
networkPathsListRelativeToScriptFolder = []
for onnxPath in networkPathsList:
    p = pathlib.Path(onnxPath)
    try:
        if p.parts[0] == "..":
            # we have a relative onnx path (relative from '../build/bin')
            # we need to remove one '..' from original '../..'
            onnxPath = p.parts[1:]
        networkPathsListRelativeToScriptFolder.append(pathlib.Path(*onnxPath))
        
    except:
        print("WARNING: cannot make relative path from : ", p)
        

#print(networkPathsListRelativeToScriptFolder)

# get all onnxPaths form resources/networks folder
listOfAllOnnxFiles = []
networksFolder = "../resources/networks"
for root, dirs, files in os.walk(networksFolder):
    for file in files:
        filepath = os.path.join(root, file)
        _, fileExtension = os.path.splitext(filepath)
        if (fileExtension == ".onnx"):
            listOfAllOnnxFiles.append( pathlib.Path(filepath))

# check if onnxFile is specified in any networksconfig file
print("The following onnx file can be deleted because they are not specified in any networksConfig.json :")
for onnxFilePath in listOfAllOnnxFiles:
    if onnxFilePath in networkPathsListRelativeToScriptFolder:
        #print(onnxFilePath)
        pass
    else:
        # onnx file can be deleted because not needed
        # delete engine file aswell for fp32 fp16 and int8 if existing
        enginePathFp32 = onnxFilePath.with_suffix('.fp32.engine')
        enginePathFp16 = onnxFilePath.with_suffix('.fp16.engine')
        enginePathInt8 = onnxFilePath.with_suffix('.int8.engine')
        enginePathPlain = onnxFilePath.with_suffix('.engine')
        
        questionToUser = "Do you want to delete this file? " + str(onnxFilePath)
        ret = query_yes_no(questionToUser)
        if (ret == True): # finally deleting onnx and engine files
            if onnxFilePath.is_file():
                onnxFilePath.unlink()
            if enginePathFp32.is_file():
                enginePathFp32.unlink()
            if enginePathFp16.is_file():
                enginePathFp16.unlink()
            if enginePathInt8.is_file():
                enginePathInt8.unlink()
            if enginePathPlain.is_file():
                enginePathPlain.unlink()
