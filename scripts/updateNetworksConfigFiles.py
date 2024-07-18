#!/usr/bin/env python3

# @copyright Copyright (c) 2023 Gerhard Schubert GmbH - All Rights Reserved

# Unauthorized copying of this file, via any medium is strictly prohibited
# Proprietary and confidential

# This Python script is designed to update JSON configuration files in a specific folder and its subfolders. 
# It reads a template file and an edit file that contain information about how the configuration files should be updated.

# When it encounters a file that begins with "networksConfig", 
# it performs a series of updates based on the instructions in the template and edit file. 
# It also makes a copy of the original file before making the changes and compares the updated file with the original file 
# to ensure that the changes have been made correctly. The original file is deleted at the end

import os
import json
import shutil
from collections import OrderedDict

# Read information from template-file
def read_file(file_path):
    with open(file_path, 'r') as file:
        data = json.load(file)
    return data

# Function for adding new data without overwriting existing keys
def add_new_data(existing_data, template_data):
    if isinstance(template_data, dict):
        for key, value in template_data.items():
            if isinstance(value, dict) and isinstance(existing_data.get(key), dict):
                add_new_data(existing_data[key], value)
            elif isinstance(value, list) and isinstance(existing_data.get(key), list):
                for i in range(len(value)):
                    if i < len(existing_data[key]):
                        if isinstance(value[i], (list, dict)):
                            add_new_data(existing_data[key][i], value[i])
                    else:
                        existing_data[key].append(value[i])
            elif key not in existing_data:
                existing_data[key] = value
    elif isinstance(template_data, list):
        for i, item in enumerate(template_data):
            if i < len(existing_data):
                if isinstance(item, (list, dict)):
                    add_new_data(existing_data[i], item)
            else:
                existing_data.append(item)

# Function to delete or overwrite key pairs
def remove_from_file(data, keys):
    if len(keys) == 1:
        if isinstance(data, list):
            index = int(keys[0])
            if index < len(data):
                del data[index]
        else:
            if keys[0] in data:
                del data[keys[0]]
    else:
        if isinstance(data, list):
            index = int(keys[0])
            if index < len(data):
                remove_from_file(data[index], keys[1:])
        else:
            if keys[0] in data:
                remove_from_file(data[keys[0]], keys[1:])

# This function sorts the keys in a given data dictionary based on the order of the keys in a template dictionary. 
# The function is recursive and can handle nested dictionaries and lists.
def sort_data_to_new_data_order(data, template_data):
    if isinstance(data, dict) and isinstance(template_data, dict):
        sorted_data = OrderedDict()
        for key in template_data.keys():
            if key in data:
                sorted_data[key] = sort_data_to_new_data_order(data[key], template_data[key])
        for key in data.keys():
            if key not in sorted_data:
                sorted_data[key] = data[key]
        return sorted_data
    elif isinstance(data, list) and isinstance(template_data, list):
        for i in range(len(data)):
            if i < len(template_data):
                data[i] = sort_data_to_new_data_order(data[i], template_data[i])
        return data
    else:
        return data

def sort_dict(d):
    return json.dumps(d, sort_keys=True)


#####################################################################################
#                                MAIN FUNCTION                                      #
#####################################################################################

def main():
    # Path information
    folder_path = '../format'
    template_file_path = 'config/updateNetworksConfigTemplate.json'
    editing_file_path = 'config/updateNetworksConfigEditing.json'

    template_data = read_file(template_file_path)
    editing_data = read_file(editing_file_path)

    # Ignored Folders
    ignored_folders = ["999", "0"]

    # Cycle through all files in the folder and its subfolders
    processed_files = 0
    total_files = 0
    for root, dirs, files in os.walk(folder_path):
        # Check if the current folder should be ignored
        if os.path.basename(root) in ignored_folders:
            continue

        for file in files:
            total_files += 1
            if file.endswith('.json'):
                file_path = os.path.join(root, file)
                file_name = os.path.basename(file_path)

                # Check if the file name starts with "networksConfig"
                if file_name.startswith("networksConfig"):
                    processed_files += 1

                    # Make a copy of the original file
                    original_file_path = f"{file_path}.orig"
                    shutil.copyfile(file_path, original_file_path)

                    with open(file_path, 'r') as json_file:
                        data = json.load(json_file)

                    # Delete the list if it exists
                    for key_path in editing_data['overwrite']:
                        remove_from_file(data, key_path.split('.'))

                    # Update the data with template_data
                    add_new_data(data, template_data)

                    # Delete the list if it exists
                    for key_path in editing_data['remove']:
                        remove_from_file(data, key_path.split('.'))
                
                    # Sort the data
                    data = sort_data_to_new_data_order(data, template_data)

                    # Write the data back to the file
                    with open(file_path, 'w') as json_file:
                        json.dump(data, json_file, indent=4)

                    # Remove the original file
                    os.remove(original_file_path)
                    
    print(f"Total number of found networksConfig.json files: {total_files}")
    print(f"Number of processed networksConfig.json files: {processed_files}")

if __name__ == "__main__":
    main()