# @copyright Copyright (c) 2022 Gerhard Schubert GmbH - All Rights Reserved
#
# Unauthorized copying of this file, via any medium is strictly prohibited
# Proprietary and confidential
#
# use this script to copy the config files from VisionSystem to be able to recover an old software state
# You can invoke it from a distant computer which is located in the same network (192.168.52.*).

#!/bin/bash

failCpyFolder(){
    echo -e "\e[1;31mfailed to copy folder, exiting \e[0m" && exit 1
}

username="cobot"
if [ "$USER" = "$username" ]
then
   echo "User name is: $username which means that you are running this scipt on the jetson which is not how this script is intended to be used."
   echo "Please change to another computer and make the backup of the jetson there." 
   exit 1 
fi

backupFolderName=VsBackup$(date +%Y%m%d%H%M)
mkdir $backupFolderName && cd $backupFolderName

echo -e "\e[1;32mcopying format folder \e[0m"
{ scp -r cobot@192.168.52.120:/home/cobot/Documents/git/VisionSystem/format . ; } || { failCpyFolder; }
echo -e "\e[1;32mcopying config folder \e[0m"
{ scp -r cobot@192.168.52.120:/home/cobot/Documents/git/VisionSystem/config . ; } || { failCpyFolder; }
echo -e "\e[1;32mcopying networks folder \e[0m"
{ scp -r cobot@192.168.52.120:/home/cobot/Documents/git/VisionSystem/resources/networks . ; } || { failCpyFolder; }
echo -e "\e[1;32mcopying VS_SoftwareBundle folder \e[0m"
{ scp -r cobot@192.168.52.120:/home/cobot/Documents/git/VisionSystem/VS_SoftwareBundle.md . ; } || { failCpyFolder; }
echo -e "\e[1;32mcopying git-state.txt \e[0m"
{ scp -r cobot@192.168.52.120:/home/cobot/Documents/git/VisionSystem/build/git-state.txt . ; } || { failCpyFolder; }
echo -e "\e[1;32mDone. Backup is located in: \e[0m"
echo $backupFolderName
cd ..
