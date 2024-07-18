# @copyright Copyright (c) 2022 Gerhard Schubert GmbH - All Rights Reserved
#
# Unauthorized copying of this file, via any medium is strictly prohibited
# Proprietary and confidential
#
# this script removes all the source files so that the robot at the customer 
# has only binary files

#!/bin/sh

rm -r ../tests
rm -r ../src
rm -r ../doc
rm -r ../resources/testfiles
rm -r ../resources/jetson-config
rm -r ../resources/installFiles
rm -r ../resources/cppCodeQualityTools
rm -r ../libs/external/googletest
rm -r ../libs/external/hungarian-algorithm
rm -r ../libs/external/redis-plus-plus
rm -r ../libs/external/spdlog
rm -r ../.git
rm -r ../.vscode
rm -rf ~/.local/share/Trash/*
