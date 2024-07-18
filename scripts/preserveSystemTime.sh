# @copyright Copyright (c) 2023 Gerhard Schubert GmbH - All Rights Reserved
#
# Unauthorized copying of this file, via any medium is strictly prohibited
# Proprietary and confidential

#!/bin/sh

# In jetpack 5.0.2 the systemtime is lost on every restart.
# We set the time on every restart according to the time in
# the rtc

export | grep JETSON # initializes the JETSON_L4T env var
if [ "$JETSON_L4T" == "35.1.0" ]; then  # jetpack 5.0.2 
    sudo cp ../resources/jetson-config/50-udev-default.rules /lib/udev/rules.d/50-udev-default.rules
    sudo rm /dev/rtc
    sudo ln -s /dev/rtc0 /dev/rtc # l4t needs to use the correct rtc which is rtc0
    sudo hwclock -l # set the local systemtime to the rtc
    sudo hwclock -s # sync hwclock time with system time (this needs to be done in autostart -> jetson-performance_jetpack5.service)
fi


