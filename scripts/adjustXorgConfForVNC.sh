# @copyright Copyright (c) 2023 Gerhard Schubert GmbH - All Rights Reserved
#
# Unauthorized copying of this file, via any medium is strictly prohibited
# Proprietary and confidential

#!/bin/sh

# This script replaces the standart xorg.conf file
# because the standart file does not work if you want
# to connect via vnc and no physical monitor is connected
# to the jetson. This is most likely a bug in the tegra driver.
# This Problem occures in jetpack 5.0.2 and might be resolved in 
# further releases.

sudo cp ../resources/jetson-config/xorg_dummy_driver.conf /etc/X11/xorg.conf

echo "adjust xorg.conf done!"


