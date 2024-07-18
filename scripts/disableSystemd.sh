# @copyright Copyright (c) 2022 Gerhard Schubert GmbH - All Rights Reserved
#
# Unauthorized copying of this file, via any medium is strictly prohibited
# Proprietary and confidential

#!/bin/sh

sudo systemctl stop systemd-networkd
sudo systemctl disable systemd-networkd

sudo systemctl start NetworkManager.service
sudo systemctl start NetworkManager-wait-online.service
sudo systemctl start NetworkManager-dispatcher.service
sudo systemctl start network-manager.service

sudo systemctl enable NetworkManager.service
sudo systemctl enable NetworkManager-wait-online.service
sudo systemctl enable NetworkManager-dispatcher.service
sudo systemctl enable network-manager.service





