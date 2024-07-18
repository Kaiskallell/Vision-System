# @copyright Copyright (c) 2022 Gerhard Schubert GmbH - All Rights Reserved
#
# Unauthorized copying of this file, via any medium is strictly prohibited
# Proprietary and confidential

#!/bin/sh

#We use `systemd-networkd` to manage our network interfaces rather than the default Ubuntu #`NetworkManager`.
sudo systemctl stop NetworkManager.service
sudo systemctl stop NetworkManager-wait-online.service
sudo systemctl stop NetworkManager-dispatcher.service
sudo systemctl stop network-manager.service

sudo systemctl disable NetworkManager.service
sudo systemctl disable NetworkManager-wait-online.service
sudo systemctl disable NetworkManager-dispatcher.service
sudo systemctl disable network-manager.service

#The `systemd-networkd` config files are stored in `resources/jetson-config/systemd/network` and need #to be copied to `/etc/systemd/network`. Now we can start and enable systemd-networkd
sudo systemctl start systemd-networkd
sudo systemctl enable systemd-networkd


#copying service files to systemd folder
sudo cp ../resources/jetson-config/systemd/system/missionControl_jetpack5.service /etc/systemd/system
sudo cp ../resources/jetson-config/systemd/system/jetson-performance_jetpack5.service /etc/systemd/system
sudo cp ../resources/jetson-config/systemd/system/syslog_manager.service /etc/systemd/system

#copying network configurations to systemd folder
sudo cp ../resources/jetson-config/systemd/network/20-lan1.network /etc/systemd/network
sudo cp ../resources/jetson-config/systemd/network/20-lan2.network /etc/systemd/network
sudo cp ../resources/jetson-config/systemd/network/20-poe1.network /etc/systemd/network
sudo cp ../resources/jetson-config/systemd/network/20-poe2.network /etc/systemd/network
sudo cp ../resources/jetson-config/systemd/network/20-poe3.network /etc/systemd/network
sudo cp ../resources/jetson-config/systemd/network/20-poe4.network /etc/systemd/network
sudo cp ../resources/jetson-config/systemd/network/20-eth0.network /etc/systemd/network/20-eth0.network
sudo cp ../resources/jetson-config/systemd/network/20-eth1.network /etc/systemd/network/20-eth1.network

#start the services
sleep 10
sudo systemctl start jetson-performance_jetpack5
sudo systemctl enable jetson-performance_jetpack5
sudo systemctl start missionControl_jetpack5
sudo systemctl enable missionControl_jetpack5

chmod u+x manage_syslog.sh
sudo systemctl start syslog_manager
sudo systemctl enable syslog_manager



