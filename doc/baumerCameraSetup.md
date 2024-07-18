# Setup of Baumer cameras 

First move to the following folder: `cd VisionSystem/libs/external/Baumer_neoAPI_1.2.0_lin_aarch64_cpp/tools`.
Then get the mac address or serial number with :
`./gevipconfig`

Then the network must be set on the laptop so that IP and subnet mask match then

`gevipconfig -c <MAC oder SERIAL NUMBER> --ip <DESIRED_IP_ADDRESS> --subnet <SUBNET> --persistent`

e.g.: 
`gevipconfig -c 700009474069 --ip 192.168.100.52 --subnet 255.255.255.0 --persistent`

Then disconnect and reconnect and check if the Ip address was applied.

There are problems with setting the Ip addresses.
You can't put all cameras in the same subnet, you have to use different Ip address ranges.
A working configuration for Jetson and cameras is:

```
Network Interface poe1:
  IPv4 Address: 192.168.101.41
  Subnet Mask:  255.255.255.0
  Camera Interface:
    IPv4 Address: 192.168.101.51
    Subnet Mask:  255.255.255.0

Network Interface poe2:
  IPv4 Address: 192.168.102.42
  Subnet Mask:  255.255.255.0
  Camera Interface:
    IPv4 Address: 192.168.102.52
    Subnet Mask:  255.255.255.0

Network Interface poe3:
  IPv4 Address: 192.168.103.43
  Subnet Mask:  255.255.255.0
  Camera Interface:
    IPv4 Address: 192.168.103.53
    Subnet Mask:  255.255.255.0

Network Interface poe4:
  IPv4 Address: 192.168.104.44
  Subnet Mask:  255.255.255.0
  Camera Interface:
    IPv4 Address: 192.168.104.54
    Subnet Mask:  255.255.255.0

Network Interface eth0:
  IPv4 Address: 192.168.105.45
  Subnet Mask:  255.255.255.0
  Camera Interface:
    IPv4 Address: 192.168.102.55
    Subnet Mask:  255.255.255.0

Network Interface eth1:
  IPv4 Address: 192.168.106.46
  Subnet Mask:  255.255.255.0
  Camera Interface:
    IPv4 Address: 192.168.106.56
    Subnet Mask:  255.255.255.0
```
