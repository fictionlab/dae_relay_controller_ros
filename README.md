# dae_relay_controller_ros

A ROS package for relay's module comunication. Based on python lib [dae-py-relay-controller](https://github.com/petersbingham/dae-py-relay-controller)

### Relay module

The relay_node is tested for [USB 16 Channel Relay Module](http://denkovi.com/usb-relay-16-channel-module-rs232-controlled-din-rail-box)


### UDEV rules

Add following udev rule to /etc/udev/rules.d/relay.rules file:
```
KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE="0666", GROUP="dialout", SYMLINK+="relay"
``` 
Reload udev rules:
``` 
sudo udevadm control --reload-rules && sudo udevadm trigger
``` 

### Start node 

``` 
rosrun dae_relay_controller_ros relay_node.py device:="/dev/relay" module_type:="type16"
``` 
or
``` 
roslaunch dae_relay_controller_ros example.launch
``` 