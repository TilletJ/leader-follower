vicon_bridge
=============
vicon_bridge is a ros package providing data from VICON motion capture systems.
The package was updated to track and publish tf of unlabeled markers using vicon_listener node. 
The ip address of DataStream server machine is 132.207.24.6:801 (801 is the default port).

## QUICK START
To use it, you have to be connected through ethernet to the local network in the lab and run vicon.launch
```
roslaunch vicon_bridge vicon.launch
rostopic echo /vicon/markers
```
