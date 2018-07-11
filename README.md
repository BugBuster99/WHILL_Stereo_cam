ROS Driver for USB Video Class Cameras
======================================

`libuvc_camera` is a ROS driver that supports webcams and other UVC-standards-compliant video devices.
It's a cross-platform replacement for `uvc_camera`, a Linux-only webcam driver.

Documentation is available on the ROS wiki: [libuvc_camera](http://wiki.ros.org/libuvc_camera).


## How to launch e-con camera
### Retrieve camera's serial number
1. Get Bus ID and Device ID of the camera by lsusb. e-con's vendor ID is `2560` and product ID is `c110` (monochrome) or `c111` (color).
For the following example, `Bus 002 Device 004`, `Bus 002 Device 007`, `Bus 002 Device 005`, and `Bus 002 Device 006` indicate all four camera sensors (two eyes for each arm = total four). 
```bash
$ lsusb
Bus 002 Device 006: ID 2560:c110  
Bus 002 Device 005: ID 2560:c110  
Bus 002 Device 003: ID 0424:5532 Standard Microsystems Corp. 
Bus 002 Device 007: ID 2560:c110  
Bus 002 Device 004: ID 2560:c110  
Bus 002 Device 002: ID 0451:8340 Texas Instruments, Inc. 
Bus 002 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
Bus 001 Device 003: ID 0424:2132 Standard Microsystems Corp. 
Bus 001 Device 004: ID 0451:82ff Texas Instruments, Inc. 
Bus 001 Device 002: ID 0451:8342 Texas Instruments, Inc. 
Bus 001 Device 005: ID 062a:4101 Creative Labs Wireless Keyboard/Mouse
Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
```


1. Get serial number of each camera 
```bash
lsusb -v -s XXX:YYY
```
