ROS Driver for USB Video Class Cameras
======================================

`libuvc_camera` is a ROS driver that supports webcams and other UVC-standards-compliant video devices.
It's a cross-platform replacement for `uvc_camera`, a Linux-only webcam driver.

Documentation is available on the ROS wiki: [libuvc_camera](http://wiki.ros.org/libuvc_camera).


## How to launch e-con camera
**NOTE**: Right eye camera (SEE3CAM 1MSTEREO xxx_R_SLAVE) requires trigger signal from left eye camera (SEE3CAM 1MSTEREO xxx_L_MASTER). Thus, "right-eye only" mode is not available. If you want to use the camera as a mono-eye camera, you must use left eye camera.

#### Copy udev file to /etc/udev/rules.d
Execute the following command and replug your e-con camera in order to make the copied udev rule effective.
```bash
$ sudo cp ~/catkin_ws/src/libuvc_ros/libuvc_camera/udev/95-econ.rule /etc/udev/rules.d
```

#### Get Bus ID and Device ID of the camera by lsusb
e-con's vendor ID is `2560` and product ID is `c110` (monochrome) or `c111` (color).
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


#### Get serial number of each camera
Type `lsusb -v -s XXX:YYY` (XXX = Bus ID, YYY = Device ID), and read `iSerial`.
```bash
$ lsusb -v -s 002:004

Bus 002 Device 004: ID 2560:c110  
Device Descriptor:
  bLength                18
  bDescriptorType         1
  bcdUSB               3.00
  bDeviceClass          239 Miscellaneous Device
  bDeviceSubClass         2 ?
  bDeviceProtocol         1 Interface Association
  bMaxPacketSize0         9
  idVendor           0x2560 
  idProduct          0xc110 
  bcdDevice            0.00
  iManufacturer           1 e-con Systems
  iProduct                2 SEE3CAM 1MSTEREO MONO_L_MASTER
  iSerial                 3 16114F0B
  bNumConfigurations      1
```


#### Launch camera
Launch `see3cam_1MStereo.launch`, ZZZZZZZZ is the serial number you obtained in previous step.

- For Monochrome Camera
```bash
$ roslaunch libuvc_camera see3cam_1MStereo.launch serial:=ZZZZZZZZ product="0xc110" video_mode="uncompressed"
```
- For Color Camera
```bash
$ roslaunch libuvc_camera see3cam_1MStereo.launch serial:=ZZZZZZZZ product="0xc111" video_mode="sgrbg"
```
