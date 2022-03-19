# Atlas Core
Core packages for the LAR DEMUA Atlas Project

# Table of Contents

- [Atlas Core](#atlascar2)
- [Table of Contents](#table-of-contents)
- [SETUP Montage](#setup-montage)
  * [1: Turning ON everything you need](#using-pr2-robot-instead-of-atlascar2) 

- [Testing the sensors](#testing-the-sensors)
  * [1: Sick LMS151 LIDAR](#1-sick-lms151-lidar)
  * [2: Point Grey Flea2 camera](#2-point-grey-Flea2-camera)
  * [3: Sick LD MRS LIDAR](#3-sick-ld-mrs-lidar)


# Description of the Atlascar2

The Atlascar2 is an instrumented vehicle used for scientific research in the areas of Autonomous Driving and Driving Assistance Systems.
It contains significant computing power onboard which is used to process the data streaming from several sensors.

![atlascar2.png](docs/atlascar2b.png?raw=true "Atlascar2")

The set of sensors mounted onboard varies according to the needs of the researchers. Nonetheless, there are a few core sensors which are always available:

Name  | Type | Range (m) | Resolution (px) | Frequency (Hz) | Description | IP address
:---: | :---: | :---: | :---: | :---: | :---: | :---: 
left laser | LIDAR, Sick LMS151 | 80 | --- | 50 | Mounted on the front bumper, near the left turn signal. | 192.168.0.4
right laser | LIDAR, Sick LMS151 | 80 | --- | 50 | Mounted on the front bumper, near the right turn signal. | 192.168.0.5
front laser | LIDAR, Sick LD MRS | 200 | --- | 50 | Mounted on the front bumper, at the center. Four scanning planes. | 192.168.0.6
top left camera | Camera, Point Grey Flea2 | --- |  964x724 | 30 | Mounted on the rooftop, to the left. | 169.254.0.4
top right camera | Camera, Point Grey Flea2 | --- |  964x724 | 30 | Mounted on the rooftop, to the right. | 169.254.0.5
gps | Novatel GPS + IMU | --- |  --- | --- | Mounted on the rooftop, to the back and right. | ---


# SETUP Montage

## 1: Turning ON everything you need

* __Step 1__: Turn on the car.

* __Case 1__: When the car is stopped:
  * __Step 2__: Connect the atlas machine to an outlet near the car.
 ![image](https://user-images.githubusercontent.com/92535336/146978669-fa6c1f84-eb44-4678-9a43-30730298b5a7.png)
  *  __Step 3__: Turn on the atlas computer.
  *  __Step 4__: Plug the ethernet cable (the cable is outside the atlascar2) to the atlas computer (on the figure port).
![Drawing_readme](https://user-images.githubusercontent.com/92535336/146978585-162eab4a-6cc2-49c0-9330-20996d7a88f6.jpg)
* __Case 2__: When going for a ride:
   * __Step 2__: Connect the atlas machine to the UPS.
![image](https://user-images.githubusercontent.com/92535336/146978719-a64f6bfe-4e12-4d9f-a1d8-5589bfa9d279.png )
   * __Step 3__: Turn on the atlas computer and the UPS.
   * __Step 4__: This step isn’t needed in this case because the ethernet cable is only used to experiment on the car.
* __Step 5__: Turn on the sensors circuit switch.

Now, atlascar2, atlas machine and all sensors are turned on and working!

## 2:Working in the Atlascar2
### Using teamviewer for remote work
The teamviewer app is configured to open automatically in atlascar2 after turning on the PC.
So in order to connect to the atlascar the user only needs to add the user and password in his teamviewer app and it should be working

* Username: Atlas
* Password: atlas

With this the user will see the atlascar pc!

# 2:Testing the sensors

## 1: Sick LMS151 LIDAR
1: Know the IP addresses of the LIDAR, which are:
  * Right_laser : 192.168.0.5
  * Left_laser : 192.168.0.4

2: Create a static address with the following IP address and mask:

`IP: 198.162.0.3   Mask: 255.255.255.0`

3: Launch the drivers.launch file with this code uncommented:

    roslaunch atlascar2_bringup left_laser.launch

```xml
<!-- Left laser -->
<include file="$(find atlascar2_bringup)/launch/left_laser.launch">
</include>

<!-- Right laser -->
<include file="$(find atlascar2_bringup)/launch/right_laser.launch">
</include>
```
4: Open rviz and it should be working
 

## 2: Point Grey Flea2 camera

1: Know the IP addresses of the camera, which are:
  * Top Right Camera: IP: 169.254.0.5
  * Top Left Camera: IP: 169.254.0.4

2: Create a static address with the following IP address and mask:

`IP: 169.254.0.3   Mask: 255.255.255.0`

3: Launch the roslaunch file that should had come with the download of the drivers:

`roslaunch pointgrey_camera_driver camera.launch`

4: Open the flycap program and it should be working

**If the program doesn't find a camera**

 1: Open the terminal and write:
 
 `updatorgui`
 
 2: Force IP address (the program goes by serial number so the IP is not very important)
 
 3: Now the flycap program should see the camera 

## 3: Sick LD MRS LIDAR

1: Follow the installation steps in this repository (if you didn't install it already):

[Installation guide for the Sick LD MRS LIDAR](https://github.com/SICKAG/sick_ldmrs_laser#installation)
(in the installation guide the line: `git clone https://github.com/SICKAG/libsick_ldmrs.git` is not needed)

2: Know the IP address of the LIDAR, which is:
  * 3D LIDAR: 192.168.0.6

3: Create a static address with the following IP address and mask:

`IP: 192.168.0.3   Mask: 255.255.255.0`

4: Launch the drivers.launch file with this code uncommented:

```xml
   <group ns="frontal_laser">
       <include file="$(find atlascar2_bringup)/launch/sick_ldmrs_node.launch">
       </include>
   </group>
```
5: Open rviz and it should be working

