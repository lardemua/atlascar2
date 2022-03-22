<a name="atlascar2"></a>
# Atlas Core
Core packages for the LAR DEMUA Atlas Project

<a name="table-of-contents"></a>
# Table of Contents

- [Atlas Core](#atlascar2)
- [Table of Contents](#table-of-contents)
- [Description of the Atlascar2](#description)
- [SETUP Montage](#setup-montage)
  * [1: Turning ON everything you need](#turning-on) 
  * [2: Configuring the sensors](#configuring-sensors) 
  * [3: Working in the Atlascar2](#working-atlascar2)

- [Testing the sensors](#testing-the-sensors)
  * [1: Sick LMS151 LIDAR](#2d-lidar)
  * [2: Point Grey Flea2 camera](#top-cameras)
  * [3: Sick LD MRS LIDAR](#3d-lidar)

<a name="description"></a>
# Description of the Atlascar2

The Atlascar2 is an instrumented vehicle used for scientific research in the areas of Autonomous Driving and Driving Assistance Systems.
It contains significant computing power onboard which is used to process the data streaming from several sensors.

![atlascar2.png](docs/atlascar2b.png?raw=true "Atlascar2")

The set of sensors mounted onboard varies according to the needs of the researchers. Nonetheless, there are a few core sensors which are always available:

Name  | Type | Range (m) | Resolution (px) | Frequency (Hz) | Description | IP address
:---: | :---: | :---: | :---: | :---: | :---: | :---: 
left laser | LIDAR, Sick LMS151 | 80 | --- | 50 | Mounted on the front bumper, near the left turn signal. | 192.168.0.5
right laser | LIDAR, Sick LMS151 | 80 | --- | 50 | Mounted on the front bumper, near the right turn signal. | 192.168.0.4
front laser | LIDAR, Sick LD MRS | 200 | --- | 50 | Mounted on the front bumper, at the center. Four scanning planes. | 192.168.0.6
top left camera | Camera, Point Grey Flea2 | --- |  964x724 | 30 | Mounted on the rooftop, to the left. | 169.254.0.4
top right camera | Camera, Point Grey Flea2 | --- |  964x724 | 30 | Mounted on the rooftop, to the right. | 169.254.0.5
gps | Novatel GPS + IMU | --- |  --- | --- | Mounted on the rooftop, to the back and right. | ---

<a name="setup-montage"></a>
# SETUP Montage

<a name="turning-on"></a>
## 1: Turning ON everything you need

* __Step 1__: Turn on the car.

* __Case 1__: When the car is stopped:
  * __Step 2__: Connect the atlas machine to an outlet near the car.

 <p align="center">
  <img width="40%" height="40%" src="https://user-images.githubusercontent.com/92535336/146978669-fa6c1f84-eb44-4678-9a43-30730298b5a7.png">
</p>

  *  __Step 3__: Turn on the atlas computer.
  *  __Step 4__: Plug the ethernet cable (the cable is outside the atlascar2) to the atlas computer (on the figure port).

 <p align="center">
  <img width="30%" height="30%" src="https://user-images.githubusercontent.com/92535336/146978585-162eab4a-6cc2-49c0-9330-20996d7a88f6.jpg">
</p>

* __Case 2__: When going for a ride:
   * __Step 2__: Connect the atlas machine to the UPS.

 <p align="center">
  <img width="40%" height="40%" src="https://user-images.githubusercontent.com/92535336/146978719-a64f6bfe-4e12-4d9f-a1d8-5589bfa9d279.png ">
</p>

   * __Step 3__: Turn on the atlas computer and the UPS.
   * __Step 4__: This step isn’t needed in this case because the ethernet cable is only used to experiment on the car.
* __Step 5__: Turn on the sensors circuit switch.

Now, atlascar2, atlas machine and all sensors are turned on and working!

<a name="configuring-sensors"></a>
## 2: Cofiguring the sensors

**Note: This part is only necessary if the atlascar is not configured or to check the ethernet IP addresses of the ethernet ports for the sensors.**

In the car exists two switches to connect to the server.
* One in the front bumper which connects the 2D lidars and the 3D lidar.
* Another in the roof where the top cameras are connected
 
In the table above, it can be seen that both of these sensors need diferent IP addresses to work.

### Front bumper switch

In the ethernet port on the pc it must be the following ip address and mask:
`IP: 198.162.0.3   Mask: 255.255.255.0`

 <p align="center">
  <img width="20%" height="20%" src="https://user-images.githubusercontent.com/92535336/159135428-f1176a52-998a-473c-bcb1-07aa87445026.png">
</p>

### Roof switch

In the ethernet port on the pc it must be the following ip address and mask:
`IP: 169.254.0.3   Mask: 255.255.255.0`

 <p align="center">
  <img width="20%" height="20%" src="https://user-images.githubusercontent.com/92535336/159135330-bcf1ba1b-a05a-42cd-98b7-5d9ff9a954cc.png">
</p>

With this, launching the drivers of the sensors must work!

<a name="working-atlascar2"></a>
## 3:Working in the Atlascar2
### Using teamviewer for remote work
The teamviewer app is configured to open automatically in atlascar2 after turning on the PC.
So in order to connect to the atlascar the user only needs to add the user and password in his teamviewer app and it should be working

* User number: 1 145 728 199
* Password: ask the administrator

With this the user will see the atlascar pc!

<a name="testing-the-sensors"></a>
# Testing the sensors

<a name="2d-lidar"></a>
## 1: Sick LMS151 LIDAR

To launch one of the 2D Lidars:

    roslaunch atlascar2_bringup laser2d_bringup.launch name:=left
    or
    roslaunch atlascar2_bringup drivers_bringup.launch 2DLidar_right_bringup:=true

Where left can be replaced for right.
 
<a name="top-cameras"></a>
## 2: Point Grey Flea2 camera

To launch a camera:

    roslaunch atlascar2_bringup top_cameras_bringup.launch.launch name:=left
    or
    roslaunch atlascar2_bringup drivers_bringup.launch top_camera_left_bringup:=true

Where left can be replaced for right.

<a name="3d-lidar"></a>
## 3: Sick LD MRS LIDAR

To launch only this sensor:

    roslaunch atlascar2_bringup drivers_bringup.launch 3DLidar_bringup:=true

## To test the system as a whole

Launch the file:

    roslaunch atlascar2_bringup bringup.launch
    
Which has the following arguments:

* visualize -> see rviz or not
* 2DLidar_left_bringup -> launch the left 2D lidar
* 2DLidar_right_bringup -> launch the right 2D lidar
* 3DLidar_bringup -> launch the 3D lidar
* top_camera_right_bringup -> launch the right top camera
* top_camera_left_bringup -> launch the left top camera
* front_camera_bringup -> launch the front camera
* RGBD_camera_bringup -> launch the RGBD camera
* novatel_bringup -> launch the GPS

Note: The front and RGBD camera aren't in the car right now, so these arguments should be false

