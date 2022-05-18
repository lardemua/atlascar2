# Table of Contents

- [Atlas Core](#atlascar2)
- [Table of Contents](#table-of-contents)
- [SETUP Montage](#setup-montage)
- [Description of the Atlascar2](#description)
- [SETUP Montage](#setup-montage)
  * [1: Turning ON everything you need](#turning-on) 
  * [2: Configuring the IP addresses](#configuring-sensors) 
  * [3: Working in the Atlascar2](#working-atlascar2)

- [Testing the sensors](#testing-the-sensors)
  * [1: Sick LMS151 LIDAR](#2d-lidar)
  * [2: Point Grey Flea2 camera](#top-cameras)
  * [3: Sick LD MRS LIDAR](#3d-lidar)
- [Launch the system](#Launch)

(Part in need of a revision)
 - [ Working on ATLAS environment](#using-pr2-robot-instead-of-atlascar2)
    * [Using ssh connection (to work on your own machine)](#using-pr2-robot-instead-of-atlascar2)
    * [Using monitors of the car](#using-pr2-robot-instead-of-atlascar2)
- [Compilation](#compilation)
- [Known problems](#known-problems)
  * [Monitors not showing image](#urdf-model-not-showing-on-rviz-or-urdf-model-showed-up-misplaced)
- [Simulating the Atlascar2](#simulation)
  * [Installing](#installing)
  * [Running](#running)

# Atlas Core
Core packages for the LAR DEMUA Atlas Project:
* [LIDAR, Sick LMS151](http://wiki.ros.org/LMS1xx)
* [LIDAR, Sick LD MRS](https://github.com/SICKAG/sick_ldmrs_laser)
* [Camera, Point Grey Flea2](http://wiki.ros.org/pointgrey_camera_driver)
* [Novatel GPS + IMU](https://github.com/swri-robotics/novatel_gps_driver)


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
top left camera | Camera, Point Grey Flea2 | --- |  964x724 | 30 | Mounted on the rooftop, to the left. | 169.254.0.5
top right camera | Camera, Point Grey Flea2 | --- |  964x724 | 30 | Mounted on the rooftop, to the right. | 169.254.0.4
gps | Novatel GPS + IMU | --- |  --- | --- | Mounted on the rooftop, to the back and right. | ---

<a name="setup-montage"></a>
# SETUP Montage

<a name="turning-on"></a>
## 1: Turning ON everything you need

* __Step 1__: Turn on the car.

* __Case 1__: When the car is stopped:
  - __Step 2__: Connect the atlas machine to an outlet near the car.

   <p align="center">
     <img width="40%" height="40%" src="https://user-images.githubusercontent.com/92535336/146978669-fa6c1f84-eb44-4678-9a43-30730298b5a7.png">
   </p>

   -  __Step 3__: Turn on the atlas computer.
   -  __Step 4__: Plug the ethernet cable (the cable is outside the atlascar2) to the atlas computer (on the figure port enp5s0f1).

 <p align="center">
  <img width="25%" height="25%" src="https://user-images.githubusercontent.com/92535336/161252671-b540bd35-3695-4a42-a245-36b14e1c8c24.png">
</p>

* __Case 2__: When going for a ride:
   - __Step 2__: Connect the atlas machine to the UPS.

   <p align="center">
     <img width="40%" height="40%" src="https://user-images.githubusercontent.com/92535336/146978719-a64f6bfe-4e12-4d9f-a1d8-5589bfa9d279.png ">
   </p>

   - __Step 3__: Turn on the atlas computer and the UPS.
   - __Step 4__: This step isn’t needed in this case because the ethernet cable is only used to experiment on the car.
* __Step 5__: Turn on the sensors' circuit switch.

Now, atlascar2, atlas machine and all sensors are turned on and working!

<a name="configuring-sensors"></a>
## 2: Cofiguring the IP addresses

**Note: This part is only necessary if the atlascar is not configured or to check the ethernet IP addresses of the ethernet ports for the sensors.**

In the car there are two switches to connect to the server.
* One in the front bumper which connects the 2D lidars and the 3D lidar.
* Another in the roof where the top cameras are connected.
 
In the table above, it can be seen that both of these sensors need different IP addresses to work.

### Front bumper switch

The ethernet port on the pc (ens6f1) must have the following ip address and mask:
`IP: 192.168.0.3   Mask: 255.255.255.0`

 <p align="center">
  <img width="20%" height="20%" src="https://user-images.githubusercontent.com/92535336/159135428-f1176a52-998a-473c-bcb1-07aa87445026.png">
</p>

### Roof switch

The ethernet port on the pc (ens6f0) must have the following ip address and mask:
`IP: 169.254.0.3   Mask: 255.255.255.0`

 <p align="center">
  <img width="20%" height="20%" src="https://user-images.githubusercontent.com/92535336/159135330-bcf1ba1b-a05a-42cd-98b7-5d9ff9a954cc.png">
</p>

With this, launching the drivers of the sensors should work!

<a name="working-atlascar2"></a>
## 3: Working in the Atlascar2
### Using teamviewer for remote work
The teamviewer app is configured to open automatically in atlascar2 after turning on the PC.
So in order to connect to the atlascar, the user only needs to add the user number and password in his teamviewer app and it should be working.

* User number: 1 145 728 199
* Password: ask the administrator

With this the user will see the atlascar desktop!

<a name="testing-the-sensors"></a>
# Testing the sensors

<a name="2d-lidar"></a>
## 1: Sick LMS151 LIDAR

To launch one of the 2D Lidars:

    roslaunch atlascar2_bringup laser2d_bringup.launch name:=left

Where left can be replaced for right.
Then, open rviz.
 
<a name="top-cameras"></a>
## 2: Point Grey Flea2 camera

To launch one camera:

    roslaunch atlascar2_bringup top_cameras_bringup.launch name:=left

Where left can be replaced for right depending which camera the user wants to see.

Then, open rviz or in the terminal write `rosrun image_view image_view image:=/camera/image_raw`

<a name="3d-lidar"></a>
## 3: Sick LD MRS LIDAR

Launch the 3D Lidar:

    roslaunch atlascar2_bringup sick_ldmrs_node.launch
    
Then, open rviz.

<a name="Launch"></a>
# Launch the system

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

# Known problems
## Monitors of the car not showing image
Just press the POWER button of the atlas machine quickly and only once



----Not need for particular user-----

Need to connect 2 cables: for ua ethernet and router
set the router ipv4 automatic
--------------------------------------------


3 connections: (FRONT SWITCH, BACK SWITCH, UA ETHERNET)

Router SMC  (enp5s0f1)

Ethernet UA (enp5s0f0)

AtlasNetwork (ens6f0)
 
 ifconfig to see all the IP gates that you have on your machine

Now you have the cable connection to atlascar (make sure that, on IPv4, is 'Utilize esta ligacao 
apenas para recurso na sua rede' is on and the IPv4 method is 'Automatic')

 
 __Usage__
 
 This launch file, launch the 2 sensors and the camera at once
 
 roslaunch atlascar2_bringup bringup.launch
 
 __Frontal Camera:__
 IP: 192.168.0.2

 __Top Right Camera:__
 IP: 169.254.0.102

 __Top Left Camera:__
 IP: 169.254.0.101
 
 Serial: 14233704 (THIS SERIAL BELONGS TO WICH SENSOR??)
 
to see the image received by the camera, run
rosrun image_view image_view image:=


# Compilation
Inside catkin_ws, run:

```
catkin_make --pkg driver_base
catkin_make --pkg novatel_gps_msgs
```

This is necessary to generate the required files for a final compilation, with:

```
catkin_make
```

# Collecting sensor data

you can record a bag file using

```bash
roslaunch atlascar2_bringup record_sensor_data.launch 
```

This saves a bag file called **atlascar2.bag** in the Desktop. You must edit the name after recording so that it is not deleted on the next recording.

If you want to view the checkerboard detection while the bagfile is recorded you may run

```bash
rosrun tuw_checkerboard tuw_checkerboard_node image:=/frontal_camera/image_color camera_info:=/frontal_camera/camera_info tf:=tf_dev_null
```

# Playing back sensor data

To playback recorded sensor data you must use a special launch file which decompresses the images. As follows:
```bash
roslaunch atlascar2_bringup playback_sensor_data.launch bag:=atlascar2
```

Bag name is referred from the Desktop and without the **.bag** extension

New way to read and display the bag files



# Using ssh connection (to work on your own machine)
(IS THIS REALLY USEFULL??)

If you want to work on your own machine, use the router SMC to create a ssh connection to the 
    atlas computer.

__Step 1:__ Turn on the SMC router.

__Step 2:__ Plug the ethernet cable from the SMC router to your own computer (white cable on the figure).

![SMC_router.jpg](docs/SMC_router.jpg?raw=true "Game arena")

__Step 3:__ On a terminal, run (on your computer):

```
sudo gedit /etc/hosts
```

this will open the hosts file. You should save the AtlasCar2 network ip (add this to your hosts file):

192.168.2.102    ATLASCAR2

Save and close.

__Step 3:__ Make sure if the connection to atlascar is on, by running 

```
ping ATLASCAR2
```

__Step 4:__ Get into the atlas environment by running, on a new terminal (the '-X' is for you visualize the image that 
    sensors are capturing):

```
 ssh atlas@ATLASCAR2 -X 
```
Now you are inside the atlascar machine in your own computer! 

You can work with Visual Studio or CLion, as they are already installed.

<a name="simulation"></a>
# Simulating the Atlascar2

<a name="Installing"></a>
## Installing
In order to ease the remote work with this vehicle, a simulated environment was developed.
This environment uses an [ackermann controller](http://wiki.ros.org/ackermann_steering_controller) that needs to be installed with the following command:

``` 
sudo apt-get install ros-noetic-ros-controllers ros-noetic-ackermann-msgs
```

![simulation.png](docs/simulation.png?raw=true "Simulation")

The user also needs to download and configure the repository [gazebo_models_worlds_collection](https://github.com/chaolmu/gazebo_models_worlds_collection) in order to run AtlasCar2 in Gazebo.


Lastly, the [steer_drive_ros](https://github.com/CIR-KIT/steer_drive_ros) (on the `melodic-devel` branch) needs to be downloaded in the user's ROS workspace.

<a name="Running"></a>
## Running
Now, to start Gazebo the user writes:

``` 
roslaunch atlascar2_gazebo gazebo.launch
```

And to spawn the car and start the controller the user writes:

``` 
roslaunch atlascar2_bringup ackermann_bringup.launch
```
To control the car, publish a twist message to the `/atlascar2/ackermann_steering_controller/cmd_vel` topic.

A video example of the simulation can be seen [here](https://www.youtube.com/watch?v=UWo4ndSZ1XU).

## Calibration
In order to calibrate atlascar2 in the simulated environment, the user needs to run a separate gazebo world:

```
roslaunch atlascar2_gazebo gazebo_calibration.launch
```

And a separate bringup:

```
roslaunch atlascar2_bringup ackermann_bringup.launch yaw:=-1.57 x_pos:=0 y_pos:=2 z_pos:=0 calibration:=true
```

After that, follow the [instructions on the ATOM package](https://github.com/lardemua/atom#system-calibration---detailed-description).
