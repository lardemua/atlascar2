# Atlas Core
Core packages for the LAR DEMUA Atlas Project

# Table of Contents

- [Atlas Core](#atlas-core)
- [Table of Contents](#table-of-contents)
- [SETUP Montage](#setup-montage)
  * [1: Turning ON everything you need](#using-pr2-robot-instead-of-atlascar2) 
  * [2: Working on ATLAS environment](#using-pr2-robot-instead-of-atlascar2)
    * [Using ssh connection (to work on your own machine)](#using-pr2-robot-instead-of-atlascar2)
    * [Using monitors of the car](#using-pr2-robot-instead-of-atlascar2)
- [Compilation](#compilation)
- [Known problems](#known-problems)
  * [Monitors not showing image](#urdf-model-not-showing-on-rviz-or-urdf-model-showed-up-misplaced)

# SETUP Montage

##1: Turning ON everything you need

__Step 1__: Turn on the car.

__Step 2:__ Connect atlas machine to a voltage source.
This connection could be to some electrical outlet outside of the car (in cases where the car doesn't have to move):

![docs/1_power_on.jpg](docs/1_power_on.jpg?raw=true "Game arena")

Or to the UPS voltage source (in cases that you'll drive the car):

![docs/1_power_on.jpg](docs/1_power_on.jpg?raw=true "Game arena")

__Step 3:__ Turn on the atlas computer.

__Step 4:__ Plug the eduroam (UA) ethernet cable (the cable is outside the atlascar2) to the atlas computer (on the figure port).

![docs/UA_cable_port.jpg](docs/UA_cable_port.jpg?raw=true "Game arena")

__Step 5:__ Turn on the sensors circuit switch:

![switch.jpg](docs/switch.jpg?raw=true "Game arena")

__Step 6:__ Turn on the sensors router:

![sensors_router.jpg](docs/sensors_router.jpg?raw=true "Game arena")

Note: If you want to go out with the car but this router is connected to some external voltage outlet, see section "jjjjjjjjjjjjj"

Now, atlascar2, atlas machine and all sensors are turned on and working!

##2: Working on ATLAS environment

###Using ssh connection (to work on your own machine)
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
    
###Using monitors the car (to work on atlas machine)
 If you want to work with the monitors, mouses and keyboards that are in the car (or directly connected to him):

__Step 1:__ Turn on the screens. Just need to get some eletrical outlet outside of the car. Once again,  if you want to go out 
    with the car, see section "jjjjjjjjjjjjj".

__Step 2:__ Connect the USB port extension to the atlas machine. This will connect the screens and the mouses and the
        keyboards.
        
Now you can work inside the atlas environment! 

If you want, you can use Visual Studio or CLion, once they are already installed.

#Known problems
##Monitors of the car not showing image
Just press the POWER button of the atlas machine quickly and only once



----Not need for particular user-----

Need to connect 2 cables: for ua ethernet and router
set the router ipv4 automiatic
--------------------------------------------


3 connections:

Router SMC  (enp5s0f1)

Ethernet UA (enp5s0f0)

AtlasNetwork (ens6f0)
 
 ifconfig to see all the IP gates that you have on your machine

Now you have the cable connection to atlascar (make sure that, on IPv4, is 'Utilize esta ligacao 
apenas para recurso na sua rede' is on and the IPv4 method is 'Automatic')

 
 #Usage
 This launch file, launch the 2 sensors and the camera at once
 
 roslaunch atlas2_bringup bringup.launch
 
 
 #Front Camera:
 IP: 192.168.0.2

 #Top Camera Right:
 IP: 169.254.0.2

 #Top Camera Left:
 IP:169.254.0.1
 
 #Top Camera Right: 
 IP: 169.254.0.2

#Top Camera Left: 
IP:169.254.0.1
 
 Serial: 14233704
 
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
roslaunch atlas2_bringup record_sensor_data.launch 
```

This saves a bag file called **atlascar2.bag** in the Desktop. You must edit the name after recording so that it is not deleted on the next recording.

If you want to view the checkerboard detection while the bagfile is recorded you may run

```bash
rosrun tuw_checkerboard tuw_checkerboard_node image:=/frontal_camera/image_color camera_info:=/frontal_camera/camera_info tf:=tf_dev_null
```

# Playing back sensor data

To playback recorded sensor data you must use a special launch file which decompresses the images. As follows:
```bash
roslaunch atlas2_bringup playback_sensor_data.launch bag:=atlascar2
```

Bag name is referred from the Desktop and without the **.bag** extension

New way to read and display the bag files