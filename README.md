# atlas-core
Core packages for the LAR DEMUA Atlas Project


# SETUP

Step 1: plug the eduroam (UA) ethernet cable (the cable is outside the atlascar2) to the atlas computer (on the figure port).

![docs/UA_cable_port.jpg](docs/UA_cable_port.jpg?raw=true "Game arena")

Step 2: Turn on the computer 

Step 3: Plug the ethernet cable from the SMC router to your own computer. (The router is inside the atlascar2)

Step 4: On a terminal, run

```
sudo gedit /etc/hosts
```

this will open the hosts file. You should save the AtlasCar2 network ip (add this to your hosts file):

192.168.2.102    ATLASCAR2

Save and close.

Step 5: Make sure if the connection to atlascar is on by running 

```
ping ATLASCAR2
```

----Not need for particular user-----

Need to connect 2 cables: for ua ethernet and router
set the router ipv4 automiatic
--------------------------------------------


3 connections:

Router SMC  (enp5s0f1)

Ethernet UA (enp5s0f0)

AtlasNetwork (ens6f0)

how to connect:
connect the cable from the router to your pc
run

sudo gedit /etc/hosts
 and add to the file 
192.168.2.102    ATLASCAR2
save

to see if the connection is on, run
 ping ATLASCAR2

Now you have the cable connection to atlascar (make sure that, on IPv4, is 'Utilize esta ligacao 
apenas para recurso na sua rede' is on and the IPv4 method is 'Automatic')

----- NOT needed ---------------
Do the deployment to syncronize the code with atlas (do stuff on our own computer but is running on atlas pc)
See  https://www.jetbrains.com/help/pycharm/creating-a-remote-server-configuration.html for that
--------------------------------

Open the code inside the atlas car pc (terminal with the atlas):
 ssh atlas@ATLASCAR2 -X (to view)
 
 And here you open the code to work on to (ATLas has VSstudio and PyCharm and CLion installed already.)
 
 
 #Usage
 This launch file, launch the 2 sensors and the camera at once
 
 roslaunch atlas2_bringup bringup.launch
 
 
 #Front Camera:
 IP: 192.168.0.2
 
 Serial: 14233704
 
to see the image received by the camera, run
rosrun image_view image_view image:=