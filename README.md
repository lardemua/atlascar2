# atlas-core
Core packages for the LAR DEMUA Atlas Project

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