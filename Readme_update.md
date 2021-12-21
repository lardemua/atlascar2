# Atlas Core
Core packages for the LAR DEMUA Atlas Project

# Table of Contents

- [Atlas Core](#atlascar2)
- [Table of Contents](#table-of-contents)
- [SETUP Montage](#setup-montage)
  * [1: Turning ON everything you need](#using-pr2-robot-instead-of-atlascar2) 



# Description of the Atlascar2

The Atlascar2 is an instrumented vehicle used for scientific research in the areas of Autonomous Driving and Driving Assistance Systems.
It contains significant computing power onboard which is used to process the data streaming from several sensors.

![atlascar2.png](docs/atlascar2b.png?raw=true "Atlascar2")

The set of sensors mounted onboard varies according to the needs of the researchers. Nonetheless, there are a few core sensors which are always available:

Name  | Type | Range (m) | Resolution (px) | Frequency (Hz) | Description
:---: | :---: | :---: | :---: | :---: | :---:
left laser | LIDAR, Sick LMS151 | 80 | --- | 50 | Mounted on the front bumper, near the left turn signal.
right laser | LIDAR, Sick LMS151 | 80 | --- | 50 | Mounted on the front bumper, near the right turn signal.
front laser | LIDAR, Sick LD MRS | 200 | --- | 50 | Mounted on the front bumper, at the center. Four scanning planes.
top left camera | Camera, Point Grey Flea2 | --- |  964x724 | 30 | Mounted on the rooftop, to the left.
top right camera | Camera, Point Grey Flea2 | --- |  964x724 | 30 | Mounted on the rooftop, to the right.
gps | Novatel GPS + IMU | --- |  --- | --- | Mounted on the rooftop, to the back and right.

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
![image](https://user-images.githubusercontent.com/92535336/146978719-a64f6bfe-4e12-4d9f-a1d8-5589bfa9d279.png)
   * __Step 3__: Turn on the atlas computer and the UPS.
   * __Step 4__: This step isn’t needed in this case because the ethernet cable is only used to experiment on the car.
* __Step 5__: Turn on the sensors circuit switch.

Now, atlascar2, atlas machine and all sensors are turned on and working!
