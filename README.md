# libcreate #

C++ library for interfacing with iRobot's [Create 2](http://www.irobot.com/About-iRobot/STEM/Create-2.aspx) as well as most models of Roomba. [create_autonomy](http://wiki.ros.org/create_autonomy) is a [ROS](http://www.ros.org/) wrapper for this library.

* [Code API](http://docs.ros.org/kinetic/api/libcreate/html/index.html)
* Protocol documentation:
  * [`V_3`](https://cdn-shop.adafruit.com/datasheets/create_2_Open_Interface_Spec.pdf) (Create 2, Roomba 600-800 series)
* Author: [Jacob Perron](http://jacobperron.ca) ([Autonomy Lab](http://autonomylab.org), [Simon Fraser University](http://www.sfu.ca))
* Contributors: [Mani Monajjemi](http:mani.im), [Ben Wolsieffer](https://github.com/lopsided98)

## Build status ##

![Build Status](https://api.travis-ci.org/RoboticaUtnFrba/libcreate.svg?branch=master)
[![GitHub issues](https://img.shields.io/github/issues-raw/RoboticaUtnFrba/libcreate)](https://github.com/RoboticaUtnFrba/libcreate/issues)
[![GitHub](https://img.shields.io/github/license/RoboticaUtnFrba/libcreate)](https://github.com/RoboticaUtnFrba/libcreate/blob/master/LICENSE)

## Dependencies ##

* [Boost System Library](http://www.boost.org/doc/libs/1_59_0/libs/system/doc/index.html)
* [Boost Thread Library](http://www.boost.org/doc/libs/1_59_0/doc/html/thread.html)

### Install ###

```bash
sudo apt-get install build-essential cmake libboost-system-dev libboost-thread-dev
```

#### Serial Permissions ####

User permission is requried to connect to Create over serial. You can add your user to the dialout group to get permission:

```bash
sudo usermod -a -G dialout $USER
```

Logout and login again for this to take effect.

## Build ##

Note, the examples found in the "examples" directory are built with the library.

### catkin ###

```bash
mkdir -p create_ws/src
cd create_ws
catkin init
cd src
git clone https://github.com/RoboticaUtnFrba/libcreate.git
catkin_make
```

## Known Issues ##

* _Clock_ and _Schedule_ buttons are not functional. This is a known bug related to the firmware.
