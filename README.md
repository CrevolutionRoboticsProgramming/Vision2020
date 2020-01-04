# Vision2020

### Crevolution's vision program for 2020

## Overview

This is a vision processing program created for the 2020 FRC game, Infinite Recharge. It can identify the game's vision target and send its horizontal offset to a connected roboRIO via UDP. Additionally, it is entirely configurable using the [CrevoDashboard](https://github.com/CrevolutionRoboticsProgramming/CrevoDashboard) JavaFX GUI.

## Requirements

Hardware requirements include:
* A Raspberry Pi (we used a Pi 3 B+)
* A Raspberry Pi Camera Module (we used [this infrared camera](https://www.amazon.com/Haiworld-Raspberry-Camera-Infrared-Megapixel/dp/B01MYUOQ0A) (the lights are included))
* A normal UVC camera (we used [this fisheye camera](https://www.amazon.com/180degree-Fisheye-Camera-usb-Android-Windows/dp/B00LQ854AG))

Software requirements include:
* [GStreamer-1.0](https://gstreamer.freedesktop.org/) and all related libraries and packages
* [gst-rpicamsrc](https://github.com/thaytan/gst-rpicamsrc)
* [OpenCV 3.4.2](https://github.com/opencv/opencv/archive/3.4.2.zip) installed with GStreamer support
* [Boost 1.58.0](https://sourceforge.net/projects/boost/files/boost/1.58.0/) (only the system module is used)
* [mjpg-streamer](https://github.com/jacksonliam/mjpg-streamer)
* [yaml-cpp](https://github.com/jbeder/yaml-cpp/)

## Installing

This assumes you have a Raspberry Pi already up and running. If you don't, I would recommend following [this tutorial](https://www.tomshardware.com/reviews/raspberry-pi-headless-setup-how-to,6028.html). Don't forget to enable the Raspberry Pi Camera Module in ```raspiconfig```.

Please make sure all software prerequisites are installed before continuing.

1. Open a new terminal
2. Install git (```sudo apt install git```)
3. Run ```cd ~/```
4. Run ```git clone https://github.com/CrevolutionRoboticsProgramming/Vision2020```
5. Run ```cd Vision2020```
6. Run ```cmake .```
7. Run ```make```
8. To run the program, run ```./run.sh```

If you would like to run the program on startup, follow these steps:
1. Open a new terminal
2. Run ```sudo nano /etc/rc.local```
3. Before ```exit 0```, add a new line with the contents ```cd ~/Vision2020 && ./run.sh &```
4. Press ```Ctrl + O``` to write your changes
5. Press ```Ctrl + X``` to exit

## Usage

Once ran, the program requires no user input. It includes a custom configuration protocol for storing and updating program-wide values like HSV thresholds and video formatting which can be accessed via our [CrevoDashboard](https://github.com/CrevolutionRoboticsProgramming/CrevoDashboard) JavaFX GUI.

Once the program identifies a vision target, it calculates its horizontal offset from the center of the target and streams it via UDP without labelling to the roboRIO. The stream can be received with the UDPHandler class included in [CrevoLib](https://github.com/CrevolutionRoboticsProgramming/RobotCode2020).

The video stream can be received from [index.html](../master/index.html) in any web browser.

## Additional Acknowledgements

The MJPEGWriter.cpp and MJPEGWriter.hpp files come from [JPery's MJPEGWriter](https://github.com/JPery/MJPEGWriter) and are used for transmitting the video stream. They have been altered to fit the needs of the project.
