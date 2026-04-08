# Introduction

> **灵视 IVG / 本仓库部署说明**：现场与 `ros2 launch aubo_ros2_web_dashboard web_dashboard.launch.py` 使用的是 ROS 2 官方维护的 **[rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite)**（`sudo apt install ros-$ROS_DISTRO-rosbridge-suite`）。**请勿**按下文安装 Node.js **ros2-web-bridge**；下文仅为 2018 会议原版硬件流程存档。

Web demo app for ROS Conference 2018

## Hardware:
* [Intel Up-board Squared](http://www.up-board.org/upsquared/)
* [TurtleBot3 Burger](http://www.robotis.us/turtlebot-3/)
* USB Wi-Fi adpater
* An available wireless network
* Node.js 8.11.4 installed

## Pre-condition
* Ubuntu Linux 16.04 installed on Up-board Squared
* ROS2 release package available

## Setup
### Install MicroRTPSAgent
```
$ git clone https://github.com/eProsima/micro-RTPS-agent.git
$ cd micro-RTPS-agent
$ mkdir build && cd build
$ cmake -DTHIRDPARTY=ON ..
$ make
$ sudo make install
```

### Install ros2-web-bridge
```
$ source <path/to/ros2-linux>/local_setup.bash
$ git clone https://github.com/RobotWebTools/ros2-web-bridge.git
$ cd ros2-web-bridge
$ npm install
```

### Install http-server CLI tools
```
$ npm install -g http-server
```

## Run the demo
### Start up MicroRTPSAgent
New a terminal session, run
```
$ cd /usr/local/bin
$ ./MicroRTPSAgent serial /dev/ttyACM0
```

### Start up the web bridge server
New a terminal session, run
```
$ source <path/to/ros2-linux>/local_setup.bash
$ cd ros2-web-bridge
$ export DEBUG=ros2-web-bridge:*
$ node bin/rosbridge.js
```

### Host the web demo app
New a terminal session and run
```
$ cd ros2-web-bridge/demo
$ http-server -c-1
```

### Try to control the robot remotely
Now you can put the TurtleBot3 robot on the ground. Let another PC be in the same wireless network, then launch the browser and enter: `http://<up-board ip>:8080`. After the page loaded, you can press the `Up/Left/Right/Down` buttons to control the robot remotely.

## Known limitation:
If you shutdown the MicroRTPSAgent utility, you have to reset the OpenCR before you launch it again.
