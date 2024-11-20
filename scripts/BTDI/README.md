## Humble BTDI structure

The Humble BTDI is built from **Ubuntu 22.04**.

Main components of the BTDI are:

- [**ROS2 Humble**](https://docs.ros.org/en/humble/index.html) 
- [**RViz**](https://github.com/ros2/rviz), Simulations
- [**Gazebo-11**](http://gazebosim.org/), Simulations
- **Python 3.10**, main libraries:
    - [websocket_server](https://pypi.org/project/websocket-server/)
    - [Django](https://docs.djangoproject.com/en/4.2/releases/4.1/)
    - [websockets](https://pypi.org/project/websockets/)
    - [asyncio](https://pypi.org/project/asyncio/)
- **VirtualGL**, GPU acceleration
- **TurboVNC**, VNC Server
- **noVNC**, VNC Client web application 
- [**Xvfb**](https://www.x.org/releases/X11R7.6/doc/man/man1/Xvfb.1.xhtml), XServer for loading images from Gazebo camera
- [**NodeJs**](https://nodejs.org/en/docs)
	- [ReactJS](https://react.dev/learn), Exercise Frontend
- [**PX4**](https://github.com/PX4/PX4-Autopilot), Drone simulation.
    - [Micro-XRCE-DDS-Agent](https://github.com/eProsima/Micro-XRCE-DDS-Agent.git)
- [**AeroStack2**](https://aerostack2.github.io/), Drone Algorithms and maneuverability
- [**Bt-Studio**](https://github.com/JdeRobot/bt-studio)
- [**RoboticsInfrastructure**](https://github.com/JdeRobot/RoboticsInfrastructure)
- [**RoboticsApplicationManager**](https://github.com/JdeRobot/RoboticsApplicationManager)
