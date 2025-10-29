## Robots Package - Turtle, Plucky, Dragger, Seggy

This is an on-board software, normally residing under Ubuntu Server on a Raspberry Pi. It runs under ROS2 Jazzy *base*.

Its Gazebo Simulation side runs on a Ubuntu 24.04 desktop (no robot hardware required) - see [ROS2 Jazzy Setup](https://github.com/slgrobotics/robots_bringup/tree/main/Docs/ROS-Jazzy)

For actual physical robots look at https://github.com/slgrobotics/robots_bringup

The idea behind this package is to create a working, well-organized "_code sample_", which could be easily ammended with more robots.

To see how different robots co-exist within this package, look into the _robots_ folder.

Adding a new robot should be as easy as copying an existing folder (e.g. _plucky_), renaming the copy and modifying files to fit your needs.

**Tip:** This project is in active development. The _main_ branch is kept as stable as possible. For the latest code, use _dev_ branch.

-----------------------

### Purpose and Limitations of this Project

This software is not ready for production use. It has neither been developed nor tested for a specific use case. However, the license conditions of the applicable Open Source licenses allow you to adapt the software to your needs. Before using it in a safety relevant setting, make sure that the software fulfills your requirements and adjust it according to any applicable safety standards, e.g., ISO 26262.

-----------------------

### Credits:

1. The inspiration (and the origin of some code here) is _Articulated Robotics_ by Josh Newans, a mechatronics engineer from Newcastle, Australia. Here is his work:
- https://articulatedrobotics.xyz/category/getting-ready-to-build-a-ros-robot

2. This project wouldn't be possible without expert help and inspiration from members of:
- [HomeBrew Robotics Club](https://www.hbrobotics.org/)
- [Robotics Society ​of Southern California](https://www.rssc.org/)

-----------------------

**Note:** As of January 7, 2025 this repository is "_unforked_" (detached) from the [original template](https://github.com/joshnewans/articubot_one).