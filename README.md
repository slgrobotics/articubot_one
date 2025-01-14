## Robot Package - Turtle, Plucky, Dragger

This is an on-board software, normally residing under Ubuntu Server on a Raspberry Pi. It runs under ROS2 Jazzy *base*.

Its Gazebo Simulation side runs on a Ubuntu 24.04 desktop (no robot hardware required) - see [ROS2 Jazzy Setup](https://github.com/slgrobotics/robots_bringup/tree/main/Docs/ROS-Jazzy)

For actual physical robots look at https://github.com/slgrobotics/robots_bringup

The idea behind this package is to create a working well-organized "code sample", which could be easily ammended with more robots.

To see how different robots co-exist within this package, look into the _robots_ folder.

Adding a new robot should be as easy as copying an existing folder (e.g. _plucky_), renaming the copy and modifying files to fit your needs.

-----------------------

The inspiration (and the origin of some code here) is _Articulated Robotics_ by Josh Newans, a mechatronics engineer from Newcastle, Australia.

Here is his work: https://articulatedrobotics.xyz/category/getting-ready-to-build-a-ros-robot

-----------------------

**Note:** As of January 7, 2025 this repository is "_unforked_" (detached) from the [original template](https://github.com/joshnewans/articubot_one).
