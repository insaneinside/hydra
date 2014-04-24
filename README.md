# ROS stack for the DUBotics Hydra

blah blah blah I'm not in an expository mood right now, so I'll fill in the
intro later.

## Setup

This repository contains ROS packages.  To use these packages, clone the
packages into a Catkin workspace directory `src` inside some scratch directory
(`hydra` in this case):

````shell
mkdir hydra && \
  cd hydra && \
  git clone https://github.com/dubotics/hydra.git src && \
  catkin_init_workspace src
````

The final command, `catkin_init_workspace`, creates a link to a generic
system-installed ROS build file.  (Don't forget to load the setup file
[`/opt/ros/<ros_distro>/setup.*`] appropriate to your shell and ROS
installation prior to running that or any ROS-related commands.)

Run `catkin_make` from your scratch directory to build the packages.  Once the
build completes successfully, you can source the shell-specific `setup.*` file
from `devel/` to add the locations of the newly-built packages to the ROS
environment variables.

Now you should be able to run e.g.

````shell
roslaunch hydra_tests motor-test.launch
````

## Packages

### hydra_drive 

Manages drive control, mapping the appropriate inputs (manual or nav
stack-based) to the appropriate outputs (hardware via serial_node).

### hydra_tests

Random test nodes/launch files/etc.
