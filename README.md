<!--
Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.

This file is part of nepi-engine
(see https://github.com/nepi-engine).

License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
-->
# nepi_edge_sdk_nav_pose #

This repository hosts the NEPI nav_pose_mgr ROS node to support GPS and IMU integration for NEPI devices. It also includes the submodule _nepi_gpsd_, which is a lightly modified GPSD fork to include some custom NMEA sentences and a GPSD-to-ROS bridge node to convert GPSD-served nav and pose to ROS topics and services.

### Build and Install ###
This repository is typically built as part of the [nepi_engine_ws](https://github.com/nepi-engine/nepi_engine_ws) catkin workspace. Refer to that project's top-level README for build and install instructions.

The repository may be included in other custom catkin workspaces or built using standard CMake commands (with appropriate variable definitions provided), though it should be noted that the executables here work in concert with a complete NEPI Engine installation, so operability and functionality may be limited when building outside of the [nepi_engine_ws](https://github.com/nepi-engine/nepi_engine_ws) source tree.

### Branching and Tagging Strategy ###
In general branching, merging, tagging are all limited to the [nepi_engine_ws](https://github.com/nepi-engine/nepi_engine_ws) container repository, with the present submodule repository kept as simple and linear as possible.

### Contribution guidelines ###
Bug reports, feature requests, and source code contributions (in the form of pull requests) are gladly accepted!

### Who do I talk to? ###
At present, all user contributions and requests are vetted by Numurus technical staff, so you can use any convenient mechanism to contact us.
