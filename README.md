<!--
NEPI Dual-Use License
Project: nepi_edge_sdk_ai

This license applies to any user of NEPI Engine software

Copyright (C) 2023 Numurus, LLC <https://www.numurus.com>
see https://github.com/numurus-nepi/nepi_edge_sdk_ai

This software is dual-licensed under the terms of either a NEPI software developer license
or a NEPI software commercial license.

The terms of both the NEPI software developer and commercial licenses
can be found at: www.numurus.com/licensing-nepi-engine

Redistributions in source code must retain this top-level comment block.
Plagiarizing this software to sidestep the license obligations is illegal.

Contact Information:
====================
- https://www.numurus.com/licensing-nepi-engine
- mailto:nepi@numurus.com

-->
# nepi_edge_sdk_nav_pose #

This repository hosts the NEPI nav_pose_mgr ROS node to support GPS and IMU integration for NEPI devices. It also includes the submodule _num_gpsd_, which is a lightly modified GPSD fork to include some custom NMEA sentences and a GPSD-to-ROS bridge node to convert GPSD-served nav and pose to ROS topics and services.

### Build and Install ###
This repository is typically built as part of the _nepi_base_ws_ catkin workspace. Refer to that project's top-level README for build and install instructions.

The repository may be included in other custom catkin workspaces or built using standard CMake commands (with appropriate variable definitions provided), though it should be noted that the executables here work in concert with a complete NEPI Engine installation, so operability and functionality may be limited when building outside of the _nepi_base_ws_ source tree.

### Branching and Tagging Strategy ###
In general branching, merging, tagging are all limited to the _nepi_base_ws_ container repository, with the present submodule repository kept as simple and linear as possible.

### Contribution guidelines ###
Bug reports, feature requests, and source code contributions (in the form of pull requests) are gladly accepted!

### Who do I talk to? ###
At present, all user contributions and requests are vetted by Numurus technical staff, so you can use any convenient mechanism to contact us.