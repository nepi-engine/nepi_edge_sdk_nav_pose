/*
 * NEPI Dual-Use License
 * Project: nepi_edge_sdk_nav_pose
 *
 * This license applies to any user of NEPI Engine software
 *
 * Copyright (C) 2023 Numurus, LLC <https://www.numurus.com>
 * see https://github.com/numurus-nepi/nepi_edge_sdk_nav_pose
 *
 * This software is dual-licensed under the terms of either a NEPI software developer license
 * or a NEPI software commercial license.
 *
 * The terms of both the NEPI software developer and commercial licenses
 * can be found at: www.numurus.com/licensing-nepi-engine
 *
 * Redistributions in source code must retain this top-level comment block.
 * Plagiarizing this software to sidestep the license obligations is illegal.
 *
 * Contact Information:
 * ====================
 * - https://www.numurus.com/licensing-nepi-engine
 * - mailto:nepi@numurus.com
 *
 */
#ifndef GPSD_ROS_CLIENT_H
#define GPSD_ROS_CLIENT_H

#include <string>

#include "sdk_node.h"

class gpsmm; // Forward declaration

namespace Numurus
{

class GPSDRosClient : public SDKNode
{
public:
	GPSDRosClient();
	virtual ~GPSDRosClient();

	// Inherited from SDKNode
	//void init() override;
  void retrieveParams() override;
  void initPublishers() override;
	void run() override;

	void serviceGPSDOnce();

private:
  NodeParam<std::string> gpsd_ip;
  NodeParam<int> gpsd_port;
  NodeParam<std::string> gps_frame_id;
  NodeParam<std::string> orientation_frame_id;
  NodeParam<bool> provides_orientation;

  // Publishers
  ros::Publisher gps_fix_pub;
  ros::Publisher heading_pub;
  ros::Publisher orientation_pub;
  ros::Publisher gps_stream_pub;

  // GPSD internals
  gpsmm *gps_rec = nullptr;
};

} // namespace

#endif // GPSD_ROS_CLIENT_H
