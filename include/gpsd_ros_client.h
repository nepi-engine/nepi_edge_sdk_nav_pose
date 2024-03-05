/*
 * Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
 *
 * This file is part of nepi-engine
 * (see https://github.com/nepi-engine).
 *
 * License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
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
