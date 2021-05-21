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
  NodeParam<std::string> attitude_frame_id;
  NodeParam<bool> provides_attitude;

  // Publishers
  ros::Publisher gps_fix_pub;
  ros::Publisher heading_pub;
  ros::Publisher attitude_pub;
  ros::Publisher gps_stream_pub;

  // GPSD internals
  gpsmm *gps_rec = nullptr;
};

} // namespace

#endif // GPSD_ROS_CLIENT_H
