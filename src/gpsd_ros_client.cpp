#include <stdlib.h>

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "num_sdk_msgs/Heading.h"

// Important that this one comes after sensor_msgs/NavSatFix.h because
// they have conflicting definitions for STATUS_NO_FIX
#include <libgpsmm.h>
//#include <gps.h>

#include "gpsd_ros_client.h"

#define NODE_NAME	"gpsd_ros_client"

static constexpr int GPSD_PORT_DEFAULT = 2947;
static constexpr auto SERVICE_ONCE_BLOCK_TIME{100000}; // 100ms

#define DEG_TO_RAD(angle_deg) ((angle_deg) * M_PI / 180.0f)

namespace Numurus
{

GPSDRosClient::GPSDRosClient() :
  gpsd_ip{"gpsd_ip", "localhost", this},
  gpsd_port{"gpsd_port", GPSD_PORT_DEFAULT, this},
  gps_frame_id{"gps_frame_id", "gps_frame", this},
  attitude_frame_id{"attitude_frame_id", "ahrs_frame", this},
  provides_attitude{"provides_attitude", true, this}
{
  init(); // RAII

  // Now that retrieveParams has run via init() we can start the gpsd interface
  const std::string ip = gpsd_ip;
  uint16_t port = gpsd_port;
  const std::string port_str = std::to_string(port);
  gps_rec = new gpsmm(ip.c_str(), port_str.c_str());
  if (nullptr == gps_rec)
  {
    ROS_FATAL("Unable to allocate gps_rec [%s:%s]", ip.c_str(), port_str.c_str());
    exit(1);
  }

  const int gpsd_timeout = 5;
  for(int i = 0; i < gpsd_timeout; ++i)
  {
    if (gps_rec->stream(WATCH_ENABLE | WATCH_JSON) == nullptr)
    {
      ROS_WARN("GPSD server [%s:%s] not yet running... %d seconds remain before timeout", ip.c_str(), port_str.c_str(), gpsd_timeout - i);
      ros::Duration(1.0).sleep();
    }
    else
    {
      return; // Exit now that we have a connection to gpsd
    }
  }

  // Only get here on timeout
  ROS_FATAL("No gpsd server detected [%s:%s]", ip.c_str(), port_str.c_str());
  exit(1);
}

GPSDRosClient::~GPSDRosClient()
{
  if (nullptr != gps_rec)
  {
    delete gps_rec;
  }
}

void GPSDRosClient::run()
{
  // Spin at the current rate
  while (ros::ok())
  {
    ros::Rate r(current_rate_hz);
    ros::spinOnce();
    serviceGPSDOnce();
    r.sleep();
  }
}

void GPSDRosClient::retrieveParams()
{
  SDKNode::retrieveParams();
  gpsd_ip.retrieve();
  gpsd_port.retrieve();
  gps_frame_id.retrieve();
  attitude_frame_id.retrieve();
  provides_attitude.retrieve();
}

void GPSDRosClient::initPublishers()
{
  gps_fix_pub = n.advertise<sensor_msgs::NavSatFix>("set_gps_fix", 3);
  heading_pub = n.advertise<num_sdk_msgs::Heading>("nav_pos_mgr/set_heading_override", 3);
  attitude_pub = n.advertise<geometry_msgs::QuaternionStamped>("nav_pos_mgr/set_attitude_override", 3);
  gps_stream_pub = n.advertise<std_msgs::String>("gps_status_stream", 3);
}

static int8_t gpsdFixStatusToRosFixStatus(int gpsd_fix_status)
{
  if ((STATUS_NO_FIX == gpsd_fix_status) || (STATUS_TIME == gpsd_fix_status))
  {
    return -1; // Can't use symbolic sensor_msgs::NavSatStatus::STATUS_NO_FIX because it conflicts with GPSD #define
  }

  // Just assign STATUS_FIX to everything else -- no obvious mapping here
  return 0; // Can't use symbolic sensor_msgs::NavSatStatus::STATUS_FIX because it conflicts with GPSD #define
}

void GPSDRosClient::serviceGPSDOnce()
{
  if (false == gps_rec->waiting(SERVICE_ONCE_BLOCK_TIME))
  {
    return;
  }

  struct gps_data_t *gpsd_data = gps_rec->read();
  if (gpsd_data == nullptr)
  {
    ROS_ERROR_THROTTLE(1, "Unexpected GPSD read failure");
    return;
  }

  if (gpsd_data->fix.mode < MODE_2D)
  {
    // No big deal, just come back later and try again
  }

  // Now interpret the results and publish as ROS
  // First GPS
  if ((gpsd_data->set & LATLON_SET) || (gpsd_data->set & ALTITUDE_SET))
  {
    sensor_msgs::NavSatFix nav_sat_fix_msg;
    nav_sat_fix_msg.header.frame_id = gps_frame_id;
    if (gpsd_data->set & TIME_SET)
    {
      nav_sat_fix_msg.header.stamp = ros::Time(gpsd_data->fix.time.tv_sec, gpsd_data->fix.time.tv_nsec);
    }
    else
    {
      // TODO: What should we do for no fix time
      nav_sat_fix_msg.header.stamp = ros::Time::now();
    }

    nav_sat_fix_msg.status.status = gpsdFixStatusToRosFixStatus(gpsd_data->fix.status);
    // No way to get the service (GPS, GLONASS, etc.) from gpsd as far as I can tell, so just hard-code it as GPS for now
    nav_sat_fix_msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;

    if (gpsd_data->set & LATLON_SET)
    {
      //ROS_WARN("Debug - Got lat/lon: [%f, %f]", gpsd_data->fix.latitude, gpsd_data->fix.longitude);
      nav_sat_fix_msg.latitude = gpsd_data->fix.latitude;
      nav_sat_fix_msg.longitude = gpsd_data->fix.longitude;
    }

    if (gpsd_data->set & ALTITUDE_SET)
    {
      //ROS_WARN("Debug - Got altitude: %f", gpsd_data->fix.altHAE);
      nav_sat_fix_msg.altitude = gpsd_data->fix.altHAE;
    }

    gps_fix_pub.publish(nav_sat_fix_msg);
  }

  // Now heading and attitude
  if (gpsd_data->set & ATTITUDE_SET)
  {
    //ROS_WARN("Debug - Got heading: %f", gpsd_data->attitude.heading);
    num_sdk_msgs::Heading heading_msg;
    heading_msg.heading = gpsd_data->attitude.heading;
    heading_msg.true_north = true;
    heading_pub.publish(heading_msg);

    // TODO: How can we differentiate between when heading set and when complete attitude (e.g., HPR or PAUV sentence?)
    // For now, just differentiate as a config option
    if (true == provides_attitude)
    {
      geometry_msgs::QuaternionStamped quat_msg;
      quat_msg.header.stamp = ros::Time(gpsd_data->attitude.mtime.tv_sec, gpsd_data->attitude.mtime.tv_nsec);
      quat_msg.header.frame_id = attitude_frame_id;

      tf2::Quaternion q;
      q.setRPY(DEG_TO_RAD(gpsd_data->attitude.roll), DEG_TO_RAD(gpsd_data->attitude.pitch), 0.0); // No yaw in these messages
      tf2::convert(q, quat_msg.quaternion);

      //ROS_WARN("Debug - Got RP(Y): [%f, %f, (0.0)] - as quaternion: [%f, %fi, %fj, %fk]",
      //         gpsd_data->attitude.roll, gpsd_data->attitude.pitch,
      //         quat_msg.quaternion.w, quat_msg.quaternion.x, quat_msg.quaternion.y, quat_msg.quaternion.z);

      attitude_pub.publish(quat_msg);
    }
  }
}

} // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, NODE_NAME);
  ROS_INFO("Starting the %s node", NODE_NAME);

  Numurus::GPSDRosClient client;
  client.run();

  return 0;
}
