#include "lord_ahrs_ros_wrapper.h"
#include "num_sdk_msgs/Heading.h"

#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Quaternion.h"

#define NODE_NAME	"lord_ahrs"

#define DEG2RAD   0.0174533f

namespace Numurus
{

LORDAHRSRosWrapper::LORDAHRSRosWrapper() :
  ahrs_serial_port{"ahrs_serial_port", "", this},
  publish_rate_hz{"publish_rate_hz", 10, this},
  ahrs_x_rot_offset_deg{"ahrs_x_rot_offset_deg", 0.0f, this},
  ahrs_y_rot_offset_deg{"ahrs_y_rot_offset_deg", 0.0f, this},
  ahrs_z_rot_offset_deg{"ahrs_z_rot_offset_deg", 0.0f, this}
{}

LORDAHRSRosWrapper::~LORDAHRSRosWrapper()
{}

void LORDAHRSRosWrapper::init()
{
  SDKNode::init(); // call the base class method first

  // Now the config variables are populated, so we can use them to initialize the driver
  const float roll_deg = ahrs_x_rot_offset_deg;
  const float pitch_deg = ahrs_y_rot_offset_deg;
  const float yaw_deg = ahrs_z_rot_offset_deg;
  const Numurus::AHRSRollPitchYaw sensor_to_vehicle_rpy_rad(DEG2RAD*roll_deg, DEG2RAD*pitch_deg, DEG2RAD*yaw_deg);
  const float ahrs_update_rate_hz = 2.0 * publish_rate_hz; // Twice as fast as we plan to publish
  const std::time_t now_s = std::time(0);

  const std::string serial_port = ahrs_serial_port;
  if (false == driver.init(serial_port.c_str(), ahrs_update_rate_hz, now_s, sensor_to_vehicle_rpy_rad))
  {
    ROS_ERROR("Failed to initialize the LORD AHRS driver");
  }
}

void LORDAHRSRosWrapper::run()
{
  // Do init() if it hasn't already been done
  if (false == _initialized)
  {
    init();
  }

  // Spin at the current publish rate
  const ros::Duration publish_period(1.0f / publish_rate_hz);

  while (ros::ok())
  {
    ros::spinOnce();
    publishIMUandOdom();
    publish_period.sleep();
  }
}

void LORDAHRSRosWrapper::retrieveParams()
{
	SDKNode::retrieveParams();
	ahrs_serial_port.retrieve();
  publish_rate_hz.retrieve();
  ahrs_x_rot_offset_deg.retrieve();
  ahrs_y_rot_offset_deg.retrieve();
  ahrs_z_rot_offset_deg.retrieve();
}

void LORDAHRSRosWrapper::initPublishers()
{
  imu_pub = n_priv.advertise<sensor_msgs::Imu>("~imu", 10);
  odom_pub = n_priv.advertise<nav_msgs::Odometry>("~odom", 10);
  heading_pub = n.advertise<num_sdk_msgs::Heading>("nav_pos_mgr/set_heading_override", 10);
}

void LORDAHRSRosWrapper::publishIMUandOdom()
{
  static uint32_t seq = 0;
  Numurus::AHRSDataSet data;
  if (false == driver.receiveLatestData(data))
  {
    ROS_WARN_THROTTLE(1, "Unable to get data from LORD AHRS driver... not publishing IMU and Odom");
    return;
  }

  if (data.filter_state != AHRS_FILTER_STAT_RUN_VAL)
  {
    ROS_WARN_THROTTLE(1, "Not publishing LORD AHRS data because filter is not running");
    return;
  }

  std_msgs::Header common_header;
  common_header.seq = seq++;
  const ros::Time tstamp_ros(data.timestamp);
  common_header.stamp = tstamp_ros;
  common_header.frame_id = "lord_ahrs_frame";

  geometry_msgs::Quaternion common_quaternion;
  if (data.orientation_valid)
  {
    common_quaternion.x = data.orientation_q1_i;
    common_quaternion.y = data.orientation_q2_j;
    common_quaternion.z = data.orientation_q3_k;
    common_quaternion.w = data.orientation_q0;
  }

  sensor_msgs::Imu imu_msg;
  imu_msg.header = common_header;
  if (data.orientation_valid)
  {
    imu_msg.orientation = common_quaternion;
  }
  if (data.angular_velocity_valid)
  {
    imu_msg.angular_velocity.x = data.angular_velocity_x;
    imu_msg.angular_velocity.y = data.angular_velocity_y;
    imu_msg.angular_velocity.z = data.angular_velocity_z;
  }
  if (data.accel_valid)
  {
    imu_msg.linear_acceleration.x = data.accel_x;
    imu_msg.linear_acceleration.y = data.accel_y;
    imu_msg.linear_acceleration.z = data.accel_z;
  }

  nav_msgs::Odometry odom_msg;
  odom_msg.header = common_header;
  if (data.orientation_valid)
  {
    odom_msg.pose.pose.orientation = common_quaternion;
  }
  if (data.velocity_valid)
  {
    odom_msg.twist.twist.linear.x = data.velocity_x;
    odom_msg.twist.twist.linear.y = data.velocity_y;
    odom_msg.twist.twist.linear.z = data.velocity_z;
  }

  // Publish IMU and Odom close together because nav_pos mgr has a dual-filter
  imu_pub.publish(imu_msg);
  odom_pub.publish(odom_msg);

  if (data.heading_valid)
  {
    num_sdk_msgs::Heading heading_msg;
    heading_msg.heading = data.heading;
    heading_msg.true_north = data.heading_true_north;
    heading_pub.publish(heading_msg);
  }
}

} // namespace Numurus

int main(int argc, char **argv)
{
	ros::init(argc, argv, NODE_NAME);
	ROS_INFO("Starting the %s node", NODE_NAME);

	Numurus::LORDAHRSRosWrapper lord_ahrs;
	lord_ahrs.run();

	return 0;
}
