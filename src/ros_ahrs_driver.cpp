#include <ros_ahrs_driver.h>

namespace Numurus
{
const std::string ROSAHRSDriver::NO_TOPIC = "None";

ROSAHRSDriver::ROSAHRSDriver(ros::NodeHandle parent_pub_nh, const std::string &imu_topic, const std::string &odom_topic)
{
  // Set up the message_filter subscribers
  setIMUSubscription(parent_pub_nh, imu_topic);
  setOdomSubscription(parent_pub_nh, odom_topic);

  approx_nav_pos_sync = new message_filters::Synchronizer<ApproxNavPosSyncPolicy>(ApproxNavPosSyncPolicy(NAV_POS_SYNC_QUEUE_SIZE),
                                                                                  imu_sub, odom_sub);
  approx_nav_pos_sync->registerCallback(boost::bind(&ROSAHRSDriver::callbackIMUAndOdom, this, _1, _2));
}

ROSAHRSDriver::~ROSAHRSDriver()
{
  if (nullptr != approx_nav_pos_sync) delete approx_nav_pos_sync;
}

bool ROSAHRSDriver::receiveLatestData(AHRSDataSet &data_out)
{
  // Simply copy (under mutex lock because this is called from a separate thread)
  {
    std::lock_guard<std::mutex> lk(ahrs_data_mutex);
    data_out = latest_ahrs;
  }
  //ROS_INFO("Debugging: Leaving receiveLatestData");
  return true;
}

void ROSAHRSDriver::overrideHeadingData(float heading_deg, bool heading_true_north)
{
  std::lock_guard<std::mutex> lk(ahrs_data_mutex);
  heading_override = true;
  heading_override_deg = heading_deg;
  heading_override_true_north = heading_true_north;
}


void ROSAHRSDriver::callbackIMUAndOdom(const sensor_msgs::ImuConstPtr& imu_msg, const nav_msgs::OdometryConstPtr& odom_msg)
{
  //ROS_INFO("Debugging: Got IMU and Odometry");

  std::lock_guard<std::mutex> lk(ahrs_data_mutex);
  // Fill out the latest_ahrs according to this combination
  latest_ahrs.timestamp = odom_msg->header.stamp.toSec(); // Odometry is the slower rate topic, so use its timestamp
  latest_ahrs.filter_state = AHRS_FILTER_STAT_RUN_VAL; // Always mark it as running, since we're receiving data
  //latest_ahrs.filter_flags = 0; // Unused in the ROSAHRSDriver

  // Linear Accelerations (m/s^2), frame transformation applied
  latest_ahrs.accel_x = imu_msg->linear_acceleration.x;
  latest_ahrs.accel_y = imu_msg->linear_acceleration.y;
  latest_ahrs.accel_z = imu_msg->linear_acceleration.z;
  latest_ahrs.accel_valid = true;

  // Linear Velocity (m/s), frame transformation applied
  latest_ahrs.velocity_x = odom_msg->twist.twist.linear.x;
  latest_ahrs.velocity_y = odom_msg->twist.twist.linear.y;
  latest_ahrs.velocity_z = odom_msg->twist.twist.linear.z;

  // Angular Velocity (rad/s), frame transformation applied
  /*
  latest_ahrs.angular_velocity_x = odom_msg->twist.twist.angular.x;
  latest_ahrs.angular_velocity_y = odom_msg->twist.twist.angular.y;
  latest_ahrs.angular_velocity_z = odom_msg->twist.twist.angular.z;
  */
  latest_ahrs.angular_velocity_x = imu_msg->angular_velocity.x;
  latest_ahrs.angular_velocity_y = imu_msg->angular_velocity.y;
  latest_ahrs.angular_velocity_z = imu_msg->angular_velocity.z;
  latest_ahrs.angular_velocity_valid = true;

  // Orientation (quaterion) w.r.t. fixed-earth coordinate frame
  /*
  latest_ahrs.orientation_q0 = odom_msg->pose.pose.orientation.w;
  latest_ahrs.orientation_q1_i = odom_msg->pose.pose.orientation.x;
  latest_ahrs.orientation_q2_j = odom_msg->pose.pose.orientation.y;
  latest_ahrs.orientation_q3_k = odom_msg->pose.pose.orientation.z;
  */
  latest_ahrs.orientation_q0 = imu_msg->orientation.w;
  latest_ahrs.orientation_q1_i = imu_msg->orientation.x;
  latest_ahrs.orientation_q2_j = imu_msg->orientation.y;
  latest_ahrs.orientation_q3_k = imu_msg->orientation.z;
  latest_ahrs.orientation_valid = true;

  // Heading (deg)
  if (true == heading_override)
  {
    latest_ahrs.heading = heading_override_deg;
    latest_ahrs.heading_true_north = heading_override_true_north;
    latest_ahrs.heading_valid = true;
  }
  else // No heading source yet, though we should support when a magnetometer is available -- Just zero it for now
  {
    latest_ahrs.heading = 0.0f;
    latest_ahrs.heading_true_north = false;
    latest_ahrs.heading_valid = false;
  }
}

void ROSAHRSDriver::setIMUSubscription(ros::NodeHandle parent_pub_nh, const std::string &imu_topic)
{
  if (imu_topic != NO_TOPIC)
  {
    imu_sub.subscribe(parent_pub_nh, ros::this_node::getNamespace() + '/' + imu_topic, 1);
  }
}

void ROSAHRSDriver::setOdomSubscription(ros::NodeHandle parent_pub_nh, const std::string &odom_topic)
{
  if (odom_topic != NO_TOPIC)
  {
    odom_sub.subscribe(parent_pub_nh, ros::this_node::getNamespace() + '/' + odom_topic, 1);
  }
}

} // namespace Numurus
