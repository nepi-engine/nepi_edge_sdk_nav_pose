#include <zed_ahrs_driver.h>

namespace Numurus
{


ZedAHRSDriver::ZedAHRSDriver(ros::NodeHandle parent_pub_nh, ros::NodeHandle parent_priv_nh)
{
  ros::NodeHandle zed_nh(parent_pub_nh, ZED_NODELET_NAME);
  ros::NodeHandle imu_nh(zed_nh, "imu");

  // Set up the message_filter subscribers
  imu_sub.subscribe(imu_nh, "data", 1);
  odom_sub.subscribe(zed_nh, "odom", 1);

  approx_nav_pos_sync = new message_filters::Synchronizer<ApproxNavPosSyncPolicy>(ApproxNavPosSyncPolicy(NAV_POS_SYNC_QUEUE_SIZE),
                                                                                  imu_sub, odom_sub);
  approx_nav_pos_sync->registerCallback(boost::bind(&ZedAHRSDriver::callbackIMUAndOdom, this, _1, _2));
}

ZedAHRSDriver::~ZedAHRSDriver()
{
  if (nullptr != approx_nav_pos_sync) delete approx_nav_pos_sync;
}

bool ZedAHRSDriver::receiveLatestData(AHRSDataSet &data_out)
{
  // Simply copy (under mutex lock because this is called from a separate thread)
  {
    std::lock_guard<std::mutex> lk(ahrs_data_mutex);
    data_out = latest_ahrs;
  }
  //ROS_INFO("Debugging: Leaving receiveLatestData");
  return true;
}

void ZedAHRSDriver::callbackIMUAndOdom(const sensor_msgs::ImuConstPtr& imu_msg, const nav_msgs::OdometryConstPtr& odom_msg)
{
  //ROS_INFO("Debugging: Got IMU and Odometry");
  // We apply a fixed transform here based on observation (odom and imu frames appear to be aligned)
  // zed_x --> -z, zed_y --> -y, zed_z --> x

  std::lock_guard<std::mutex> lk(ahrs_data_mutex);
  // Fill out the latest_ahrs according to this combination
  latest_ahrs.timestamp = odom_msg->header.stamp.toSec(); // Odometry is the slower rate topic, so use its timestamp
  latest_ahrs.filter_state = AHRS_FILTER_STAT_RUN_VAL; // Always mark it as running, since we're receiving data
  //latest_ahrs.filter_flags = 0; // Unused in the ZedAHRSDriver

  // Linear Accelerations (m/s^2), frame transformation applied
  latest_ahrs.accel_x = imu_msg->linear_acceleration.x;
  latest_ahrs.accel_y = -imu_msg->linear_acceleration.y;
  latest_ahrs.accel_z = -imu_msg->linear_acceleration.z;
  latest_ahrs.accel_valid = true;

  // Linear Velocity (m/s), frame transformation applied
  latest_ahrs.velocity_x = odom_msg->twist.twist.linear.x;
  latest_ahrs.velocity_y = -odom_msg->twist.twist.linear.y;
  latest_ahrs.velocity_z = -odom_msg->twist.twist.linear.z;

  // Angular Velocity (rad/s), frame transformation applied
  /*
  latest_ahrs.angular_velocity_x = odom_msg->twist.twist.angular.x;
  latest_ahrs.angular_velocity_y = odom_msg->twist.twist.angular.y;
  latest_ahrs.angular_velocity_z = odom_msg->twist.twist.angular.z;
  */
  latest_ahrs.angular_velocity_x = imu_msg->angular_velocity.x;
  latest_ahrs.angular_velocity_y = -imu_msg->angular_velocity.y;
  latest_ahrs.angular_velocity_z = -imu_msg->angular_velocity.z;
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
  latest_ahrs.orientation_q2_j = -imu_msg->orientation.y;
  latest_ahrs.orientation_q3_k = -imu_msg->orientation.z;
  latest_ahrs.orientation_valid = true;

  // Heading (deg)
  // No heading computation (yet, though there is a magnetometer, so possible) -- Just zero it.
  latest_ahrs.heading = 0.0f;
  latest_ahrs.heading_true_north = false;
  latest_ahrs.heading_valid = false;
}

} // namespace Numurus
