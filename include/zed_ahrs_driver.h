#pragma once

#include <mutex>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include <ahrs_driver.h>

namespace Numurus
{

class ZedAHRSDriver : public AHRSDriver
{
public:
  ZedAHRSDriver(ros::NodeHandle parent_pub_nh, ros::NodeHandle parent_priv_nh); // Default constructor
  virtual ~ZedAHRSDriver(); // Destructor

  // Implement the abstract AHRSDriver interface
  bool receiveLatestData(AHRSDataSet &data_out) override;

private:
  const int NAV_POS_SYNC_QUEUE_SIZE = 50;
  const std::string ZED_NODELET_NAME = "3dx_device/stereo_cam_driver";

  std::mutex ahrs_data_mutex; // NAV PosMgr accesses this from its dedicated service thread
  AHRSDataSet latest_ahrs;

  message_filters::Subscriber<sensor_msgs::Imu> imu_sub;
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, nav_msgs::Odometry> ApproxNavPosSyncPolicy;
  message_filters::Synchronizer<ApproxNavPosSyncPolicy>* approx_nav_pos_sync = nullptr;

  void callbackIMUAndOdom(const sensor_msgs::ImuConstPtr& imu_msg, const nav_msgs::OdometryConstPtr& odom_msg);
};

} // namespace Numurus
