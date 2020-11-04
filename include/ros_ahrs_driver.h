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

class ROSAHRSDriver : public AHRSDriver
{
public:
  ROSAHRSDriver(ros::NodeHandle parent_pub_nh, const std::string &imu_topic = NO_TOPIC, const std::string &odom_topic = NO_TOPIC); // Default constructor
  virtual ~ROSAHRSDriver(); // Destructor

  // Implement the abstract AHRSDriver interface
  bool receiveLatestData(AHRSDataSet &data_out) override;
  void overrideHeadingData(float heading_deg, bool heading_true_north) override;

  void setIMUSubscription(ros::NodeHandle parent_pub_nh, const std::string &imu_topic);
  void setOdomSubscription(ros::NodeHandle parent_pub_nh, const std::string &odom_topic);

private:
  const int NAV_POS_SYNC_QUEUE_SIZE = 50;
  const static std::string NO_TOPIC;

  std::mutex ahrs_data_mutex; // NAV PosMgr accesses this from its dedicated service thread
  AHRSDataSet latest_ahrs;

  message_filters::Subscriber<sensor_msgs::Imu> imu_sub;
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, nav_msgs::Odometry> ApproxNavPosSyncPolicy;
  message_filters::Synchronizer<ApproxNavPosSyncPolicy>* approx_nav_pos_sync = nullptr;

  void callbackIMUAndOdom(const sensor_msgs::ImuConstPtr& imu_msg, const nav_msgs::OdometryConstPtr& odom_msg);

};

} // namespace Numurus
