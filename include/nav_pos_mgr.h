#ifndef NAV_POS_MGR_H
#define NAV_POS_MGR_H

#include <thread>
#include <atomic>
#include <mutex>
#include <deque>

#include "tf/transform_listener.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/NavSatFix.h"
#include "num_sdk_msgs/Offset.h"
#include "num_sdk_msgs/Heading.h"

#include "sdk_node.h"
//#include "lord_ahrs_driver.h"
#include "drivers/generic_ahrs/ahrs_driver.h"
#include "num_sdk_msgs/NavPosQuery.h"
#include "num_sdk_msgs/NavPosStatusQuery.h"

namespace Numurus
{
//class LORDAHRSDriver; // forward declaration;
class AHRSDriver;
class SaveDataInterface;

class NavPosMgr : public SDKNode
{
public:
	NavPosMgr();
	virtual ~NavPosMgr();

	// Inherited from SDKNode
	void init() override;
	void retrieveParams() override;
	void initServices() override;
	void initSubscribers() override;

private:
	NodeParam<std::string> ahrs_comm_device;
	NodeParam<float> ahrs_update_rate_hz;
	NodeParam<bool> ahrs_offsets_set;
	NodeParam<float> ahrs_x_offset_m;
	NodeParam<float> ahrs_y_offset_m;
	NodeParam<float> ahrs_z_offset_m;
	NodeParam<float> ahrs_x_rot_offset_deg;
	NodeParam<float> ahrs_y_rot_offset_deg;
	NodeParam<float> ahrs_z_rot_offset_deg;
	NodeParam<std::string> ahrs_type;
	NodeParam<std::string> imu_topic;
	NodeParam<std::string> odom_topic;
	NodeParam<std::string> ahrs_src_frame_id;
	NodeParam<std::string> ahrs_out_frame_id;
	NodeParam<bool> lat_lon_alt_is_fixed;
	NodeParam<float> fixed_lat_deg;
	NodeParam<float> fixed_lon_deg;
	NodeParam<float> fixed_alt_m_hae;
	NodeParam<bool> orientation_is_fixed;
	NodeParam<float> fixed_orientation_x;
	NodeParam<float> fixed_orientation_y;
	NodeParam<float> fixed_orientation_z;
	NodeParam<float> fixed_orientation_w;
	NodeParam<bool> heading_is_fixed;
	NodeParam<float> fixed_heading_deg;
	NodeParam<bool> fixed_heading_is_true_north;
	bool ahrs_ready;
	std::thread *ahrs_rcv_thread;
	std::atomic<bool> ahrs_rcv_continue;
	std::deque<AHRSDataSet> ahrs_data_stack;
	std::mutex ahrs_data_stack_mutex;
	size_t ahrs_data_stack_max_size;

	AHRSDriver *ahrs = nullptr;
	static constexpr double AHRS_DRIVER_SERVICE_RATE = 100; //Hz

	std::atomic<bool> update_ahrs_nav_sat_fix;
	std::mutex latest_nav_sat_fix_mutex;
	sensor_msgs::NavSatFix latest_nav_sat_fix;

	SaveDataInterface *save_data_if = nullptr;

	tf::TransformListener transform_listener;
	tf2_ros::StaticTransformBroadcaster static_transform_broadcaster;

	bool need_new_data_file = true;
	FILE *data_fd = nullptr;

	/**
	 * @brief      Provide the (interpolated) NavPos response for the requested timestamp
	 *
	 *			   This method serves as the callback for the nav_pos_query service
	 *
	 * @param      req   The requested timestamp. Use 0 to indicate that the most recent fix is requested
	 * @param      resp  The NavPos response.
	 *
	 * @return     true if successful, false otherwise
	 */
	bool provideNavPos(num_sdk_msgs::NavPosQuery::Request &req, num_sdk_msgs::NavPosQuery::Response &resp);

	/**
	 * @brief      Provide status information about the nav/pos/time subsystem
	 *
	 *			   This method serves as the callback for the nav_pos_time_status service
	 *
	 * @param      req   Empty
	 * @param      resp  The response (NavPosStatus)
	 *
	 * @return     true if successful, false otherwise
	 */
	bool provideNavPosStatus(num_sdk_msgs::NavPosStatusQuery::Request &req, num_sdk_msgs::NavPosStatusQuery::Response &resp);

	void setGPSFixHandler(const sensor_msgs::NavSatFix::ConstPtr &msg);
	void enableGPSFixOverrideHandler(const std_msgs::Bool::ConstPtr &msg);
	void setGPSFixOverrideHandler(const sensor_msgs::NavSatFix::ConstPtr &msg);

	void enableHeadingOverrideHandler(const std_msgs::Bool::ConstPtr &msg);
	void setHeadingOverrideHandler(const num_sdk_msgs::Heading::ConstPtr &msg);

	void enableAttitudeOverrideHandler(const std_msgs::Bool::ConstPtr &msg);
	void setAttitudeOverrideHandler(const geometry_msgs::QuaternionStamped &msg);

	void setIMUTopic(const std_msgs::String::ConstPtr &msg);
	void setOdomTopic(const std_msgs::String::ConstPtr &msg);

	void setAHRSSourceFrameHandler(const std_msgs::String::ConstPtr &msg);
	void setAHRSOutputFrameHandler(const std_msgs::String::ConstPtr &msg);

	void setAHRSOffsetHandler(const num_sdk_msgs::Offset::ConstPtr &msg);

	void serviceAHRS();

	static bool validateAHRSData(const AHRSDataSet &ahrs_data);

	void saveDataIfNecessary(const AHRSDataSet &ahrs_data);

	void ensureAHRSTypeROS();

	bool transformAHRSData(AHRSDataSet &ahrs_data);

	void setupAHRSOffsetFrame();

	void startNewDataFile();

}; // class NavPosTimeMgr
} // namespace Numurus
#endif // NAV_POS_TIME_MGR
