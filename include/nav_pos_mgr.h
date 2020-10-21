#ifndef NAV_POS_MGR_H
#define NAV_POS_MGR_H

#include <thread>
#include <atomic>
#include <mutex>
#include <deque>

#include "sensor_msgs/NavSatFix.h"

#include "sdk_node.h"
//#include "lord_ahrs_driver.h"
#include "ahrs_driver.h"
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
	void initServices() override;
	void initSubscribers() override;

private:
	NodeParam<std::string> ahrs_comm_device;
	NodeParam<float> ahrs_update_rate_hz;
	NodeParam<float> ahrs_roll_offset_deg;
	NodeParam<float> ahrs_pitch_offset_deg;
	NodeParam<float> ahrs_yaw_offset_deg;
	NodeParam<std::string> ahrs_type;
	NodeParam<std::string> imu_topic;
	NodeParam<std::string> odom_topic;
	bool ahrs_ready;
	std::thread *ahrs_rcv_thread;
	std::atomic<bool> ahrs_rcv_continue;
	std::deque<AHRSDataSet> ahrs_data_stack;
	std::mutex ahrs_data_stack_mutex;
	size_t ahrs_data_stack_max_size;

	AHRSDriver *ahrs = nullptr;

	std::atomic<bool> update_ahrs_nav_sat_fix;
	std::mutex latest_nav_sat_fix_mutex;
	sensor_msgs::NavSatFix latest_nav_sat_fix;

	SaveDataInterface *save_data_if = nullptr;

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

	void setIMUTopic(const std_msgs::String::ConstPtr &msg);
	void setOdomTopic(const std_msgs::String::ConstPtr &msg);

	void serviceAHRS();

	static bool validateAHRSData(const AHRSDataSet &ahrs_data);

	void saveDataIfNecessary(const AHRSDataSet &ahrs_data);

	void ensureAHRSTypeROS();

}; // class NavPosTimeMgr
} // namespace Numurus
#endif // NAV_POS_TIME_MGR
