#ifndef NAV_POS_MGR_H
#define NAV_POS_MGR_H

#include <thread>
#include <atomic>
#include <mutex>
#include <deque>

#include "sensor_msgs/NavSatFix.h"

#include "sdk_node.h"
#include "lord_ahrs_driver.h"
#include "num_sdk_msgs/NavPosQuery.h"
#include "num_sdk_msgs/NavPosStatusQuery.h"

namespace Numurus
{
class LORDAHRSDriver; // forward declaration;

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
	bool ahrs_ready;
	std::thread *ahrs_rcv_thread;
	std::atomic<bool> ahrs_rcv_continue;
	std::deque<AHRSDataSet> ahrs_data_stack;
	std::mutex ahrs_data_stack_mutex;
	size_t ahrs_data_stack_max_size;
	// TODO: How should we specialize this class for other AHRS drivers
	// (e.g., raw IMU driver)? Subclass it? Conditional compilation? Config variable
	// (pushing the instantiation out to the init method after config variables are
	// loaded)?
	LORDAHRSDriver ahrs;

	std::atomic<bool> update_ahrs_nav_sat_fix;
	std::mutex latest_nav_sat_fix_mutex;
	sensor_msgs::NavSatFix latest_nav_sat_fix;

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

	void serviceAHRS();

	static bool validateAHRSData(const AHRSDataSet &ahrs_data);

}; // class NavPosTimeMgr
} // namespace Numurus
#endif // NAV_POS_TIME_MGR
