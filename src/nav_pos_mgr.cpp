#include <cstdlib>
#include <cmath>

#include "nav_pos_mgr.h"
#include "lord_ahrs_driver.h"

#define NODE_NAME	"nav_pos_mgr"

#define MAX_NAV_POS_QUERY_DELAY	5.0 // TODO: Make this a configurable parameter?

#define DEG_TO_RAD(angle_deg) ((angle_deg) * M_PI / 180.0f)

namespace Numurus
{

NavPosMgr::NavPosMgr() :
	ahrs_comm_device{"ahrs_comm_device", "/dev/ttyUSB0", this},
	ahrs_update_rate_hz{"ahrs_update_rate_hz", 10.0f, this},
	ahrs_roll_offset_deg{"ahrs_roll_offset_deg", 0.0f, this},
	ahrs_pitch_offset_deg{"ahrs_pitch_offset_deg", 0.0f, this},
	ahrs_yaw_offset_deg{"ahrs_yaw_offset_deg", 0.0f, this},
	ahrs_ready{false},
	ahrs_rcv_thread{nullptr},
	ahrs_rcv_continue{false},
	ahrs_data_stack_max_size{1},
	update_ahrs_nav_sat_fix{false}
{
	latest_nav_sat_fix.header.seq = 0;
	latest_nav_sat_fix.header.stamp = ros::Time(0.0);
	latest_nav_sat_fix.header.frame_id = "vehicle_frame";
	latest_nav_sat_fix.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
	latest_nav_sat_fix.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
	latest_nav_sat_fix.latitude = 0.0;
	latest_nav_sat_fix.longitude = 0.0;
	latest_nav_sat_fix.altitude = 0.0;
	latest_nav_sat_fix.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
}

NavPosMgr::~NavPosMgr()
{
	if (nullptr != ahrs_rcv_thread)
	{
		delete ahrs_rcv_thread;
	}
}

void NavPosMgr::init()
{
	SDKNode::init(); // call the base class method first

	// Now the config variables are populated
	const LORDAHRSRollPitchYaw rpy_rad(DEG_TO_RAD(ahrs_roll_offset_deg),
																		 DEG_TO_RAD(ahrs_pitch_offset_deg),
																	 	 DEG_TO_RAD(ahrs_yaw_offset_deg));
	const std::time_t now_s = std::time(0);
	std::string ahrs_comm_device_name = ahrs_comm_device;
	ahrs_ready = ahrs.init(ahrs_comm_device_name.c_str(), ahrs_update_rate_hz, now_s,
            						 rpy_rad);
	if (false == ahrs_ready)
	{
		ROS_ERROR("Unable to initialize AHRS device driver");
	}
	else
	{
		// Launch the AHRS receive thread
		ahrs_rcv_continue = true;
		ahrs_data_stack_max_size = static_cast<size_t>(std::ceil(ahrs_update_rate_hz * MAX_NAV_POS_QUERY_DELAY));
		ahrs_rcv_thread = new std::thread(&NavPosMgr::serviceAHRS, this);
	}
}

void NavPosMgr::initServices()
{
	// Call the base method
	SDKNode::initServices();

	// Advertise trigger status query service
	servicers.push_back(n.advertiseService("nav_pos_query", &Numurus::NavPosMgr::provideNavPos, this));
	servicers.push_back(n.advertiseService("nav_pos_status_query", &Numurus::NavPosMgr::provideNavPosStatus, this));
}

void NavPosMgr::initSubscribers()
{
	// Call the base method
	SDKNode::initSubscribers();

	subscribers.push_back(n.subscribe("set_gps_fix", 3, &NavPosMgr::setGPSFixHandler, this));
}

bool NavPosMgr::provideNavPos(num_sdk_msgs::NavPosQuery::Request &req, num_sdk_msgs::NavPosQuery::Response &resp)
{
	const bool latestDataRequested = (0.0 == req.query_time.toSec())? true : false;
	if (false == latestDataRequested)
	{
		// If they aren't requesting the most recent data, ensure the request is within the valid period
		const ros::Time now = ros::Time::now();
		const ros::Duration elapsed = now - req.query_time;
		if (elapsed.toSec() > MAX_NAV_POS_QUERY_DELAY)
		{
			ROS_ERROR("%s: Requested nav/pos time (%.3f) is more than %.3f seconds ago... cannot retrieve data", getUnqualifiedName().c_str(),
					  req.query_time.toSec(), MAX_NAV_POS_QUERY_DELAY);
			return false;
		}
	}

	// Now, gather the ahrs data - under mutex protection since this is multi-threaded
	AHRSDataSet ahrs_data;
	{
		std::lock_guard<std::mutex> lk(ahrs_data_stack_mutex);
		if (ahrs_data_stack.size() > 0)
		{
			// TODO: Respect the requested data timestamp via zero-order hold, linear interpolation, etc.
			// instead of just providing latest data via [0] index.
			ahrs_data = ahrs_data_stack[0];
		}
		else
		{
			return false;
		}
	}

	// Debugging
	//ahrs_data.print();

	// TODO: Simulated and non-simulated control. For now, just use the returned data
	// (will be zero'd if we failed)
	resp.nav_pos.timestamp = ros::Time(ahrs_data.timestamp);

	// Copy in the latest position fix -- under mutex protection since this could be
	// read by the driver thread concurrently.
	{
		std::lock_guard<std::mutex> lk(latest_nav_sat_fix_mutex);
		resp.nav_pos.fix = latest_nav_sat_fix;
	}

	// accel
	resp.nav_pos.accel.linear.x = ahrs_data.accel_x;
	resp.nav_pos.accel.linear.y = ahrs_data.accel_y;
	resp.nav_pos.accel.linear.z = ahrs_data.accel_z;
	resp.nav_pos.accel.angular.x = 0.0;
	resp.nav_pos.accel.angular.y = 0.0;
	resp.nav_pos.accel.angular.z = 0.0;

	// linear velocity
	resp.nav_pos.linear_velocity.x = 0.0;
	resp.nav_pos.linear_velocity.y = 0.0;
	resp.nav_pos.linear_velocity.z = 0.0;

	// angular velocity
	resp.nav_pos.angular_velocity.x = ahrs_data.angular_velocity_x;
	resp.nav_pos.angular_velocity.y = ahrs_data.angular_velocity_y;
	resp.nav_pos.angular_velocity.z = ahrs_data.angular_velocity_z;

	// orientation - must ensure it's normalized. Using clever procedure from here: http://planning.cs.uiuc.edu/node198.html
	resp.nav_pos.orientation.x = ahrs_data.orientation_q1_i;
	resp.nav_pos.orientation.y = ahrs_data.orientation_q2_j;
	resp.nav_pos.orientation.z = ahrs_data.orientation_q3_k;
	resp.nav_pos.orientation.w = ahrs_data.orientation_q0;

	// heading
	resp.nav_pos.heading = ahrs_data.heading;
	resp.nav_pos.heading_true_north = ahrs_data.heading_true_north;

	return true;
}

bool NavPosMgr::provideNavPosStatus(num_sdk_msgs::NavPosStatusQuery::Request&, num_sdk_msgs::NavPosStatusQuery::Response &resp)
{
	{
		std::lock_guard<std::mutex> lk(latest_nav_sat_fix_mutex);
		resp.status.last_nav_sat_fix = latest_nav_sat_fix.header.stamp;
	}
	resp.status.nav_sat_fix_rate = 0.0; // TODO: Keep track of this in a member variable
	resp.status.last_imu = ros::Time(0.0);
	resp.status.imu_rate = 0.0;
	{
		std::lock_guard<std::mutex> lk(ahrs_data_stack_mutex);
		if (ahrs_data_stack.size() > 0)
		{
			resp.status.last_imu = ros::Time(ahrs_data_stack[0].timestamp);
		}
		if (ahrs_data_stack.size() > 1)
		{
			// Just compute the latest two-point rate
			resp.status.imu_rate = 1.0f / (ahrs_data_stack[0].timestamp - ahrs_data_stack[1].timestamp);
		}
	}

	return true;
}

void NavPosMgr::setGPSFixHandler(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
	latest_nav_sat_fix = *msg;

	// Update the AHRS -- this allows it to calculate a declination and provide a true
	// north heading.
	if (sensor_msgs::NavSatStatus::STATUS_NO_FIX != latest_nav_sat_fix.status.status)
	{
		// Must interact with the driver in a threadsafe way -- the service thread
		// handles all calls into driver API, just signal it here
		update_ahrs_nav_sat_fix = true; // atomic variable
	}
}

void NavPosMgr::serviceAHRS()
{
	// Time update rate
	const ros::Duration TIME_UPDATE_INTERVAL(1.0); // 1Hz
	ros::Time last_update_time = ros::Time::now();

	while(true == ahrs_rcv_continue)
	{
		// Query for the latest data
		AHRSDataSet ahrs_data;
		const bool got_data = ahrs.receiveLatestData(ahrs_data); // Blocks most of the time

		if (true == update_ahrs_nav_sat_fix) // atomic variable
		{
			std::lock_guard<std::mutex> lk(latest_nav_sat_fix_mutex);
			LORDAHRSReferencePosition ref_pos;
			ref_pos.latitude_deg = latest_nav_sat_fix.latitude;
			ref_pos.longitude_deg = latest_nav_sat_fix.longitude;
			ref_pos.altitude_m = latest_nav_sat_fix.altitude;
			ahrs.updateReferencePosition(ref_pos);
			update_ahrs_nav_sat_fix = false;
		}

		// Send time updates at intervals -- do this quickly after receiveLatestData()
		// to help ensure an empty rcv buffer... that's easier on everybody
		const ros::Time now = ros::Time::now();
		if (now - last_update_time >= TIME_UPDATE_INTERVAL)
		{
			if (true == ahrs.updateSystemTime(now.sec))
			{
				last_update_time = now;
			}
		}

		// Now, stuff the data on our stack as long as it is sufficiently "valid"
		if ((true == got_data) && (true == validateAHRSData(ahrs_data)))
		{
			std::lock_guard<std::mutex> lk(ahrs_data_stack_mutex);
			// Clear the end of this fixed-size stack if necessary before adding a new
			// element
			while(ahrs_data_stack.size() >= ahrs_data_stack_max_size)
			{
				ahrs_data_stack.pop_back();
			}
			ahrs_data_stack.push_front(ahrs_data);
		}
	}

	ROS_WARN("Terminating AHRS service thread");
}

bool NavPosMgr::validateAHRSData(const AHRSDataSet &ahrs_data)
{
	if ((AHRS_FILTER_STAT_RUN_VAL != ahrs_data.filter_state) ||
			(false == ahrs_data.accel_valid) ||
			(false == ahrs_data.angular_velocity_valid) ||
			(false == ahrs_data.orientation_valid) ||
			(false == ahrs_data.heading_valid))
	{
		return false;
	}
	return true;
}

} // namespace Numurus

int main(int argc, char **argv)
{
	ros::init(argc, argv, NODE_NAME);
	ROS_INFO("Starting the %s node", NODE_NAME);

	Numurus::NavPosMgr nav_pos_mgr;
	nav_pos_mgr.run();

	return 0;
}
