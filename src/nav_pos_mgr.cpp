#include <cstdlib>
#include <cmath>

#include <sys/stat.h>

#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"

#include "nav_pos_mgr.h"
#include "drivers/generic_ahrs/lord_ahrs_driver.h"
#include "drivers/generic_ahrs/ros_ahrs_driver.h"
#include "save_data_interface.h"

#define NODE_NAME	"nav_pos_mgr"

#define MAX_NAV_POS_QUERY_DELAY	5.0 // TODO: Make this a configurable parameter?

#define AHRS_TYPE_LORD "lord"
#define AHRS_TYPE_ROS "ros"

#define DEG_TO_RAD(angle_deg) ((angle_deg) * M_PI / 180.0f)
#define RAD_TO_DEG(angle_rad) ((angle_rad) * 180.0f / M_PI)

#define OFFSETS_SET_FRAME_ID		"manual_offset_frame"

namespace Numurus
{

static inline const char* navSatStatusToString(int8_t status)
{
	if (status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) return "NO FIX";
	if (status == sensor_msgs::NavSatStatus::STATUS_FIX) return "FIX";
	if (status == sensor_msgs::NavSatStatus::STATUS_SBAS_FIX) return "SBAS FIX";
	if (status == sensor_msgs::NavSatStatus::STATUS_GBAS_FIX) return "GBAS FIX";
	return "UNKNOWN";
}

static inline const char* navSatServiceToString(int8_t service)
{
	if (service == sensor_msgs::NavSatStatus::SERVICE_GPS) return "GPS";
	if (service == sensor_msgs::NavSatStatus::SERVICE_GLONASS) return "GLONASS";
	if (service == sensor_msgs::NavSatStatus::SERVICE_COMPASS) return "COMPASS";
	if (service == sensor_msgs::NavSatStatus::SERVICE_GALILEO) return "GALILEO";
	return "UNKNOWN";
}

NavPosMgr::NavPosMgr() :
	ahrs_comm_device{"ahrs_comm_device", "", this}, // e.g., /dev/ttyUSB0 for LORD via USB
	ahrs_update_rate_hz{"ahrs_update_rate_hz", 10.0f, this},
	ahrs_offsets_set{"ahrs_offset/offsets_set", false, this},
	ahrs_x_offset_m{"ahrs_offset/x_m", 0.0f, this},
	ahrs_y_offset_m{"ahrs_offset/y_m", 0.0f, this},
	ahrs_z_offset_m{"ahrs_offset/z_m", 0.0f, this},
	ahrs_x_rot_offset_deg{"ahrs_offset/x_rot_deg", 0.0f, this},
	ahrs_y_rot_offset_deg{"ahrs_offset/y_rot_deg", 0.0f, this},
	ahrs_z_rot_offset_deg{"ahrs_offset/z_rot_deg", 0.0f, this},
	ahrs_type{"ahrs_type", AHRS_TYPE_ROS, this}, // Better default?
	imu_topic{"imu_topic", "sensor_3dx/stereo_cam_driver/imu/data", this}, // Better default?
	odom_topic{"odom_topic", "sensor_3dx/stereo_cam_driver/odom", this}, // Better default?
	ahrs_src_frame_id{"ahrs_src_frame_id", "stereo_cam_imu_link", this},
	ahrs_out_frame_id{"ahrs_out_frame_id", "3dx_center_frame", this},
	ahrs_ready{false},
	ahrs_rcv_thread{nullptr},
	ahrs_rcv_continue{false},
	ahrs_data_stack_max_size{1},
	update_ahrs_nav_sat_fix{false}
{
	latest_nav_sat_fix.header.seq = 0;
	latest_nav_sat_fix.header.stamp = ros::Time(0.0);
	latest_nav_sat_fix.header.frame_id = ahrs_out_frame_id;
	latest_nav_sat_fix.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
	latest_nav_sat_fix.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
	latest_nav_sat_fix.latitude = 0.0;
	latest_nav_sat_fix.longitude = 0.0;
	latest_nav_sat_fix.altitude = 0.0;
	latest_nav_sat_fix.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

	save_data_if = new SaveDataInterface(this, &n, &n_priv);
	save_data_if->registerDataProduct("nav_pos");
}

NavPosMgr::~NavPosMgr()
{
	if (nullptr != ahrs_rcv_thread)
	{
		delete ahrs_rcv_thread;
	}
	if (nullptr != ahrs)
	{
		delete ahrs;
	}
	if (nullptr != save_data_if)
	{
		delete save_data_if;
	}
}

void NavPosMgr::init()
{
	SDKNode::init(); // call the base class method first

	// Now the config variables are populated
	std::string type = ahrs_type; // Force a cast for SDKNodeParam to string
	if (type == AHRS_TYPE_LORD)
	{
		ahrs = new LORDAHRSDriver();
	}
	else if (type == AHRS_TYPE_ROS)
	{
		ahrs = new ROSAHRSDriver(n, imu_topic, odom_topic);
	}
	else
	{
		ROS_FATAL("%s: Critical failure: Invalid AHRS type %s\n", getUnqualifiedName().c_str(), type.c_str());
		return;
	}

	// Initialize the AHRS
	/*
	const AHRSRollPitchYaw rpy_rad(DEG_TO_RAD(ahrs_roll_offset_deg),
																 DEG_TO_RAD(ahrs_pitch_offset_deg),
															 	 DEG_TO_RAD(ahrs_yaw_offset_deg));
	*/
	const AHRSRollPitchYaw rpy_rad(0.0, 0.0, 0.0); // Now handling this via ROS TF, so don't apply at transform at the sensor
	const std::time_t now_s = std::time(0);
	std::string ahrs_comm_device_name = ahrs_comm_device; // Must assign to get the cast overload on SDKNodeParam to kick in.
	ahrs_ready = ahrs->init(ahrs_comm_device_name.c_str(), ahrs_update_rate_hz, now_s,
	            						 rpy_rad);
	if (false == ahrs_ready)
	{
		ROS_ERROR("Unable to initialize AHRS (%s:%s) device driver\n", type.c_str(), ahrs_comm_device_name.c_str());
		return;
	}

	// Launch the AHRS receive thread
	ahrs_rcv_continue = true;
	ahrs_data_stack_max_size = static_cast<size_t>(std::ceil(ahrs_update_rate_hz * MAX_NAV_POS_QUERY_DELAY));
	ahrs_rcv_thread = new std::thread(&NavPosMgr::serviceAHRS, this);
}

void NavPosMgr::retrieveParams()
{
	SDKNode::retrieveParams();
	ahrs_comm_device.retrieve();
	ahrs_update_rate_hz.retrieve();
	ahrs_offsets_set.retrieve();
	ahrs_x_offset_m.retrieve();
	ahrs_y_offset_m.retrieve();
	ahrs_z_offset_m.retrieve();
	ahrs_x_rot_offset_deg.retrieve();
	ahrs_y_rot_offset_deg.retrieve();
	ahrs_z_rot_offset_deg.retrieve();
	ahrs_type.retrieve();
	imu_topic.retrieve();
	odom_topic.retrieve();
	ahrs_src_frame_id.retrieve();
	ahrs_out_frame_id.retrieve();

	setupAHRSOffsetFrame();
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
	subscribers.push_back(n_priv.subscribe("set_heading_override", 3, &NavPosMgr::setHeadingOverrideHandler, this));
	subscribers.push_back(n_priv.subscribe("clear_heading_override", 3, &NavPosMgr::clearHeadingOverrideHandler, this));
	subscribers.push_back(n_priv.subscribe("set_attitude_override", 3, &NavPosMgr::setAttitudeOverrideHandler, this));
	subscribers.push_back(n_priv.subscribe("clear_attitude_override", 3, &NavPosMgr::clearAttitudeOverrideHandler, this));

	subscribers.push_back(n_priv.subscribe("set_imu_topic", 3, &NavPosMgr::setIMUTopic, this));
	subscribers.push_back(n_priv.subscribe("set_odom_topic", 3, &NavPosMgr::setOdomTopic, this));

	subscribers.push_back(n_priv.subscribe("set_ahrs_src_frame", 3, &NavPosMgr::setAHRSSourceFrameHandler, this));
	subscribers.push_back(n_priv.subscribe("set_ahrs_out_frame", 3, &NavPosMgr::setAHRSOutputFrameHandler, this));

	subscribers.push_back(n_priv.subscribe("set_ahrs_offset", 3, &NavPosMgr::setAHRSOffsetHandler, this));
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
		//ROS_INFO("Debug: Servicing nav_pos request from %lu in the stack\n", ahrs_data_stack.size());
		if (ahrs_data_stack.size() > 0)
		{
			// TODO: Respect the requested data timestamp via zero-order hold, linear interpolation, etc.
			// instead of just providing latest data via [0] index.
			ahrs_data = ahrs_data_stack[0];
		}
		else
		{
			// Just silently fail -- this would be the norm if there was no AHRS attached.
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
	resp.nav_pos.linear_velocity.x = ahrs_data.velocity_x;
	resp.nav_pos.linear_velocity.y = ahrs_data.velocity_y;
	resp.nav_pos.linear_velocity.z = ahrs_data.velocity_z;

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

	// Finally, the current AHRS transform via transform_listener
	ros::Time latest(0.0);
	const std::string src_frame = ahrs_src_frame_id;
	const std::string target_frame = ahrs_out_frame_id;
	tf::StampedTransform transform;
	transform_listener.lookupTransform(src_frame, target_frame, latest, transform); // API description is backwards
	tf::transformStampedTFToMsg(transform, resp.status.transform);

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

void NavPosMgr::setHeadingOverrideHandler(const num_sdk_msgs::Heading::ConstPtr &msg)
{
	if (msg->heading <= -360.0 || msg->heading >= 360.0)
	{
		ROS_ERROR("Invalid heading override: %f deg... ignoring", msg->heading);
	}
	else
	{
		ahrs->overrideHeadingData(msg->heading, msg->true_north);
	}
}

void NavPosMgr::clearHeadingOverrideHandler(const std_msgs::Empty::ConstPtr &msg)
{
	ahrs->clearHeadingOverride();
}

void NavPosMgr::setAttitudeOverrideHandler(const geometry_msgs::QuaternionStamped &msg)
{
	// Need to map into configured AHRS frame, so it looks like it came from standard AHRS
	const std::string out_frame = ahrs_out_frame_id;
	const std::string src_frame = msg.header.frame_id;
	if (false == transform_listener.frameExists(src_frame))
	{
		ROS_ERROR("Attitude override has invalid frame id: %s... override not applied", src_frame.c_str());
		return;
	}

	geometry_msgs::QuaternionStamped quat_out;
	try
	{
		transform_listener.transformQuaternion(ahrs_out_frame_id, msg, quat_out);
	}
	catch (tf2::TransformException &ex)
	{
		ROS_WARN_THROTTLE(1.0, "%s", ex.what());
	}

	ahrs->overrideOrientationData(quat_out.quaternion.w, quat_out.quaternion.x, quat_out.quaternion.y, quat_out.quaternion.z);
}

void NavPosMgr::clearAttitudeOverrideHandler(const std_msgs::Empty::ConstPtr &msg)
{
	ahrs->clearOrientationOverride();
}

void NavPosMgr::ensureAHRSTypeROS()
{
	std::string type = ahrs_type; // Force a cast for SDKNodeParam to string
	if (type != AHRS_TYPE_ROS)
	{
		ROS_WARN("Changing AHRS type to ROS... No nav/pos data until both IMU and Odom topics are set");
		if (nullptr != ahrs)
		{
			delete ahrs;
		}
		ahrs = new ROSAHRSDriver(n);
		ahrs_type = AHRS_TYPE_ROS;
	}
	else if (nullptr == ahrs)
	{
		ahrs = new ROSAHRSDriver(n);
	}
}

void NavPosMgr::setIMUTopic(const std_msgs::String::ConstPtr &msg)
{
	{
		std::lock_guard<std::mutex> lk(ahrs_data_stack_mutex);
		ensureAHRSTypeROS();
		dynamic_cast<ROSAHRSDriver*>(ahrs)->setIMUSubscription(n, msg->data);
	}
	imu_topic = msg->data;
}

void NavPosMgr::setOdomTopic(const std_msgs::String::ConstPtr &msg)
{
	{
		std::lock_guard<std::mutex> lk(ahrs_data_stack_mutex);
		ensureAHRSTypeROS();
		dynamic_cast<ROSAHRSDriver*>(ahrs)->setOdomSubscription(n, msg->data);
	}
	odom_topic = msg->data;
}

void NavPosMgr::setAHRSSourceFrameHandler(const std_msgs::String::ConstPtr &msg)
{
	const std::string src_frame = msg->data;
	if (false == transform_listener.frameExists(src_frame))
	{
		ROS_ERROR("Cannot set source frame to %s -- frame does not exist", src_frame.c_str());
		return;
	}
	// Clear the offsets_set flag and apply -- mutual exclusion here
	ahrs_offsets_set = false;
	setupAHRSOffsetFrame();

	ahrs_src_frame_id = src_frame; // TODO: Thread safety?

}

void NavPosMgr::setAHRSOutputFrameHandler(const std_msgs::String::ConstPtr &msg)
{
	const std::string output_frame = msg->data;
	if (false == transform_listener.frameExists(output_frame))
	{
		ROS_ERROR("Cannot set output frame to %s -- frame does not exist", output_frame.c_str());
		return;
	}
	ahrs_out_frame_id = output_frame; // TODO: Thread safety?
}

void NavPosMgr::setAHRSOffsetHandler(const num_sdk_msgs::Offset::ConstPtr &msg)
{
	ahrs_offsets_set = true;
	ahrs_x_offset_m = msg->translation.x;
	ahrs_y_offset_m = msg->translation.y;
	ahrs_z_offset_m = msg->translation.z;

	ahrs_x_rot_offset_deg = msg->rotation.x;
	ahrs_y_rot_offset_deg = msg->rotation.y;
	ahrs_z_rot_offset_deg = msg->rotation.z;

	setupAHRSOffsetFrame();
}


void NavPosMgr::serviceAHRS()
{
	// Time update rate
	const ros::Duration TIME_UPDATE_INTERVAL(1.0); // 1Hz
	ros::Time last_update_time = ros::Time::now();

	// No sense running this thread much, much faster
	// than the rest of the system can keep up. 100Hz is
	// probably fine.
	ros::Rate service_rate(AHRS_DRIVER_SERVICE_RATE);
	while((true == ahrs_rcv_continue && true == ros::ok()))
	{
		// Query for the latest data
		AHRSDataSet ahrs_data;
		const bool got_data = ahrs->receiveLatestData(ahrs_data); // Blocks most of the time

		if (true == update_ahrs_nav_sat_fix) // atomic variable
		{
			std::lock_guard<std::mutex> lk(latest_nav_sat_fix_mutex);
			AHRSReferencePosition ref_pos;
			ref_pos.latitude_deg = latest_nav_sat_fix.latitude;
			ref_pos.longitude_deg = latest_nav_sat_fix.longitude;
			ref_pos.altitude_m = latest_nav_sat_fix.altitude;
			ahrs->updateReferencePosition(ref_pos);

			update_ahrs_nav_sat_fix = false;
		}

		// Send time updates at intervals -- do this quickly after receiveLatestData()
		// to help ensure an empty rcv buffer... that's easier on everybody
		const ros::Time now = ros::Time::now();
		if (now - last_update_time >= TIME_UPDATE_INTERVAL)
		{
			if (true == ahrs->updateSystemTime(now.sec))
			{
				last_update_time = now;
			}
		}

		// Now, stuff the data on our stack as long as it is sufficiently "valid"
		if ((true == got_data) && (true == validateAHRSData(ahrs_data)))
		{
			// For now, we transform every ahrs_data that goes on the stack, but
			// we could optimize by transforming only those that are getting saved to file
			// or transmitted as part of a request -- however, that requires that the
			// data timestamp is within the transform_listener buffer window (10 secs by default),
			// so would need some safeguards.
			if (false == transformAHRSData(ahrs_data)) // Transforms in place
			{
				// Just return -- don't stick it on the stack and don't save it
				return; // error logged upstream
			}

			std::lock_guard<std::mutex> lk(ahrs_data_stack_mutex);
			// Clear the end of this fixed-size stack if necessary before adding a new
			// element
			while(ahrs_data_stack.size() >= ahrs_data_stack_max_size)
			{
				ahrs_data_stack.pop_back();
			}
			ahrs_data_stack.push_front(ahrs_data);

			// And save the data to disk if necessary
			saveDataIfNecessary(ahrs_data);
		}

		service_rate.sleep();
	}

	ROS_WARN("Terminating AHRS service thread");
}

bool NavPosMgr::validateAHRSData(const AHRSDataSet &ahrs_data)
{
	if ((AHRS_FILTER_STAT_RUN_VAL != ahrs_data.filter_state) ||
			(false == ahrs_data.accel_valid) ||
			(false == ahrs_data.angular_velocity_valid) ||
			(false == ahrs_data.orientation_valid))
			//|| (false == ahrs_data.heading_valid))
	{
		return false;
	}
	return true;
}

void NavPosMgr::saveDataIfNecessary(const AHRSDataSet &ahrs_data)
{
	// Good place to check if we need to close a previously open file due to
	// data saving being disabled
	const bool save_enabled = save_data_if->saveContinuousEnabled();
	if ((false == save_enabled) && (nullptr != data_fd))
	{
		fclose(data_fd);
		data_fd = nullptr;
		return;
	}

	const bool needs_save = save_enabled && save_data_if->dataProductShouldSave("nav_pos");
	if (false == needs_save)
	{
		return;
	}

	// Do the calibration if necessary
	if (true == save_data_if->calibrationShouldSave())
	{
		// TODO: Any calibration. i.e., complete list of transfer frames?

		// Also use this to indicate that we need a new file started -- the logic is the same as
		// for when we need a new calibration file saved, so this is just a bit of a hack
		need_new_data_file = true;
	}

	if ((true == need_new_data_file) || (nullptr == data_fd))
	{
		startNewDataFile();
	}

	if (nullptr == data_fd) // Failed to open a new file
	{
		// Error logged upstream, just return
		return;
	}

	// Print the GPS fix here directly
	sensor_msgs::NavSatFix nav_sat_fix;
	{
		std::lock_guard<std::mutex> lk(latest_nav_sat_fix_mutex);
		nav_sat_fix = latest_nav_sat_fix;
	}
	fprintf(data_fd, "  -  gps:\n");
	fprintf(data_fd, "       timestamp: %f\n", nav_sat_fix.header.stamp.toSec());
	fprintf(data_fd, "       status: %s\n", navSatStatusToString(nav_sat_fix.status.status));
	fprintf(data_fd, "       service: %s\n", navSatServiceToString(nav_sat_fix.status.service));
	fprintf(data_fd, "       latitude: %f\n", nav_sat_fix.latitude);
	fprintf(data_fd, "       longitude: %f\n", nav_sat_fix.longitude);
	fprintf(data_fd, "       altitude: %f\n", nav_sat_fix.altitude);
	// Let the AHRS data print itself

	ahrs_data.printYAML(data_fd, "     ");
}

bool NavPosMgr::transformAHRSData(AHRSDataSet &ahrs_data)
{
	geometry_msgs::Vector3Stamped vect3_in;
	vect3_in.header.seq = 0;
	vect3_in.header.stamp = ros::Time(ahrs_data.timestamp);
	vect3_in.header.frame_id = ahrs_src_frame_id;

	bool transformed = false;
	bool timed_out = false;
	ros::Time start_time = ros::Time::now();
	ros::Duration timeout_period(0.5); // TODO: Configurable?
	while ((transformed == false) && (timed_out == false))
	{
		try
		{
			geometry_msgs::Vector3Stamped vect3_out;

			// Linear accelerations
			if (ahrs_data.accel_valid)
			{
				vect3_in.vector.x = ahrs_data.accel_x;
				vect3_in.vector.y = ahrs_data.accel_y;
				vect3_in.vector.z = ahrs_data.accel_z;

				transform_listener.transformVector(ahrs_out_frame_id, vect3_in, vect3_out);

				ahrs_data.accel_x = vect3_out.vector.x;
				ahrs_data.accel_y = vect3_out.vector.y;
				ahrs_data.accel_z = vect3_out.vector.z;
			}

			// Linear Velocity
			vect3_in.vector.x = ahrs_data.velocity_x;
			vect3_in.vector.y = ahrs_data.velocity_y;
			vect3_in.vector.z = ahrs_data.velocity_z;

			transform_listener.transformVector(ahrs_out_frame_id, vect3_in, vect3_out);

			ahrs_data.velocity_x = vect3_out.vector.x;
			ahrs_data.velocity_y = vect3_out.vector.y;
			ahrs_data.velocity_z = vect3_out.vector.z;

			// Angular Velocity
			if (ahrs_data.angular_velocity_valid)
			{
				vect3_in.vector.x = ahrs_data.angular_velocity_x;
				vect3_in.vector.y = ahrs_data.angular_velocity_y;
				vect3_in.vector.z = ahrs_data.angular_velocity_z;

				transform_listener.transformVector(ahrs_out_frame_id, vect3_in, vect3_out);

				ahrs_data.angular_velocity_x = vect3_out.vector.x;
				ahrs_data.angular_velocity_y = vect3_out.vector.y;
				ahrs_data.angular_velocity_z = vect3_out.vector.z;
			}

			// Orientation
			if (ahrs_data.angular_velocity_valid)
			{
				geometry_msgs::QuaternionStamped quat_in;
				quat_in.header.seq = 0;
				quat_in.header.stamp = ros::Time(ahrs_data.timestamp);
				quat_in.header.frame_id = ahrs_src_frame_id;

				geometry_msgs::QuaternionStamped quat_out;

				quat_in.quaternion.x = ahrs_data.orientation_q1_i;
				quat_in.quaternion.y = ahrs_data.orientation_q2_j;
				quat_in.quaternion.z = ahrs_data.orientation_q3_k;
				quat_in.quaternion.w = ahrs_data.orientation_q0;

				transform_listener.transformQuaternion(ahrs_out_frame_id, quat_in, quat_out);

				ahrs_data.orientation_q1_i = quat_out.quaternion.x;
				ahrs_data.orientation_q2_j = quat_out.quaternion.y;
				ahrs_data.orientation_q3_k = quat_out.quaternion.z;
				ahrs_data.orientation_q0 = quat_out.quaternion.w;
			}

			// Heading
			// TODO: Maybe transformation of heading should be configurable?
			/*
			if (ahrs_data.heading_valid)
			{
				// Make a vector out of the heading and transform it
				vect3_in.vector.x = cos(DEG_TO_RAD(ahrs_data.heading));
				vect3_in.vector.y = -sin(DEG_TO_RAD(ahrs_data.heading));
				vect3_in.vector.z = 0.0;

				transform_listener.transformVector(ahrs_out_frame_id, vect3_in, vect3_out);

				// Now make a heading from the output vector
				// TODO: How do we know which dimension of the new frame is 'forward'???
				ahrs_data.heading = RAD_TO_DEG(atan2(vect3_out.vector.x, vect3_out.vector.y));
			}
			*/
			// If we get this far, set the exit condition
			transformed = true;
		}
		catch (tf2::TransformException &ex)
		{
			if ((ros::Time::now() - start_time) > timeout_period)
			{
				ROS_WARN_THROTTLE(1.0, "%s", ex.what());
				ROS_WARN_THROTTLE(1.0, "Frame listener timeout while trying to transform AHRS data");
				timed_out = true;
			}
		}
	}
	return (transformed && !timed_out);
}

void NavPosMgr::setupAHRSOffsetFrame()
{
	if (true == ahrs_offsets_set)
	{
		ROS_INFO("Setting up a new AHRS Offset Frame:\n\tTranslation = [%f,%f,%f]\n\tRotation = [%f,%f,%f]",
						 (float)ahrs_x_offset_m, (float)ahrs_y_offset_m, (float)ahrs_z_offset_m,
						 (float)ahrs_x_rot_offset_deg, (float)ahrs_y_offset_m, (float)ahrs_z_offset_m);
		geometry_msgs::TransformStamped static_transform;
		static_transform.header.stamp = ros::Time::now();
		static_transform.header.frame_id = ahrs_out_frame_id;
		static_transform.child_frame_id = OFFSETS_SET_FRAME_ID;
		static_transform.transform.translation.x = ahrs_x_offset_m;
		static_transform.transform.translation.y = ahrs_y_offset_m;
		static_transform.transform.translation.z = ahrs_z_offset_m;
		tf2::Quaternion quat;
		quat.setRPY(DEG_TO_RAD(ahrs_x_rot_offset_deg), DEG_TO_RAD(ahrs_y_rot_offset_deg), DEG_TO_RAD(ahrs_z_rot_offset_deg));
		static_transform.transform.rotation.x = quat.x();
		static_transform.transform.rotation.y = quat.y();
		static_transform.transform.rotation.z = quat.z();
		static_transform.transform.rotation.w = quat.w();
		static_transform_broadcaster.sendTransform(static_transform);

		// Update the output frame
		ahrs_src_frame_id = OFFSETS_SET_FRAME_ID;
	}
	else // Clearing the offsets
	{
		// If we are clearing from previously set, set the source to the output as the fallback
		if (ahrs_src_frame_id == OFFSETS_SET_FRAME_ID)
		{
			ahrs_src_frame_id = ahrs_out_frame_id;
		}
	}
}

void NavPosMgr::startNewDataFile()
{
	if (nullptr != data_fd)
	{
		fclose(data_fd);
		data_fd = nullptr;
	}
	// Build the filename
	const std::string display_name = _display_name;
	const std::string tstamp_str = save_data_if->getTimestampString();
	const std::string qualified_filename = save_data_if->getFullPathFilename(tstamp_str, display_name + "_nav", "yaml");
	// Now create the file -- need to do this as a low-level open() call to set the permissions
	int fh = open(qualified_filename.c_str(), O_WRONLY | O_CREAT, 0664);
	if (fh < 0)
	{
		ROS_ERROR("Unable to create file %s for saving nav/pos data", qualified_filename.c_str());
		return;
	}

	// Now open it as a stream
	data_fd = fdopen(fh, "w");
	if (data_fd == nullptr)
	{
		ROS_ERROR("Unable to open file stream for saving nav/pos data");
		return;
	}

	// Now print the header info
	// First, the current AHRS transform
	ros::Time latest(0.0);
	const std::string src_frame = ahrs_src_frame_id;
	const std::string target_frame = ahrs_out_frame_id;
	tf::StampedTransform transform;
	transform_listener.lookupTransform(src_frame, target_frame, latest, transform); // API description is backwards
	geometry_msgs::TransformStamped transform_msg;
	tf::transformStampedTFToMsg(transform, transform_msg);

	fprintf(data_fd, "# Nav/Pose Data File\n");
	fprintf(data_fd, "start_time: %s\n", tstamp_str.c_str());
	fprintf(data_fd, "transform:\n");
	fprintf(data_fd, "  source_frame_id: %s\n", transform_msg.header.frame_id.c_str());
	fprintf(data_fd, "  target_frame_id: %s\n", transform_msg.child_frame_id.c_str());
	fprintf(data_fd, "  # Translation in meters\n");
	fprintf(data_fd, "  translation:\n");
	fprintf(data_fd, "    x: %f\n", transform_msg.transform.translation.x);
	fprintf(data_fd, "    y: %f\n", transform_msg.transform.translation.y);
	fprintf(data_fd, "    z: %f\n", transform_msg.transform.translation.z);
	fprintf(data_fd, "  #Rotation Quaternion\n");
	fprintf(data_fd, "  rotation:\n");
	fprintf(data_fd, "    x: %f\n", transform_msg.transform.rotation.x);
	fprintf(data_fd, "    y: %f\n", transform_msg.transform.rotation.y);
	fprintf(data_fd, "    z: %f\n", transform_msg.transform.rotation.z);
	fprintf(data_fd, "    w: %f\n", transform_msg.transform.rotation.w);
	fprintf(data_fd, "entries:\n");
	fflush(data_fd);

	need_new_data_file = false;
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
