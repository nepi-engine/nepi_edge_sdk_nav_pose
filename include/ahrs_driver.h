#ifndef __AHRS_DRIVER_H
#define __AHRS_DRIVER_H

#include <stdint.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/stat.h>

namespace Numurus
{
struct AHRSRollPitchYaw
{
  float roll_rad = 0.0f;
  float pitch_rad = 0.0f;
  float yaw_rad = 0.0f;

  AHRSRollPitchYaw() :
    roll_rad{0.0f},
    pitch_rad{0.0f},
    yaw_rad{0.0f}
  {}

  AHRSRollPitchYaw(float r, float p, float y) :
    roll_rad{r},
    pitch_rad{r},
    yaw_rad{r}
  {}
};

struct AHRSReferencePosition
{
  double latitude_deg = 0.0;
  double longitude_deg = 0.0;
  double altitude_m = 0.0;
};

enum AHRSFilterStatus
{
  AHRS_FILTER_STAT_STARTUP    = 0x00,
  AHRS_FILTER_STAT_INIT       = 0x01,
  AHRS_FILTER_STAT_RUN_VAL    = 0x02,
  AHRS_FILTER_STAT_RUN_ERR    = 0x03
};

typedef uint16_t AHRSFilterStatusFlags_t;

inline static const char* boolToCStringValid(bool input)
{
  return ((true == input)? "true" : "false");
}

inline static const char* filterStatusToCString(AHRSFilterStatus status)
{
  if (status == AHRS_FILTER_STAT_STARTUP) return "STARTUP";
  if (status == AHRS_FILTER_STAT_INIT   ) return "INIT";
  if (status == AHRS_FILTER_STAT_RUN_VAL) return "RUNNING";
  if (status == AHRS_FILTER_STAT_RUN_ERR) return "ERROR";
  return "UNKNOWN";
}

struct AHRSDataSet
{
  double timestamp = 0.0f;

  AHRSFilterStatus filter_state = AHRS_FILTER_STAT_STARTUP;
  AHRSFilterStatusFlags_t filter_flags = 0;

  // Linear Accelerations (m/s^2), grav vector removed, frame transformation applied
  float accel_x = 0.0f;
  float accel_y = 0.0f;
  float accel_z = 0.0f;
  bool accel_valid = false;

  // Linear Velocity (m/s), frame transformation applied
  float velocity_x = 0.0f;
  float velocity_y = 0.0f;
  float velocity_z = 0.0f;
  bool velocity_valid = false;

  // Angular Velocity (rad/s), frame transformation applied
  float angular_velocity_x = 0.0f;
  float angular_velocity_y = 0.0f;
  float angular_velocity_z = 0.0f;
  bool angular_velocity_valid = false;

  // Orientation (quaterion) w.r.t. fixed-earth coordinate frame
  float orientation_q0 = 1.0f;
  float orientation_q1_i = 0.0f;
  float orientation_q2_j = 0.0f;
  float orientation_q3_k = 0.0f;
  bool orientation_valid = false;

  // Heading (deg)
  float heading = 0.0f;
  bool heading_true_north = false;
  bool heading_valid = false;

  inline void printYAML(FILE *stream=stdout, const char* prefix="") const
  {
    // YAML
    fprintf(stream, "%sahrs:\n", prefix);
    fprintf(stream, "%s  timestamp: %f\n", prefix, timestamp);
    fprintf(stream, "%s  filter_state: %s\n", prefix, filterStatusToCString(filter_state));
    fprintf(stream, "%s  filter_flags: 0x%x\n", prefix, filter_flags);

    fprintf(stream, "%s  accel:\n", prefix);
    fprintf(stream, "%s    x: %f\n", prefix, accel_x);
    fprintf(stream, "%s    y: %f\n", prefix, accel_y);
    fprintf(stream, "%s    z: %f\n", prefix, accel_z);
    fprintf(stream, "%s    valid: %s\n", prefix, boolToCStringValid(accel_valid));

    fprintf(stream, "%s  velocity:\n", prefix);
    fprintf(stream, "%s    x: %f\n", prefix, velocity_x);
    fprintf(stream, "%s    y: %f\n", prefix, velocity_y);
    fprintf(stream, "%s    z: %f\n", prefix, velocity_z);
    fprintf(stream, "%s    valid: %s\n", prefix, boolToCStringValid(velocity_valid));

    fprintf(stream, "%s  angular_rate:\n", prefix);
    fprintf(stream, "%s    x: %f\n", prefix, angular_velocity_x);
    fprintf(stream, "%s    y: %f\n", prefix, angular_velocity_y);
    fprintf(stream, "%s    z: %f\n", prefix, angular_velocity_z);
    fprintf(stream, "%s    valid: %s\n", prefix, boolToCStringValid(angular_velocity_valid));

    fprintf(stream, "%s  pose_quaternion:\n", prefix);
    fprintf(stream, "%s    x: %f\n", prefix, orientation_q1_i);
    fprintf(stream, "%s    y: %f\n", prefix, orientation_q2_j);
    fprintf(stream, "%s    z: %f\n", prefix, orientation_q3_k);
    fprintf(stream, "%s    w: %f\n", prefix, orientation_q0);
    fprintf(stream, "%s    valid: %s\n", prefix, boolToCStringValid(orientation_valid));

    fprintf(stream, "%s  heading:\n", prefix);
    fprintf(stream, "%s    deg: %f\n", prefix, heading);
    fprintf(stream, "%s    ref: %s\n", prefix, (true == heading_true_north)? "true" : "magnetic");
    fprintf(stream, "%s    valid: %s\n", prefix, boolToCStringValid(heading_valid));
  }
};

class AHRSDriver
{
public:
  AHRSDriver();
  virtual ~AHRSDriver();

  // Subclasses may implement
  virtual inline bool init(const char* hardware_id, float data_rate_hz, uint32_t current_time_posix,
                           const AHRSRollPitchYaw &sensor_to_vehicle) {return true;}
  virtual inline bool updateSystemTime(uint32_t posix_time){return true;}
  virtual inline bool updateReferencePosition(const AHRSReferencePosition &pos){return true;}
  virtual void overrideHeadingData(float heading_deg, bool heading_true_north) {
    heading_override = true;
    heading_override_deg = heading_deg;
    heading_override_true_north = heading_true_north;
  }
  virtual inline void clearHeadingOverride(){heading_override = false;}

   // Subclasses MUST implement
  virtual bool receiveLatestData(AHRSDataSet &data_out) = 0;

protected:
  bool heading_override = false;
  float heading_override_deg = 0.0f;
  bool heading_override_true_north = true;
}; // class AHRSDriver

} // namespance Numurus

#endif
