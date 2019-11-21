#ifndef __AHRS_DRIVER_H
#define __AHRS_DRIVER_H

#include <stdint.h>
#include <stdio.h>

namespace Numurus
{

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
  return ((true == input)? "valid" : "invalid");
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

  inline void print()
  {
    printf("*** AHRS Data ***\n");
    printf("\tTimestamp: %f\n", timestamp);
    printf("\tFilter State: 0x%x, Flags: 0x%x\n", filter_state, filter_flags);
    printf("\tAccel: [%f, %f, %f] g -- %s\n", accel_x, accel_y, accel_z, boolToCStringValid(accel_valid));
    printf("\tVelocity: [%f, %f, %f] m/s\n", velocity_x, velocity_y, velocity_z);
    printf("\tAngular Rate: [%f, %f, %f] rad/s -- %s\n", angular_velocity_x, angular_velocity_y, angular_velocity_z,
           boolToCStringValid(angular_velocity_valid));
    printf("\tOrientation: [%f, %fi, %fj, %fk] -- %s\n", orientation_q0, orientation_q1_i, orientation_q2_j,
          orientation_q3_k, boolToCStringValid(orientation_valid));
    printf("\tHeading: %f deg (%s north) -- %s\n", heading,
           (true == heading_true_north)? "true" : "mag.",
           boolToCStringValid(heading_valid));
  }
};

class AHRSDriver
{
public:
  AHRSDriver();
  virtual ~AHRSDriver();

  virtual bool receiveLatestData(AHRSDataSet &data_out) = 0;
}; // class AHRSDriver

} // namespance Numurus

#endif
