#ifndef __LORD_AHRS_DRIVER_H
#define __LORD_AHRS_DRIVER_H

#include <vector>
#include <string>
#include <cmath>
#include <chrono>
//#include <queue>

#include <ahrs_driver.h>

namespace Numurus
{

inline static const char* boolToCString(bool input)
{
  return ((true == input)? "true" : "false");
}

static constexpr AHRSFilterStatusFlags_t AHRS_FILTER_STAT_FLAGS_ATT_NOT_INIT   = 0x1000;
static constexpr AHRSFilterStatusFlags_t AHRS_FILTER_STAT_FLAGS_POS_VEL_NOT_INIT = 0x2000;
static constexpr AHRSFilterStatusFlags_t AHRS_FILTER_STAT_FLAGS_IMU_NOT_AVAIL  = 0x0001;
static constexpr AHRSFilterStatusFlags_t AHRS_FILTER_STAT_FLAGS_MATRIX_SING = 0x0008;
static constexpr AHRSFilterStatusFlags_t AHRS_FILTER_STAT_FLAGS_POS_COV_HIGH = 0x0010;
static constexpr AHRSFilterStatusFlags_t AHRS_FILTER_STAT_FLAGS_VEL_COV_HIGH = 0x0020;
static constexpr AHRSFilterStatusFlags_t AHRS_FILTER_STAT_FLAGS_ATT_COV_HIGH = 0x0040;
static constexpr AHRSFilterStatusFlags_t AHRS_FILTER_STAT_FLAGS_NAN_DETECTED = 0x0080;
static constexpr AHRSFilterStatusFlags_t AHRS_FILTER_STAT_FLAGS_GYRO_BIAS_HIGH = 0x0100;
static constexpr AHRSFilterStatusFlags_t AHRS_FILTER_STAT_FLAGS_ACCEL_BIAS_HIGH = 0x0200;
static constexpr AHRSFilterStatusFlags_t AHRS_FILTER_STAT_FLAGS_MAG_BIAS_HIGH = 0x1000;
static constexpr AHRSFilterStatusFlags_t AHRS_FILTER_STAT_FLAGS_HARD_IRON_EST_HIGH = 0x4000;
static constexpr AHRSFilterStatusFlags_t AHRS_FILTER_STAT_FLAGS_SOFT_IRON_EST_HIGH = 0x8000;

static constexpr uint8_t LORD_MIP_FIELD_FUNCTION_SELECTOR_APPLY_NEW = 0x01;
static constexpr uint8_t LORD_MIP_FIELD_FUNCTION_SELECTOR_READ_CURR = 0x02;
static constexpr uint8_t LORD_MIP_FIELD_FUNCTION_SELECTOR_APPLY_NEW_NO_ACK = 0x06;

typedef uint8_t LORDMIPFieldDescriptor_t;
static constexpr LORDMIPFieldDescriptor_t LORD_MIP_FIELD_DESC_INVALID = 0x0;
// Generic Response Set
static constexpr LORDMIPFieldDescriptor_t LORD_MIP_FIELD_DESC_ACK_RESP = 0xF1;

// Base Command Descriptors (0x01)
static constexpr LORDMIPFieldDescriptor_t LORD_MIP_FIELD_DESC_IDLE_MODE = 0x02;
static constexpr LORDMIPFieldDescriptor_t LORD_MIP_FIELD_DESC_DEV_INFO = 0x03;
static constexpr LORDMIPFieldDescriptor_t LORD_MIP_FIELD_DESC_RESUME = 0x06;
static constexpr LORDMIPFieldDescriptor_t LORD_MIP_FIELD_DESC_GPS_TIME = 0x72;
// Base Command Response Set
static constexpr LORDMIPFieldDescriptor_t LORD_MIP_FIELD_DESC_DEV_INFO_RESP = 0x81;

// 3DM Command Set (0x0C)
static constexpr LORDMIPFieldDescriptor_t LORD_MIP_FIELD_DESC_IMU_BASE_RATE = 0x06;
static constexpr LORDMIPFieldDescriptor_t LORD_MIP_FIELD_DESC_EST_FILTER_BASE_RATE = 0x0B;
static constexpr LORDMIPFieldDescriptor_t LORD_MIP_FIELD_DESC_IMU_MSG_FORMAT = 0x08;
static constexpr LORDMIPFieldDescriptor_t LORD_MIP_FIELD_DESC_EST_FILTER_MSG_FORMAT = 0x0A;
static constexpr LORDMIPFieldDescriptor_t LORD_MIP_FIELD_DESC_ENABLE_DATA_STREAM = 0x11;
// 3DM Command Response Set
static constexpr LORDMIPFieldDescriptor_t LORD_MIP_FIELD_DESC_IMU_BASE_RATE_RESP = 0x83;
static constexpr LORDMIPFieldDescriptor_t LORD_MIP_FIELD_DESC_EST_FILTER_BASE_RATE_RESP = 0x8A;
static constexpr LORDMIPFieldDescriptor_t LORD_MIP_FIELD_DESC_IMU_MSG_FORMAT_RESP = 0x80;

// Est Filter Command Set (0x0D)
static constexpr LORDMIPFieldDescriptor_t LORD_MIP_FIELD_DESC_HEADING_UPDATE_CONTROL = 0x18;
static constexpr LORDMIPFieldDescriptor_t LORD_MIP_FIELD_DESC_EST_CONTROL_FLAGS = 0x14;
static constexpr LORDMIPFieldDescriptor_t LORD_MIP_FIELD_DESC_PITCH_ROLL_AIDING_CONTROL = 0x4B;
static constexpr LORDMIPFieldDescriptor_t LORD_MIP_FIELD_DESC_AUTO_INIT_CTL = 0x19;
static constexpr LORDMIPFieldDescriptor_t LORD_MIP_FIELD_DESC_SENSOR_VEHICLE_TRANSFORM = 0x11;
static constexpr LORDMIPFieldDescriptor_t LORD_MIP_FIELD_DESC_REF_POSITION = 0x26;
static constexpr LORDMIPFieldDescriptor_t LORD_MIP_FIELD_DESC_MAG_CAPTURE_AUTO_CAL = 0x27;
static constexpr LORDMIPFieldDescriptor_t LORD_MIP_FIELD_DESC_INCLINATION_SRC = 0x4C;
static constexpr LORDMIPFieldDescriptor_t LORD_MIP_FIELD_DESC_DECLINATION_SRC = 0x43;

// IMU Data Set (0x80)
static constexpr uint8_t IMU_NUM_FIELD_DESCRIPTORS = 3; // Must match the count below
static constexpr LORDMIPFieldDescriptor_t LORD_MIP_FIELD_DESC_IMU_GPS_TIMESTAMP = 0x12; // IMU Time
static constexpr LORDMIPFieldDescriptor_t LORD_MIP_FIELD_DESC_IMU_STABILIZED_MAG = 0x10; // IMU Time
static constexpr LORDMIPFieldDescriptor_t LORD_MIP_FIELD_DESC_IMU_DELTA_VELOCITY = 0x08; // Velocity (v = v[0] + delta_v[t] + delta_v[t-1] ...)

// Est Filter Data Set (0x82)
constexpr uint8_t EST_FILTER_NUM_FIELD_DESCRIPTORS = 6; // Must match the count below
static constexpr LORDMIPFieldDescriptor_t LORD_MIP_FIELD_DESC_EST_FILTER_GPS_TIMESTAMP = 0x11; // Est Filter Time
static constexpr LORDMIPFieldDescriptor_t LORD_MIP_FIELD_DESC_EST_FILTER_STATUS = 0x10; // Status
static constexpr LORDMIPFieldDescriptor_t LORD_MIP_FIELD_DESC_EST_FILTER_QUATERNION = 0x03; // Orienation
static constexpr LORDMIPFieldDescriptor_t LORD_MIP_FIELD_DESC_EST_FILTER_LIN_ACCEL = 0x0D; // Linear acceleration
static constexpr LORDMIPFieldDescriptor_t LORD_MIP_FIELD_DESC_EST_FILTER_COMP_ANGULAR_RATE = 0x0E; // Angular rate
static constexpr LORDMIPFieldDescriptor_t LORD_MIP_FIELD_DESC_EST_FILTER_MAG_SOLUTION = 0x15; // Heading

static constexpr uint8_t LORD_MIP_GPS_TIME_FIELD_SELECTOR_WEEK_NUMBER = 0x01;
static constexpr uint8_t LORD_MIP_GPS_TIME_FIELD_SELECTOR_SECONDS = 0x02;

static constexpr uint8_t LORD_MIP_ACK_FIELD_NO_ERROR_VAL = 0x00;

static constexpr uint8_t LORD_MIP_FIELD_LENGTH_ACK = 0x4;
static constexpr uint8_t LORD_MIP_FIELD_LENGTH_DEV_INFO = 0x54;

static constexpr uint8_t LORD_MIP_FIELD_HEADING_CONTROL_INTERNAL = 0x01;

// These are not well-documented. They were extracted by querying the control flag field
// after incremental changes, all within the MIP Monitor GUI application
static constexpr uint16_t LORD_MIP_FIELD_EST_CONTROL_BASE_UNKNOWN_DEFAULT = 0xFF80;
static constexpr uint16_t LORD_MIP_FIELD_EST_CONTROL_FLAG_ENABLE_GYRO_BIAS = 0x0001;
static constexpr uint16_t LORD_MIP_FIELD_EST_CONTROL_FLAG_ENABLE_HARD_IRON_AUTOCAL = 0x0020;
static constexpr uint16_t LORD_MIP_FIELD_EST_CONTROL_FLAG_ENABLE_SOFT_IRON_AUTOCAL = 0x0040;

static constexpr uint8_t LORD_MIP_FIELD_ENABLE_GRAV_VECT_AIDING = 0x01;

static constexpr uint8_t LORD_MIP_FIELD_DEVICE_SELECTOR_IMU = 0x01;
static constexpr uint8_t LORD_MIP_FIELD_DEVICE_SELECTOR_EST_FILTER = 0x03;

static constexpr uint8_t LORD_MIP_FIELD_INC_DEC_SRC_NONE = 0x01;
static constexpr uint8_t LORD_MIP_FIELD_INC_DEC_SRC_WORLD_MAG_MODEL = 0x02;

static constexpr uint8_t LORD_MIP_FIELD_GENERIC_ENABLE = 0x01;
static constexpr uint16_t LORD_MIP_FIELD_GENERIC_VALID_FLAG = 0x0001;

static constexpr size_t LORD_MIP_FIELD_DATA_LENGTH_IMU_GPS_TIMESTAMP = 12;
static constexpr size_t LORD_MIP_FIELD_DATA_LENGTH_IMU_STABILIZED_MAG = 12;
static constexpr size_t LORD_MIP_FIELD_DATA_LENGTH_IMU_DELTA_VELOCITY = 12; // Velocity (v = v[0] + delta_v[t] + delta_v[t-1] ...)
static constexpr size_t LORD_MIP_FIELD_DATA_LENGTH_EST_FILTER_GPS_TIMESTAMP = 12; // Est Filter Time
static constexpr size_t LORD_MIP_FIELD_DATA_LENGTH_EST_FILTER_STATUS = 6; // Status
static constexpr size_t LORD_MIP_FIELD_DATA_LENGTH_EST_FILTER_QUATERNION = 18; // Orienation
static constexpr size_t LORD_MIP_FIELD_DATA_LENGTH_EST_FILTER_LIN_ACCEL = 14; // Linear acceleration
static constexpr size_t LORD_MIP_FIELD_DATA_LENGTH_EST_FILTER_COMP_ANGULAR_RATE = 14; // Angular rate
static constexpr size_t LORD_MIP_FIELD_DATA_LENGTH_EST_FILTER_MAG_SOLUTION = 22; // Heading

typedef uint8_t MIPDescriptorSet_t;
static constexpr MIPDescriptorSet_t MIP_DESC_SET_INVALID = 0x0;
static constexpr MIPDescriptorSet_t MIP_DESC_SET_BASE_CMD = 0x01;
static constexpr MIPDescriptorSet_t MIP_DESC_SET_3DM_CMD = 0x0C;
static constexpr MIPDescriptorSet_t MIP_DESC_SET_EST_FILT_CMD = 0x0D;
static constexpr MIPDescriptorSet_t MIP_DESC_SET_SYS_CMD = 0x7F;
static constexpr MIPDescriptorSet_t MIP_DESC_SET_IMU_DATA = 0x80;
static constexpr MIPDescriptorSet_t MIP_DESC_SET_EST_FILT_DATA = 0x82;

static constexpr uint8_t LORD_MIP_PKT_SYNC1 = 0x75;
static constexpr uint8_t LORD_MIP_PKT_SYNC2 = 0x65;
static constexpr uint8_t LORD_MIP_PKT_CHKSUM_LENGTH = 2;

class LORDMIPField
{
public:
  LORDMIPField(LORDMIPFieldDescriptor_t field_desc);

  // Overloaded push methods
  void pushBack(uint8_t val);
  void pushBack(uint16_t val);
  void pushBack(uint32_t val);
  void pushBack(float val);
  void pushBack(double val);

  bool extractVal(uint16_t &val_out, size_t binary_offset) const;
  bool extractVal(float &val_out, size_t binary_offset) const;
  bool extractVal(double &val_out, size_t binary_offset) const;

  inline uint8_t getFieldLength() const {return (uint8_t)(field_data.size() + 2);}

  static bool isValidResponseFieldDescriptor(MIPDescriptorSet_t pkt_descriptor, LORDMIPFieldDescriptor_t field_descriptor);

  inline void reset() {field_descriptor = LORD_MIP_FIELD_DESC_INVALID; field_data.clear();}

  inline bool validate(LORDMIPFieldDescriptor_t expected_field_desc, size_t expected_field_length) const
  {
    return ((field_descriptor == expected_field_desc) && (field_data.size() == expected_field_length));
  }

private:
  LORDMIPFieldDescriptor_t field_descriptor;
  std::vector<uint8_t> field_data;

  bool extractVal(void *dest, size_t binary_offset, size_t DATA_SIZE, void *swap_buffer) const;

  friend class LORDMIPPkt;
  friend class LORDAHRSDriver;
};

class LORDMIPPkt
{
public:
  LORDMIPPkt();
  LORDMIPPkt(MIPDescriptorSet_t ds);
  void addField(const LORDMIPField &field);
  void serialize(std::vector<uint8_t> &bytes_out) const;
  inline void reset() {descriptor = MIP_DESC_SET_INVALID; fields.clear();}

  static bool isValidPktDescriptor(uint8_t descriptor_byte);

private:
  MIPDescriptorSet_t descriptor;
  std::vector<LORDMIPField> fields;

  friend class LORDAHRSDriver;
};

struct LORDAHRSDeviceInfo
{
  uint16_t fw_version = 0;
  std::string model_name = "None";
  std::string model_num = "None";
  std::string serial_num = "None";
  std::string options = "None";

  void print();
};

struct LORDAHRSIMUGPSTimestampData
{
  double gps_tow = 0.0;
  uint16_t gps_week_number = 0;
  bool pps_present = false;
  uint8_t gps_time_refresh_toggle = 0;
  bool gps_time_initialized = false;

  inline void print()
  {
    printf("IMU GPS Timestamp:\n");
    printf("\tTime of Week: %f, GPS Week Number: %u\n", gps_tow, gps_week_number);
    printf("\tGPS Time Initialized: %s, PPS Present: %s, GPS Refresh Toggle: %u\n",
           boolToCString(gps_time_initialized), boolToCString(pps_present), gps_time_refresh_toggle);
  }
};

struct LORDAHRSIMUStabilizedMagData
{
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;

  inline void print()
  {
    const float field_strength = sqrt((x*x) + (y*y) + (z*z));
    printf("IMU Stabilized Mag Field\n");
    printf("\tX: %f, Y: %f, Z: %f gauss\n", x, y, z);
    printf("\tMeasured Field Strength: %f gauss\n", field_strength);
  }
};

struct LORDAHRSIMUDeltaVelocity
{
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;

  static constexpr float G_SEC_TO_MPS = 9.80665; // m/s^2

  inline void print()
  {
    printf("IMU Delta Velocity Vector:\n");
    // assumes g*s --> m/s conversion has already taken place
    printf("\tX: %f, Y: %f, Z: %f m/s\n", x, y, z);
  }
};

struct LORDAHRSIMUData
{
  LORDAHRSIMUGPSTimestampData gps_timestamp;
  LORDAHRSIMUStabilizedMagData stabilized_mag;
  LORDAHRSIMUDeltaVelocity delta_v_vector;

  inline void print()
  {
    printf("*** IMU Data ***\n");
    gps_timestamp.print();
    stabilized_mag.print();
    delta_v_vector.print();
    printf("****************\n");
  }
};

struct LORDAHRSEstFiltGPSTimestampData
{
  double gps_tow = 0.0;
  uint16_t gps_week_number = 0;
  bool valid = false;

  inline void print()
  {
    printf("Est. Filter GPS Timestamp:\n");
    printf("\tValid = %s\n", boolToCStringValid(valid));
    printf("\tTime of Week: %f, GPS Week Number: %u\n", gps_tow, gps_week_number);
  }
};

struct LORDAHRSEstFiltStatus
{
  static constexpr uint16_t STATE_STARTUP = 0x0000;
  static constexpr uint16_t STATE_INIT = 0x0001;
  static constexpr uint16_t STATE_RUNNING_VALID = 0x0002;
  static constexpr uint16_t STATE_RUNNING_ERROR = 0x0003;

  static constexpr uint16_t DYN_MODE_PORTABLE = 0x0001;
  static constexpr uint16_t DYN_MODE_AUTOMOTIVE = 0x0002;
  static constexpr uint16_t DYN_MODE_AIRBORNE = 0x0003;

  static constexpr uint16_t FLAGS_NONE = 0x0000;

  static constexpr uint16_t INIT_FLAGS_ATTITUDE_UNINT = 0x1000;
  static constexpr uint16_t INIT_FLAGS_POS_VEL_UNINT = 0x2000;

  static constexpr uint16_t RUNNING_FLAGS_IMU_UNAVAILABLE = 0x0001;
  static constexpr uint16_t RUNNING_FLAGS_GNSS = 0x0002;
  static constexpr uint16_t RUNNING_FLAGS_MATRIX_SINGULARITY = 0x0008;
  static constexpr uint16_t RUNNING_FLAGS_POS_COVARIANCE_HIGH = 0x0010;
  static constexpr uint16_t RUNNING_FLAGS_VEL_COVARIANCE_HIGH = 0x0020;
  static constexpr uint16_t RUNNING_FLAGS_ATT_COVARIANCE_HIGH = 0x0040;
  static constexpr uint16_t RUNNING_FLAGS_NAN = 0x0080;
  static constexpr uint16_t RUNNING_FLAGS_GYRO_BIAS_EST_HIGH = 0x0100;
  static constexpr uint16_t RUNNING_FLAGS_ACCEL_BIAS_EST_HIGH = 0x0200;
  static constexpr uint16_t RUNNING_FLAGS_GYRO_SCALE_EST_HIGH = 0x0400;
  static constexpr uint16_t RUNNING_FLAGS_ACCEL_SCALE_EST_HIGH = 0x0800;
  static constexpr uint16_t RUNNING_FLAGS_MAG_BIAS_EST_HIGH = 0x1000;
  static constexpr uint16_t RUNNING_FLAGS_MAG_HARD_IRON_EST_HIGH = 0x4000;
  static constexpr uint16_t RUNNING_FLAGS_MAG_SOFT_IRON_EST_HIGH = 0x8000;

  uint16_t state = STATE_STARTUP;
  uint16_t dynamics_mode = DYN_MODE_PORTABLE;
  uint16_t flags = FLAGS_NONE;

  inline void printState()
  {
    const char *state_string;
    switch (state)
    {
      case STATE_STARTUP:
        state_string = "STARTUP";
        break;
      case STATE_INIT:
        state_string = "INIT";
        break;;
      case STATE_RUNNING_VALID:
        state_string = "RUNNING";
        break;
      case STATE_RUNNING_ERROR:
        state_string = "ERROR";
        break;
      default:
        state_string = "UNKNOWN";
    }
    printf("\tState: %s\n", state_string);
  }

  inline void printDynMode()
  {
    const char *mode_string;
    switch (dynamics_mode)
    {
      case DYN_MODE_PORTABLE:
        mode_string = "PORTABLE";
        break;
      case DYN_MODE_AUTOMOTIVE:
        mode_string = "AUTOMOTIVE";
        break;
      case DYN_MODE_AIRBORNE:
        mode_string = "AIRBORNE";
        break;
      default:
        mode_string = "UNKNOWN";
    }
    printf("\tDynamics Mode: %s\n", mode_string);
  }

  inline void printFlags()
  {
    printf("\tFlags: 0x%04X\n", flags);
    if (FLAGS_NONE == flags)
    {
      return;
    }
    if ((STATE_STARTUP == state) || (STATE_RUNNING_VALID == state))
    {
      printf("\t\t(Interpretation unspecified)\n");
    }
    else if (STATE_INIT == state)
    {
      if (flags & INIT_FLAGS_ATTITUDE_UNINT)
      {
        printf("\t\tUnitialized attitude\n");
      }
      if (flags & INIT_FLAGS_POS_VEL_UNINT)
      {
        printf("\t\tUnitialized velocity\n");
      }
    }
    else if (STATE_RUNNING_VALID == state)
    {
      if (flags & RUNNING_FLAGS_IMU_UNAVAILABLE)
      {
        printf("\t\tIMU UNAVAILABLE\n");
      }
      if (flags & RUNNING_FLAGS_GNSS)
      {
        printf("\t\tGNSS\n");
      }
      if (flags & RUNNING_FLAGS_MATRIX_SINGULARITY)
      {
        printf("\t\tMATRIX SINGULARITY\n");
      }
      if (flags & RUNNING_FLAGS_POS_COVARIANCE_HIGH)
      {
        printf("\t\tPOSITION COVAR HIGH\n");
      }
      if (flags & RUNNING_FLAGS_VEL_COVARIANCE_HIGH)
      {
        printf("\t\tVELOCITY COVAR HIGH\n");
      }
      if (flags & RUNNING_FLAGS_ATT_COVARIANCE_HIGH)
      {
        printf("\t\tATTITUDE COVAR HIGH\n");
      }
      if (flags & RUNNING_FLAGS_NAN)
      {
        printf("\t\tNAN IN COMPUTATION\n");
      }
      if (flags & RUNNING_FLAGS_GYRO_BIAS_EST_HIGH)
      {
        printf("\t\tGYRO BIAS ESTIMATE HIGH\n");
      }
      if (flags & RUNNING_FLAGS_ACCEL_BIAS_EST_HIGH)
      {
        printf("\t\tACCELEROMETER BIAS ESTIMATE HIGH\n");
      }
      if (flags & RUNNING_FLAGS_GYRO_SCALE_EST_HIGH)
      {
        printf("\t\tGYRO SCALE FACTOR ESTIMATE HIGH\n");
      }
      if (flags & RUNNING_FLAGS_ACCEL_SCALE_EST_HIGH)
      {
        printf("\t\tACCELEROMETER SCALE FACTOR ESTIMATE HIGH\n");
      }
      if (flags & RUNNING_FLAGS_MAG_BIAS_EST_HIGH)
      {
        printf("\t\tMAG BIAS ESTIMATE HIGH\n");
      }
      if (flags & RUNNING_FLAGS_MAG_HARD_IRON_EST_HIGH)
      {
        printf("\t\tMAG HARD IRON ESTIMATE HIGH\n");
      }
      if (flags & RUNNING_FLAGS_MAG_SOFT_IRON_EST_HIGH)
      {
        printf("\t\tMAG SOFT IRON ESTIMATE HIGH\n");
      }
    }
  }

  inline void print()
  {
    printf("Est. Filter Status:\n");
    printState();
    printDynMode();
    printFlags();
  }
};

struct LORDAHRSEstFiltQuaternion
{
  float q0 = 0.0f;
  float q1_i = 0.0f;
  float q2_j = 0.0f;
  float q3_k = 0.0f;
  bool valid = false;

  inline void print()
  {
    printf("Est. Filter Quaternion\n");
    printf("\tValid: %s\n", boolToCStringValid(valid));
    printf("\t[%.3f,%.3fi,%.3fj,%.3fk]\n", q0, q1_i, q2_j, q3_k);
  }
};

struct LORDAHRSEstFiltLinearAccel
{
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  bool valid = false;

  inline void print()
  {
    printf("Est. Filter Linear Acceleration\n");
    printf("\tValid: %s\n", boolToCStringValid(valid));
    printf("\tX:%f, Y:%f, Z:%f m/s^2\n", x, y, z);
  }
};

struct LORDAHRSEstFiltAngularRate
{
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  bool valid = false;

  inline void print()
  {
    printf("Est. Filter Angular Rate\n");
    printf("\tValid: %s\n", boolToCStringValid(valid));
    printf("\tX:%f, Y:%f, Z:%f rad/s\n", x, y, z);
  }
};

struct LORDAHRSEstFiltMagSolution
{
  float north_g = 0.0f;
  float east_g = 0.0f;
  float down_g = 0.0f;
  float inclination_rad = 0.0f;
  float declination_rad = 0.0f;
  bool valid = false;

  inline void print()
  {
    const float exp_field_strength = sqrt(north_g*north_g + east_g*east_g + down_g*down_g);
    printf("Est. Filter Mag Solution\n");
    printf("\tValid: %s\n", boolToCStringValid(valid));
    printf("\tNorth: %f, East: %f, Down: %f gauss\n", north_g, east_g, down_g);
    printf("\tInclination: %f, Declination: %f radians\n", inclination_rad, declination_rad);
    printf("\tExpected Field Strength: %f gauss\n", exp_field_strength);
  }
};

struct LORDAHRSEstFiltData
{
  LORDAHRSEstFiltGPSTimestampData gps_timestamp;
  LORDAHRSEstFiltStatus filt_status;
  LORDAHRSEstFiltQuaternion quaternion;
  LORDAHRSEstFiltLinearAccel lin_accel;
  LORDAHRSEstFiltAngularRate angular_rate;
  LORDAHRSEstFiltMagSolution mag_solution;

  inline void print()
  {
    printf("*** Estimation Filter Data ***\n");
    gps_timestamp.print();
    filt_status.print();
    quaternion.print();
    lin_accel.print();
    angular_rate.print();
    mag_solution.print();
    printf("******************************\n");
  }
};

struct LORDAHRSRollPitchYaw
{
  float roll_rad = 0.0f;
  float pitch_rad = 0.0f;
  float yaw_rad = 0.0f;

  LORDAHRSRollPitchYaw() :
    roll_rad{0.0f},
    pitch_rad{0.0f},
    yaw_rad{0.0f}
  {}

  LORDAHRSRollPitchYaw(float r, float p, float y) :
    roll_rad{r},
    pitch_rad{r},
    yaw_rad{r}
  {}
};

struct LORDAHRSReferencePosition
{
  double latitude_deg = 0.0;
  double longitude_deg = 0.0;
  double altitude_m = 0.0;
};

class LORDAHRSDriver : public AHRSDriver
{
public:
  LORDAHRSDriver();
  ~LORDAHRSDriver();

  bool init(const char* serial_dev, float data_rate_hz, uint32_t current_time_posix,
            const LORDAHRSRollPitchYaw &sensor_to_vehicle);
  bool receiveLatestData(AHRSDataSet &data_out) override;
  bool updateSystemTime(uint32_t posix_time);
  bool updateReferencePosition(const LORDAHRSReferencePosition &pos);
  bool magnetometerCaptureAutoCal();
  bool updateDataStreamRate(float data_rate_hz);

private:
    bool initialized = false;
    int serial_fd = -1;
    LORDAHRSDeviceInfo dev_info;
    float data_stream_rate_hz = 1.0f;
    bool has_valid_reference_position = false;
    bool world_mag_model_enabled = false;
    // std::queue<LORDMIPPkt> deferred_rx_packets;

    bool transmitMIPPktSynchronous(const LORDMIPPkt &pkt, std::vector<LORDMIPField> *resp_fields, uint32_t rx_timeout_ms = 2000);

    bool rcvPktBlocking(LORDMIPPkt &pkt_out, const std::chrono::milliseconds &timeout_ms);

    // Setup methods
    bool setIdleMode(bool idle);
    bool queryDeviceInfo();
    bool enableIncDecWorldMagModel(bool enabled);
    bool queryDataBaseRates(uint16_t &imu_base_rate, uint16_t &est_filter_base_rate);
    bool initializeIMUDataStreamFormat(uint16_t imu_rate_decimation);
    bool initializeEstFilterDataStreamFormat(uint16_t est_filter_rate_decimation);
    bool initializeHeadingUpdateSource();
    bool initializeSensorToVehicleTransform(const LORDAHRSRollPitchYaw &transform);
    bool setEstFilterControlFlags();
    bool enableGravityVectorAiding();
    bool enableConfiguredDataStreams();
    bool enableKalmanFilterAutoInitialization();

    bool parseDataPkts(const LORDMIPPkt &imu_pkt, const LORDMIPPkt &est_filt_pkt, AHRSDataSet &data_out);

    static bool parseIMUDataPkt(const LORDMIPPkt &imu_pkt, LORDAHRSIMUData &imu_data_out);
    static bool parseIMUGPSTimestampField(const LORDMIPField &field, LORDAHRSIMUGPSTimestampData &data_out);
    static bool parseIMUStabilizedMagField(const LORDMIPField &field, LORDAHRSIMUStabilizedMagData &data_out);
    static bool parseIMUDeltaVelocity(const LORDMIPField &field,  LORDAHRSIMUDeltaVelocity &data_out);

    static bool parseEstFilterDataPkt(const LORDMIPPkt &est_filt_pkt, LORDAHRSEstFiltData &est_filt_data_out);
    static bool parseEstFilterGPSTimestamp(const LORDMIPField &field, LORDAHRSEstFiltGPSTimestampData &data_out);
    static bool parseEstFilterStatus(const LORDMIPField &field, LORDAHRSEstFiltStatus &data_out);
    static bool parseEstFilterQuaternion(const LORDMIPField &field, LORDAHRSEstFiltQuaternion &data_out);
    static bool parseEstFilterLinAccel(const LORDMIPField &field, LORDAHRSEstFiltLinearAccel &data_out);
    static bool parseEstFilterAngularRate(const LORDMIPField &field, LORDAHRSEstFiltAngularRate &data_out);
    static bool parseEstFilterMagSolution(const LORDMIPField &field, LORDAHRSEstFiltMagSolution &data_out);

    void mergeAHRSData(const LORDAHRSIMUData &imu_data_in, const LORDAHRSEstFiltData &est_filt_data_in,
                       AHRSDataSet &merged_out);
}; // class LordAHRSDriver

} // namespace Numurus
#endif
