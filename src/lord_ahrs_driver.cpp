#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>

#include <algorithm>
#include <iterator>
//#include <../include/ahrs_driver.h>
#include <lord_ahrs_driver.h>

//#define PRINT_RX_BYTES
//#define PRINT_TX_BYTES

#warning "Lord AHRS Driver needs thread safety"

namespace Numurus
{

static const char* mipErrorCodeToString(uint8_t error_code)
{
  switch(error_code)
  {
    // As defined in Data Comms Protocol Doc
    case 0x00: return("Success");
    case 0x01: return("Unknown Command");
    case 0x02: return("Invalid Checksum");
    case 0x03: return("Invalid Parameteter");
    case 0x04: return("Device Can't Complete Command");
    case 0x05: return("Command Timeout");
    default: return("Unknown Error Code");
  }
}

LORDMIPField::LORDMIPField(LORDMIPFieldDescriptor_t field_desc) :
  field_descriptor{field_desc}
{}

void LORDMIPField::pushBack(uint8_t val)
{
  field_data.push_back(val);
}

void LORDMIPField::pushBack(uint16_t val)
{
  // Wire data is big-endian, but x86_64 and ARM are small-endian, so swap
  const uint8_t first_byte = (uint8_t)(val >> 8);
  pushBack(first_byte);
  const uint8_t second_byte = (uint8_t)(val & 0xFF);
  pushBack(second_byte);
}

void LORDMIPField::pushBack(uint32_t val)
{
  // Wire data is big-endian, but x86_64 and ARM are small-endian, so swap
  // TODO: Make endianness configurable or auto-detected?

  // Beware of unaligned data access -- just mask, shift, and copy as necessary
  const uint8_t first_byte = (uint8_t)(val >> 24);
  pushBack(first_byte);
  const uint8_t second_byte = (uint8_t)((val >> 16) & 0xFF);
  pushBack(second_byte);
  const uint8_t third_byte = (uint8_t)((val >> 8) & 0xFF);
  pushBack(third_byte);
  const uint8_t fourth_byte = (uint8_t)(val & 0xFF);
  pushBack(fourth_byte);
}

union Float2Bytes
{
  float f;
  uint8_t bytes[sizeof(float)];
};

void LORDMIPField::pushBack(float val)
{
  Float2Bytes f2b;
  f2b.f = val;
  // Start at the highest index to convert to big endian for wire format
  for (int i = sizeof(float) - 1; i >= 0; --i)
  {
    pushBack(f2b.bytes[i]);
  }
}

union Double2Bytes
{
  double d;
  uint8_t bytes[sizeof(double)];
};

void LORDMIPField::pushBack(double val)
{
  Double2Bytes d2b;
  d2b.d =  val;
  // Start at the highest index to convert to big endian for wire format
  for (int i = sizeof(double) - 1; i >= 0; --i)
  {
    pushBack(d2b.bytes[i]);
  }
}

bool LORDMIPField::extractVal(void *dest, size_t binary_offset, size_t DATA_SIZE, void *swap_buffer) const
{
  if (field_data.size() < binary_offset + DATA_SIZE)
  {
    return false;
  }

  // Wire data is big endian
  uint8_t *byte_swapped = (uint8_t *)swap_buffer;
  for (size_t i = 0; i < DATA_SIZE; ++i)
  {
    *(byte_swapped + DATA_SIZE - i - 1) = field_data[binary_offset + i];
  }
  memcpy(dest, byte_swapped, DATA_SIZE);
  return true;
}

bool LORDMIPField::extractVal(uint16_t &val_out, size_t binary_offset) const
{
  static constexpr size_t DATA_SIZE = 2;
  uint8_t byte_swapped[DATA_SIZE];
  return extractVal(&val_out, binary_offset, DATA_SIZE, byte_swapped);
}

bool LORDMIPField::extractVal(float &val_out, size_t binary_offset) const
{
  static constexpr size_t DATA_SIZE = 4;
  uint8_t byte_swapped[DATA_SIZE];
  return extractVal(&val_out, binary_offset, DATA_SIZE, byte_swapped);
}

bool LORDMIPField::extractVal(double &val_out, size_t binary_offset) const
{
  static constexpr size_t DATA_SIZE = 8;
  uint8_t byte_swapped[DATA_SIZE];
  return extractVal(&val_out, binary_offset, DATA_SIZE, byte_swapped);
}

bool LORDMIPField::isValidResponseFieldDescriptor(MIPDescriptorSet_t pkt_descriptor, LORDMIPFieldDescriptor_t field_descriptor)
{
  // The ACK response field is always valid
  if (LORD_MIP_FIELD_DESC_ACK_RESP == field_descriptor)
  {
    return true;
  }

  // Check these in pkt-then-field order, as the same field descriptors are used
  // for different packet descriptors
  switch(pkt_descriptor)
  {
    case MIP_DESC_SET_BASE_CMD:
    {
      switch(field_descriptor)
      {
        case LORD_MIP_FIELD_DESC_DEV_INFO_RESP:
          return true;
        default:
          return false;
      }
    } break;
    case MIP_DESC_SET_3DM_CMD:
    {
      switch(field_descriptor)
      {
        case LORD_MIP_FIELD_DESC_IMU_BASE_RATE_RESP:
        case LORD_MIP_FIELD_DESC_EST_FILTER_BASE_RATE_RESP:
          return true;
        default:
          return false;
      }
    }
    case MIP_DESC_SET_EST_FILT_CMD:
    {
      switch(field_descriptor)
      {

        default:
          return false;
      }
    } break;
    case MIP_DESC_SET_SYS_CMD:
    {
      switch(field_descriptor)
      {

        default:
          return false;
      }
    } break;
    case MIP_DESC_SET_IMU_DATA:
    {
      switch(field_descriptor)
      {
        case LORD_MIP_FIELD_DESC_IMU_GPS_TIMESTAMP:
        case LORD_MIP_FIELD_DESC_IMU_STABILIZED_MAG:
        case LORD_MIP_FIELD_DESC_IMU_DELTA_VELOCITY:
          return true;
        default:
          return false;
      }
    } break;
    case MIP_DESC_SET_EST_FILT_DATA:
    {
      switch(field_descriptor)
      {
        case LORD_MIP_FIELD_DESC_EST_FILTER_GPS_TIMESTAMP:
        case LORD_MIP_FIELD_DESC_EST_FILTER_STATUS:
        case LORD_MIP_FIELD_DESC_EST_FILTER_QUATERNION:
        case LORD_MIP_FIELD_DESC_EST_FILTER_LIN_ACCEL:
        case LORD_MIP_FIELD_DESC_EST_FILTER_COMP_ANGULAR_RATE:
        case LORD_MIP_FIELD_DESC_EST_FILTER_MAG_SOLUTION:
          return true;
        default:
          return false;
      }
    } break;
    default: // Unknown pkt descriptor type
      return false;
  }
}

LORDMIPPkt::LORDMIPPkt() :
  descriptor{LORD_MIP_FIELD_DESC_INVALID}
{
}

LORDMIPPkt::LORDMIPPkt(MIPDescriptorSet_t ds) :
  descriptor{ds}
{
}

void LORDMIPPkt::addField(const LORDMIPField &field)
{
  fields.push_back(field);
}

void LORDMIPPkt::serialize(std::vector<uint8_t> &bytes_out) const
{
  bytes_out.push_back(LORD_MIP_PKT_SYNC1);
  bytes_out.push_back(LORD_MIP_PKT_SYNC2);
  bytes_out.push_back(descriptor);
  uint8_t payload_length = 0; // Defer actually adding this to the buffer until we know it

  for (auto field : fields)
  {
    const uint8_t field_length = field.field_data.size() + 2;
    bytes_out.push_back(field_length);
    bytes_out.push_back(field.field_descriptor);

    std::copy(field.field_data.begin(), field.field_data.end(), std::back_inserter(bytes_out));
    payload_length += field_length;
  }

  // Now that we know it, add the payload length
  auto it = bytes_out.begin() + 3;
  bytes_out.insert(it, payload_length);

  // Now, compute the checksum
  uint8_t chksum_byte_1 = 0;
  uint8_t chksum_byte_2 = 0;
  for (auto b : bytes_out)
  {
    chksum_byte_1 += b;
    chksum_byte_2 += chksum_byte_1;
  }
  // And add it to the serial buffer
  bytes_out.push_back(chksum_byte_1);
  bytes_out.push_back(chksum_byte_2);
}

bool LORDMIPPkt::isValidPktDescriptor(uint8_t descriptor_byte)
{
  switch(descriptor_byte)
  {
    case MIP_DESC_SET_INVALID:
    case MIP_DESC_SET_BASE_CMD:
    case MIP_DESC_SET_3DM_CMD:
    case MIP_DESC_SET_EST_FILT_CMD:
    case MIP_DESC_SET_SYS_CMD:
    case MIP_DESC_SET_IMU_DATA:
    case MIP_DESC_SET_EST_FILT_DATA:
      return true;
    default:
      return false;
  }
}

void LORDAHRSDeviceInfo::print()
{
  printf("AHRS Driver Device Info:\n");
  printf("\tF/W Version: %u\n", fw_version);
  printf("\tModel: %s\n", model_name.c_str());
  printf("\tModel Num: %s\n", model_num.c_str());
  printf("\tSerial Num: %s\n", serial_num.c_str());
  printf("\tOptions: %s\n\n", options.c_str());
}

LORDAHRSDriver::LORDAHRSDriver()
{
}

LORDAHRSDriver::~LORDAHRSDriver()
{
  if (-1 != serial_fd)
  {
    close(serial_fd);
  }
}

bool LORDAHRSDriver::init(const char* serial_dev, float data_rate_hz, uint32_t current_time_posix,
                          const AHRSRollPitchYaw &sensor_to_vehicle)
{
  initialized = false;

  // Open the serial port
  serial_fd = ::open(serial_dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (-1 == serial_fd)
  {
    printf("Failed to open the serial port\n");
    return false;
  }

  struct termios options;
  if (tcgetattr(serial_fd, &options) == -1) {
    printf("Unable to get current termios prior to reconfig (%s)\n", strerror(errno));
    return -1;
  }

  // set up raw mode / no echo / binary
  options.c_cflag |= (tcflag_t)  (CLOCAL | CREAD);
  options.c_lflag &= (tcflag_t) ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL |
                                       ISIG | IEXTEN); //|ECHOPRT

  options.c_oflag &= (tcflag_t) ~(OPOST);
  options.c_iflag &= (tcflag_t) ~(INLCR | IGNCR | ICRNL | IGNBRK);
  #ifdef IUCLC
    options.c_iflag &= (tcflag_t) ~IUCLC;
  #endif
  #ifdef PARMRK
    options.c_iflag &= (tcflag_t) ~PARMRK;
  #endif
  ::cfsetispeed(&options, B115200); // 115200 baud
  ::cfsetospeed(&options, B115200);
  options.c_cflag |= CS8; // 8 bits
  options.c_cflag &= (tcflag_t) ~(CSTOPB); // 1 stop bit
  options.c_iflag &= (tcflag_t) ~(INPCK | ISTRIP);
  options.c_cflag &= (tcflag_t) ~(PARENB | PARODD); // No parity bit
  options.c_iflag &= (tcflag_t) ~(IXON | IXOFF | IXANY); // No s/w flow control
  options.c_cflag &= (unsigned long) ~(CRTSCTS); // No h/w flow control
  // activate settings
  ::tcsetattr (serial_fd, TCSANOW, &options);

  // Now we can start initializing the device via serial port.

  // Put device into idle mode (0x1, 0x2)
  if (false == setIdleMode(true))
  {
    return false;
  }

  // Set the clock, GPS Time Update (0x1, 0x72)
  if (false == updateSystemTime(current_time_posix))
  {
    return false;
  }

  // Collect a bunch of information about the device (0x01, 0x03)
  if (false == queryDeviceInfo())
  {
    return false;
  }

  // Initially, use no inclination/declination because we don't yet have a valid
  // reference position so it can't be derived from the World Magnetic Model
  if (false == enableIncDecWorldMagModel(false))
  {
    return false;
  }

  if (false == updateDataStreamRate(data_rate_hz))
  {
    return false;
  }

  // Set the heading update source to magnetometer (0x0D, 0x18)
  if (false == initializeHeadingUpdateSource())
  {
    return false;
  }

  // Initialize the sensor to vehicle transform (0x0D, 0x11)
  if (false == initializeSensorToVehicleTransform(sensor_to_vehicle))
  {
    return false;
  }

  // Set estimation filter flags (0x0D, 0x14)
  if (false == setEstFilterControlFlags())
  {
    return false;
  }

  // Enable the gravity vector aiding of pitch/roll (0x0D, 0x4B)
  if (false == enableGravityVectorAiding())
  {
    return false;
  }

  // Enable Kalman filter auto-initialize (0x0D, 0x19)
  if (false == enableKalmanFilterAutoInitialization())
  {
    return false;
  }

  //  Enable the data streams (0x0C, 0x11 AND 0x11 (one for each stream))
  if (false == enableConfiguredDataStreams())
  {
    return false;
  }

  // Now flush the receive buffer for a clean slate
  ::tcflush(serial_fd, TCIFLUSH);

  initialized = true;
  return initialized;
}

bool LORDAHRSDriver::receiveLatestData(AHRSDataSet &data_out)
{
  // Just Testing
  LORDMIPPkt pkt_out_1;
  LORDMIPPkt pkt_out_2;
  // Allow twice as long as necessary
  const float rate_for_timeout_hz = (data_stream_rate_hz > 0.0f)? data_stream_rate_hz : 1.0f;
  std::chrono::milliseconds timeout = std::chrono::duration<int, std::milli>((int)(2000.0f / rate_for_timeout_hz));
  if ((false == rcvPktBlocking(pkt_out_1, timeout)) ||
      (false == rcvPktBlocking(pkt_out_2, timeout)))
    {
      printf("Failed to acquire two packets\n");
      return false;
    }

  if (pkt_out_1.descriptor == MIP_DESC_SET_IMU_DATA && pkt_out_2.descriptor == MIP_DESC_SET_EST_FILT_DATA)
  {
    if (false == parseDataPkts(pkt_out_1, pkt_out_2, data_out))
    {
      return false;
    }
  }
  else if (pkt_out_2.descriptor == MIP_DESC_SET_IMU_DATA && pkt_out_1.descriptor == MIP_DESC_SET_EST_FILT_DATA)
  {
    if (false == parseDataPkts(pkt_out_2, pkt_out_1, data_out))
    {
      return false;
    }
  }

  // End testing

  // TODO:
  return true;
}

static constexpr uint32_t UNIX_TO_GPS_EPOCH_OFFSET = 315964800; // Jan 1, 1970 to Jan 6, 1980
static constexpr uint32_t SECS_IN_A_WEEK = 604800; // 60 * 60 *24 *7

static void posixToGPSTime(uint32_t posix_time_in, uint32_t &gps_week_number_out, uint32_t &gps_secs_in_week)
{
  // Note, this doesn't produce a real GPS time because it will include leap seconds, but GPS time does not.
  // Thats okay as long as we always compute and return a POSIX time stamp using this same conversion
  // in the data output, too.
  gps_week_number_out = (posix_time_in - UNIX_TO_GPS_EPOCH_OFFSET) / SECS_IN_A_WEEK;
  gps_secs_in_week = (posix_time_in - UNIX_TO_GPS_EPOCH_OFFSET) % SECS_IN_A_WEEK;
}

static void gpsToPosixTime(double gps_tow, uint16_t gps_week, double &posix_time_out)
{
  const double secs_from_weeks = gps_week * SECS_IN_A_WEEK;
  posix_time_out = secs_from_weeks + gps_tow + UNIX_TO_GPS_EPOCH_OFFSET;
}

bool LORDAHRSDriver::updateSystemTime(uint32_t posix_time)
{
  uint32_t week_number;
  uint32_t secs_in_current_week;
  posixToGPSTime(posix_time, week_number, secs_in_current_week);

  // Now construct the packets -- This driver requires a single field per packet, so construct
  // two separate packets and transmit them.
  LORDMIPField update_week_field(LORD_MIP_FIELD_DESC_GPS_TIME);
  update_week_field.pushBack(LORD_MIP_FIELD_FUNCTION_SELECTOR_APPLY_NEW);
  update_week_field.pushBack(LORD_MIP_GPS_TIME_FIELD_SELECTOR_WEEK_NUMBER);
  update_week_field.pushBack(week_number);
  LORDMIPPkt update_gps_week_pkt(MIP_DESC_SET_BASE_CMD);
  update_gps_week_pkt.addField(update_week_field);

  if (false == transmitMIPPktSynchronous(update_gps_week_pkt, nullptr))
  {
    return false;
  }

  LORDMIPField update_seconds_field(LORD_MIP_FIELD_DESC_GPS_TIME);
  update_seconds_field.pushBack(LORD_MIP_FIELD_FUNCTION_SELECTOR_APPLY_NEW);
  update_seconds_field.pushBack(LORD_MIP_GPS_TIME_FIELD_SELECTOR_SECONDS);
  update_seconds_field.pushBack(secs_in_current_week);
  LORDMIPPkt update_gps_seconds_pkt(MIP_DESC_SET_BASE_CMD);
  update_gps_seconds_pkt.addField(update_seconds_field);

  // No expected response besides the ACK/NACK handled in transmitMIPPktSynchronous
  return transmitMIPPktSynchronous(update_gps_seconds_pkt, nullptr);
}

bool LORDAHRSDriver::updateReferencePosition(const AHRSReferencePosition &pos)
{
  if (false == setIdleMode(true))
  {
    return false;
  }

  LORDMIPField update_pos_field(LORD_MIP_FIELD_DESC_REF_POSITION);
  update_pos_field.pushBack(LORD_MIP_FIELD_FUNCTION_SELECTOR_APPLY_NEW);
  update_pos_field.pushBack(LORD_MIP_FIELD_GENERIC_ENABLE);
  update_pos_field.pushBack(pos.latitude_deg);
  update_pos_field.pushBack(pos.longitude_deg);
  update_pos_field.pushBack(pos.altitude_m);
  LORDMIPPkt update_pos_pkt(MIP_DESC_SET_EST_FILT_CMD);
  update_pos_pkt.addField(update_pos_field);

  // No expected response besides the ACK/NACK handled in transmitMIPPktSynchronous
  // but want to store whether this ever succeeds in the member variable
  const bool pos_accepted = transmitMIPPktSynchronous(update_pos_pkt, nullptr);
  if (false == has_valid_reference_position) // it is a sticky so only update it to true once
  {
    has_valid_reference_position = pos_accepted;
  }

  // Now, if successful enable the World Mag. Model as inclination/declination source
  if (true == pos_accepted && false == world_mag_model_enabled)
  {
    world_mag_model_enabled = enableIncDecWorldMagModel(true);
  }

  setIdleMode(false);

  return (pos_accepted && world_mag_model_enabled);
}

bool LORDAHRSDriver::magnetometerCaptureAutoCal()
{
  // Per LORD documentation, only allow this if we have a valid reference position:
  if (false == has_valid_reference_position)
  {
    printf("Magnetometer auto-calibration not allowed without a valid reference position\n");
    return false;
  }

  LORDMIPField mag_capture_autocal_field(LORD_MIP_FIELD_DESC_MAG_CAPTURE_AUTO_CAL);
  mag_capture_autocal_field.pushBack(LORD_MIP_FIELD_FUNCTION_SELECTOR_APPLY_NEW);
  LORDMIPPkt mag_capture_autocal_pkt(MIP_DESC_SET_EST_FILT_CMD);
  mag_capture_autocal_pkt.addField(mag_capture_autocal_field);

  // No expected response besides the ACK/NACK handled in transmitMIPPktSynchronous
  return transmitMIPPktSynchronous(mag_capture_autocal_pkt, nullptr);
}

bool LORDAHRSDriver::updateDataStreamRate(float data_rate_hz)
{
  // Collect the IMU and E.F. base rate to be used when setting messsage rates (0x0C, 0x06 AND 0x0B)
  uint16_t imu_base_rate;
  uint16_t est_filter_base_rate;
  if (false == queryDataBaseRates(imu_base_rate, est_filter_base_rate))
  {
    return false;
  }

  if ((imu_base_rate < data_rate_hz) || (est_filter_base_rate < data_rate_hz) || (data_rate_hz < 0.0f))
  {
    printf("Invalid data rate %fHz for base rates %u, %u\n", data_rate_hz, imu_base_rate, est_filter_base_rate);
    return false;
  }

  // Configure the IMU data stream format (0x0C, 0x08)
  const uint16_t imu_stream_rate_decimation = static_cast<uint16_t>(imu_base_rate / data_rate_hz);
  if (false == initializeIMUDataStreamFormat(imu_stream_rate_decimation))
  {
    return false;
  }

  // Configure the Estimation Filter data stream format (0x0C, 0x0A)
  const uint16_t est_filter_stream_rate_decimation = static_cast<uint16_t>(est_filter_base_rate / data_rate_hz);
  if (false == initializeEstFilterDataStreamFormat(est_filter_stream_rate_decimation))
  {
    return false;
  }

  const float actual_imu_rate = (float)imu_base_rate / (float)imu_stream_rate_decimation;
  const float actual_est_filter_rate = (float)est_filter_base_rate / (float)est_filter_stream_rate_decimation;
  data_stream_rate_hz = std::min(actual_imu_rate, actual_est_filter_rate);

  return true;
}

bool LORDAHRSDriver::setIdleMode(bool idle)
{
  LORDMIPFieldDescriptor_t field_desc = (idle == true)?
    LORD_MIP_FIELD_DESC_IDLE_MODE : LORD_MIP_FIELD_DESC_RESUME;
  LORDMIPField idle_field(field_desc);

  LORDMIPPkt update_idle_pkt(MIP_DESC_SET_BASE_CMD);
  update_idle_pkt.addField(idle_field);
  return transmitMIPPktSynchronous(update_idle_pkt, nullptr);
}

bool LORDAHRSDriver::queryDeviceInfo()
{
  LORDMIPField device_info_field(LORD_MIP_FIELD_DESC_DEV_INFO);
  LORDMIPPkt device_info_query_pkt(MIP_DESC_SET_BASE_CMD);
  device_info_query_pkt.addField(device_info_field);
  std::vector<LORDMIPField> response_fields;
  if (false == transmitMIPPktSynchronous(device_info_query_pkt, &response_fields))
  {
    return false;
  }

  // Now check and parse the response
  if (response_fields.size() != 1)
  {
    printf("Unexpected number of response fields (%lu) for device info query response\n", response_fields.size());
    return false;
  }

  LORDMIPField *resp_field = &(response_fields[0]);

  if (resp_field->field_descriptor != LORD_MIP_FIELD_DESC_DEV_INFO_RESP)
  {
    printf("Unexpected field descriptor (0X%02X) for device info query response\n", resp_field->field_descriptor);
    return false;
  }

  const uint8_t resp_field_length = resp_field->getFieldLength();
  if (resp_field_length != LORD_MIP_FIELD_LENGTH_DEV_INFO)
  {
    printf("Unexpected field length (0X%02X) for the device info query response\n", resp_field_length);
    return false;
  }

  dev_info.fw_version = (resp_field->field_data[0] << 8) + resp_field->field_data[1];
  // The others are all 16 char strings
  dev_info.model_name.clear();
  dev_info.model_num.clear();
  dev_info.serial_num.clear();
  dev_info.options.clear();
  for (int i = 0; i < 16; ++i)
  {
    dev_info.model_name.append(1, resp_field->field_data[2 + i]);
    dev_info.model_num.append(1, resp_field->field_data[2 + 16 + i]);
    dev_info.serial_num.append(1, resp_field->field_data[2 + 32 + i]);
    dev_info.options.append(1, resp_field->field_data[2 + 48 + i]);
  }

  dev_info.print();

  return true;
}

bool LORDAHRSDriver::enableIncDecWorldMagModel(bool enabled)
{
  // Only allow enabling if we already have a valid reference position
  if (true == enabled && false == has_valid_reference_position)
  {
    printf("Refusing to enable World Magnetic Model without a valid ref. position\n");
    return false;
  }

  const uint8_t inc_dec_src = (true == enabled)?
    LORD_MIP_FIELD_INC_DEC_SRC_WORLD_MAG_MODEL : LORD_MIP_FIELD_INC_DEC_SRC_NONE;

  // Do the inclination source packet
  LORDMIPField inclination_src_field(LORD_MIP_FIELD_DESC_INCLINATION_SRC);
  inclination_src_field.pushBack(LORD_MIP_FIELD_FUNCTION_SELECTOR_APPLY_NEW);
  inclination_src_field.pushBack(inc_dec_src);
  LORDMIPPkt inclination_src_pkt(MIP_DESC_SET_EST_FILT_CMD);
  inclination_src_pkt.addField(inclination_src_field);
  // No response besides ACK/NACK expected
  if (false == transmitMIPPktSynchronous(inclination_src_pkt, nullptr))
  {
    return false;
  }

  // Now the declination source packet
  LORDMIPField declination_src_field(LORD_MIP_FIELD_DESC_DECLINATION_SRC);
  declination_src_field.pushBack(LORD_MIP_FIELD_FUNCTION_SELECTOR_APPLY_NEW);
  declination_src_field.pushBack(inc_dec_src);
  LORDMIPPkt declination_src_pkt(MIP_DESC_SET_EST_FILT_CMD);
  declination_src_pkt.addField(declination_src_field);

  // No response besides ACK/NACK expected
  if (false == transmitMIPPktSynchronous(declination_src_pkt, nullptr))
  {
    return false;
  }

  // If we get here, update the member variable
  world_mag_model_enabled = enabled;
  return true;
}

bool LORDAHRSDriver::queryDataBaseRates(uint16_t &imu_base_rate, uint16_t &est_filter_base_rate)
{
  LORDMIPField imu_base_rate_field(LORD_MIP_FIELD_DESC_IMU_BASE_RATE);
  LORDMIPPkt imu_base_rate_query_pkt(MIP_DESC_SET_3DM_CMD);
  imu_base_rate_query_pkt.addField(imu_base_rate_field);

  std::vector<LORDMIPField> imu_response_fields;
  if (false == transmitMIPPktSynchronous(imu_base_rate_query_pkt, &imu_response_fields))
  {
    return false;
  }

  if (imu_response_fields.size() != 1)
  {
    printf("Unexpected number of response fields (%lu) for imu base rate query response\n", imu_response_fields.size());
    return false;
  }
  LORDMIPField *imu_base_rate_resp_field = &(imu_response_fields[0]);

  LORDMIPField est_filter_base_rate_field(LORD_MIP_FIELD_DESC_EST_FILTER_BASE_RATE);
  LORDMIPPkt est_filter_base_rate_query_pkt(MIP_DESC_SET_3DM_CMD);
  est_filter_base_rate_query_pkt.addField(est_filter_base_rate_field);

  std::vector<LORDMIPField> est_filt_response_fields;
  if (false == transmitMIPPktSynchronous(est_filter_base_rate_query_pkt, &est_filt_response_fields))
  {
    return false;
  }

  if (est_filt_response_fields.size() != 1)
  {
    printf("Unexpected number of response fields (%lu) for est filter base rate query response\n", est_filt_response_fields.size());
    return false;
  }
  LORDMIPField *est_filter_base_rate_resp_field = &(est_filt_response_fields[0]);

  if ((imu_base_rate_resp_field->field_descriptor != LORD_MIP_FIELD_DESC_IMU_BASE_RATE_RESP) ||
      (est_filter_base_rate_resp_field->field_descriptor != LORD_MIP_FIELD_DESC_EST_FILTER_BASE_RATE_RESP))
  {
    printf("Unexpected field descriptor(s) in base rates query response\n");
    return false;
  }

  if ((imu_base_rate_resp_field->getFieldLength() != 4) ||
      (est_filter_base_rate_resp_field->getFieldLength() != 4))
  {
    printf("Unexpected field length(s) in base rates query response\n");
    return false;
  }

  imu_base_rate = (imu_base_rate_resp_field->field_data[0] << 8) + imu_base_rate_resp_field->field_data[1];
  est_filter_base_rate = (est_filter_base_rate_resp_field->field_data[0] << 8) + est_filter_base_rate_resp_field->field_data[1];

  // Debugging
  printf("IMU Base Rate: %uHz, Est. Filter Base Rate: %uHz\n", imu_base_rate, est_filter_base_rate);
  return true;
}

bool LORDAHRSDriver::initializeIMUDataStreamFormat(uint16_t imu_rate_decimation)
{
  LORDMIPField imu_settings_field(LORD_MIP_FIELD_DESC_IMU_MSG_FORMAT);
  imu_settings_field.pushBack(LORD_MIP_FIELD_FUNCTION_SELECTOR_APPLY_NEW);
  imu_settings_field.pushBack(IMU_NUM_FIELD_DESCRIPTORS); // Number of descriptors
  imu_settings_field.pushBack(LORD_MIP_FIELD_DESC_IMU_GPS_TIMESTAMP);
  imu_settings_field.pushBack(imu_rate_decimation);
  imu_settings_field.pushBack(LORD_MIP_FIELD_DESC_IMU_STABILIZED_MAG);
  imu_settings_field.pushBack(imu_rate_decimation);
  imu_settings_field.pushBack(LORD_MIP_FIELD_DESC_IMU_DELTA_VELOCITY);
  imu_settings_field.pushBack(imu_rate_decimation);

  LORDMIPPkt imu_data_stream_settings_pkt(MIP_DESC_SET_3DM_CMD);
  imu_data_stream_settings_pkt.addField(imu_settings_field);

  // No expected response besides the ACK/NACK handled in transmitMIPPktSynchronous
  return transmitMIPPktSynchronous(imu_data_stream_settings_pkt, nullptr);
}

bool LORDAHRSDriver::initializeEstFilterDataStreamFormat(uint16_t est_filter_rate_decimation)
{
  LORDMIPField est_filter_settings_field(LORD_MIP_FIELD_DESC_EST_FILTER_MSG_FORMAT);
  est_filter_settings_field.pushBack(LORD_MIP_FIELD_FUNCTION_SELECTOR_APPLY_NEW);
  est_filter_settings_field.pushBack(EST_FILTER_NUM_FIELD_DESCRIPTORS); // Number of descriptors
  est_filter_settings_field.pushBack(LORD_MIP_FIELD_DESC_EST_FILTER_GPS_TIMESTAMP);
  est_filter_settings_field.pushBack(est_filter_rate_decimation);
  est_filter_settings_field.pushBack(LORD_MIP_FIELD_DESC_EST_FILTER_STATUS);
  est_filter_settings_field.pushBack(est_filter_rate_decimation);
  est_filter_settings_field.pushBack(LORD_MIP_FIELD_DESC_EST_FILTER_QUATERNION);
  est_filter_settings_field.pushBack(est_filter_rate_decimation);
  est_filter_settings_field.pushBack(LORD_MIP_FIELD_DESC_EST_FILTER_LIN_ACCEL);
  est_filter_settings_field.pushBack(est_filter_rate_decimation);
  est_filter_settings_field.pushBack(LORD_MIP_FIELD_DESC_EST_FILTER_COMP_ANGULAR_RATE);
  est_filter_settings_field.pushBack(est_filter_rate_decimation);
  est_filter_settings_field.pushBack(LORD_MIP_FIELD_DESC_EST_FILTER_MAG_SOLUTION);
  est_filter_settings_field.pushBack(est_filter_rate_decimation);

  LORDMIPPkt est_filter_stream_settings_pkt(MIP_DESC_SET_3DM_CMD);
  est_filter_stream_settings_pkt.addField(est_filter_settings_field);

  // No expected response besides the ACK/NACK handled in transmitMIPPktSynchronous
  return transmitMIPPktSynchronous(est_filter_stream_settings_pkt, nullptr);
}

bool LORDAHRSDriver::initializeHeadingUpdateSource()
{
  LORDMIPField heading_update_src_field(LORD_MIP_FIELD_DESC_HEADING_UPDATE_CONTROL);
  heading_update_src_field.pushBack(LORD_MIP_FIELD_FUNCTION_SELECTOR_APPLY_NEW);
  heading_update_src_field.pushBack(LORD_MIP_FIELD_HEADING_CONTROL_INTERNAL);
  LORDMIPPkt heading_update_ctl_pkt(MIP_DESC_SET_EST_FILT_CMD);
  heading_update_ctl_pkt.addField(heading_update_src_field);

  // No expected response besides the ACK/NACK handled in transmitMIPPktSynchronous
  return transmitMIPPktSynchronous(heading_update_ctl_pkt, nullptr);
}

bool LORDAHRSDriver::initializeSensorToVehicleTransform(const AHRSRollPitchYaw &transform)
{
  LORDMIPField sensor_vehicle_transform_field(LORD_MIP_FIELD_DESC_SENSOR_VEHICLE_TRANSFORM);
  sensor_vehicle_transform_field.pushBack(LORD_MIP_FIELD_FUNCTION_SELECTOR_APPLY_NEW);
  sensor_vehicle_transform_field.pushBack(transform.roll_rad);
  sensor_vehicle_transform_field.pushBack(transform.pitch_rad);
  sensor_vehicle_transform_field.pushBack(transform.yaw_rad);
  LORDMIPPkt sensor_vehicle_transform_pkt(MIP_DESC_SET_EST_FILT_CMD);
  sensor_vehicle_transform_pkt.addField(sensor_vehicle_transform_field);

  // No expected response besides the ACK/NACK handled in transmitMIPPktSynchronous
  return transmitMIPPktSynchronous(sensor_vehicle_transform_pkt, nullptr);
}

bool LORDAHRSDriver::setEstFilterControlFlags()
{
  // Empirically determined using MIP Monitor GUI application
  static constexpr uint16_t LORD_MIP_FIELD_EST_CONTROL_FLAG_VAL =
    LORD_MIP_FIELD_EST_CONTROL_BASE_UNKNOWN_DEFAULT | LORD_MIP_FIELD_EST_CONTROL_FLAG_ENABLE_GYRO_BIAS |
    LORD_MIP_FIELD_EST_CONTROL_FLAG_ENABLE_HARD_IRON_AUTOCAL | LORD_MIP_FIELD_EST_CONTROL_FLAG_ENABLE_SOFT_IRON_AUTOCAL;

  LORDMIPField est_control_flag_field(LORD_MIP_FIELD_DESC_EST_CONTROL_FLAGS);
  est_control_flag_field.pushBack(LORD_MIP_FIELD_FUNCTION_SELECTOR_APPLY_NEW);
  est_control_flag_field.pushBack(LORD_MIP_FIELD_EST_CONTROL_FLAG_VAL);
  LORDMIPPkt est_control_flag_pkt(MIP_DESC_SET_EST_FILT_CMD);
  est_control_flag_pkt.addField(est_control_flag_field);

  // No expected response besides the ACK/NACK handled in transmitMIPPktSynchronous
  return transmitMIPPktSynchronous(est_control_flag_pkt, nullptr);
}

bool LORDAHRSDriver::enableGravityVectorAiding()
{
  LORDMIPField pitch_roll_aiding_ctl_field(LORD_MIP_FIELD_DESC_PITCH_ROLL_AIDING_CONTROL);
  pitch_roll_aiding_ctl_field.pushBack(LORD_MIP_FIELD_FUNCTION_SELECTOR_APPLY_NEW);
  pitch_roll_aiding_ctl_field.pushBack(LORD_MIP_FIELD_ENABLE_GRAV_VECT_AIDING);
  LORDMIPPkt pitch_roll_aiding_pkt(MIP_DESC_SET_EST_FILT_CMD);
  pitch_roll_aiding_pkt.addField(pitch_roll_aiding_ctl_field);

  // No expected response besides the ACK/NACK handled in transmitMIPPktSynchronous
  return transmitMIPPktSynchronous(pitch_roll_aiding_pkt, nullptr);
}

bool LORDAHRSDriver::enableConfiguredDataStreams()
{
  LORDMIPField imu_stream_enable_field(LORD_MIP_FIELD_DESC_ENABLE_DATA_STREAM);
  imu_stream_enable_field.pushBack(LORD_MIP_FIELD_FUNCTION_SELECTOR_APPLY_NEW);
  imu_stream_enable_field.pushBack(LORD_MIP_FIELD_DEVICE_SELECTOR_IMU);
  imu_stream_enable_field.pushBack(LORD_MIP_FIELD_GENERIC_ENABLE);
  LORDMIPPkt imu_stream_enable_pkt(MIP_DESC_SET_3DM_CMD);
  imu_stream_enable_pkt.addField(imu_stream_enable_field);
  if (false == transmitMIPPktSynchronous(imu_stream_enable_pkt, nullptr))
  {
    return false;
  }

  LORDMIPField est_filter_stream_enable_field(LORD_MIP_FIELD_DESC_ENABLE_DATA_STREAM);
  est_filter_stream_enable_field.pushBack(LORD_MIP_FIELD_FUNCTION_SELECTOR_APPLY_NEW);
  est_filter_stream_enable_field.pushBack(LORD_MIP_FIELD_DEVICE_SELECTOR_EST_FILTER);
  est_filter_stream_enable_field.pushBack(LORD_MIP_FIELD_GENERIC_ENABLE);
  LORDMIPPkt est_filter_stream_enable_pkt(MIP_DESC_SET_3DM_CMD);
  est_filter_stream_enable_pkt.addField(est_filter_stream_enable_field);

  return transmitMIPPktSynchronous(est_filter_stream_enable_pkt, nullptr);
}

bool LORDAHRSDriver::enableKalmanFilterAutoInitialization()
{
  LORDMIPField kfilt_auto_init_field(LORD_MIP_FIELD_DESC_AUTO_INIT_CTL);
  kfilt_auto_init_field.pushBack(LORD_MIP_FIELD_FUNCTION_SELECTOR_APPLY_NEW);
  kfilt_auto_init_field.pushBack(LORD_MIP_FIELD_GENERIC_ENABLE);
  LORDMIPPkt kfilt_auto_init_pkt(MIP_DESC_SET_EST_FILT_CMD);
  kfilt_auto_init_pkt.addField(kfilt_auto_init_field);

  return transmitMIPPktSynchronous(kfilt_auto_init_pkt, nullptr);
}

bool LORDAHRSDriver::transmitMIPPktSynchronous(const LORDMIPPkt &pkt, std::vector<LORDMIPField> *resp_fields, uint32_t rx_timeout_ms)
{
  // Check if this kind of packet is supported by this function
  const size_t tx_pkt_field_count = pkt.fields.size();
  if (tx_pkt_field_count != 1)
  {
    printf("Packets with %lu fields not supported for transmit\n", tx_pkt_field_count);
    return false;
  }
  // Serialize the packet into a byte stream
  std::vector<uint8_t> pkt_bytes;
  pkt.serialize(pkt_bytes);

  // Write the byte stream to serial
  size_t bytes_written = 0;
  const size_t pkt_byte_count = pkt_bytes.size();

  // Debugging
  /*
  unsigned char debug_buffer[8] = {0x75, 0x65, 0x01, 0x02, 0x02, 0x02, 0xE1, 0xC7}; // Set IDLE
  write(serial_fd, debug_buffer, 8);
  */

  while (bytes_written < pkt_byte_count)
  {
    const size_t bytes_to_write = pkt_byte_count - bytes_written;
    const uint8_t *buf_ptr = pkt_bytes.data() + bytes_written;
    const ssize_t bytes_written_now = write(serial_fd, buf_ptr, bytes_to_write);

    #ifdef PRINT_TX_BYTES
      printf("TX: ");
      for (int i = 0; i < bytes_written_now; ++i)
      {
        printf("%02x ", *(pkt_bytes.data() + bytes_written + i));
      }
    #endif

    if (-1 == bytes_written_now)
    {
      printf("Failed to write pkt to serial (%s)\n", strerror(errno));
      return false;
    }
    bytes_written += bytes_written_now;
  }
  #ifdef PRINT_TX_BYTES
    //Debugging
    printf("\n\n");
  #endif

  bool still_waiting_for_rx = true;
  std::chrono::time_point<std::chrono::steady_clock> timeout_time = std::chrono::steady_clock::now() +
    std::chrono::duration<uint32_t, std::milli>(rx_timeout_ms);

  // Wait for the ACK/NACK response
  LORDMIPPkt resp_pkt(MIP_DESC_SET_INVALID);
  while (true == still_waiting_for_rx)
  {
    const std::chrono::time_point<std::chrono::steady_clock> curr_time = std::chrono::steady_clock::now();
    if (curr_time >= timeout_time)
    {
      printf("Timeout while waiting for a response packet for descriptor set 0x%02X\n", pkt.descriptor);
      return false;
    }

    const std::chrono::milliseconds remaining_timeout_ms = std::chrono::duration_cast<std::chrono::milliseconds>(timeout_time - curr_time);
    if (false == rcvPktBlocking(resp_pkt, remaining_timeout_ms))
    {
      return false;
    }
    // See if this packet contains the expected ACK/NACK for the field in the request packet
    if (resp_pkt.descriptor == pkt.descriptor)
    {
      if (resp_pkt.fields.size() < 1)
      {
        printf("Ill-formed receive packet with no fields... discarding\n");
        continue;
      }
      const LORDMIPField &req_field = pkt.fields[0];
      const LORDMIPField &resp_field = resp_pkt.fields[0];
      // Check that this is an ACK/NACK field -- it must be
      if (LORD_MIP_FIELD_DESC_ACK_RESP == resp_field.field_descriptor)
      {
        // Make sure this ACK/NACK field is well-formed
        if (LORD_MIP_FIELD_LENGTH_ACK != resp_field.getFieldLength())
        {
          printf("Invalid field length for the ACK field... discarding packet\n");
          continue; // Just throw away the packet
        }

        // Check if this ACK/NACK matches the command
        if (resp_field.field_data[0] != req_field.field_descriptor)
        {
          // This response pkt did not match -- it is the ACK for some other
          // (apparently non-synchronous) request
          printf("Non-matching echo in an ACK field... discarding packet\n");
          continue; // Just throw it away
        }

        // This is our ACK/NACK -- check for NACK
        if (LORD_MIP_ACK_FIELD_NO_ERROR_VAL != resp_field.field_data[1])
        {
          printf("Received error code (%s) for command [0x%02X,0x%02X]\n",
                  mipErrorCodeToString(resp_field.field_data[1]), pkt.descriptor,
                  req_field.field_descriptor);
          return false;
        }
        else // No error
        {
          // Set the exit condition
          still_waiting_for_rx = false;
          continue;
        }
      }
      else // First field of the response was not an ack -- must be data... just ignore it
      {
        //deferred_rx_packets.push(resp_pkt);
        continue; // Keep searching for our response
      }
    }
    else // This response packet wasn't from our command class... just ignore it
    {
      //deferred_rx_packets.push(resp_pkt);
      continue; // Keep searching for our response
    }
  }

  // If we get here, we got a successful ACK response.
  // Copy the non-ACK/NACK response fields to the output vector
  if (nullptr != resp_fields)
  {
    resp_fields->clear();
    // Assume the first "tx_pkt_field_count" packets are ACKs, which we don't
    // return in the output vector
    for (auto it = resp_pkt.fields.begin() + tx_pkt_field_count; it < resp_pkt.fields.end(); ++it)
    {
      resp_fields->push_back(*it);
    }
  }

  return true;
}

struct RcvPktParsingState
{
  bool last_byte_was_sync_1 = false;
  bool first_sync_found = false;
  bool found_pkt_descriptor = false;
  bool found_payload_length = false;
  uint8_t expected_payload_length = 0;
  uint8_t rcvd_payload_length = 0;
  uint8_t expected_current_field_length = 0;
  uint8_t rcvd_current_field_length = 0;
  bool found_current_field_descriptor = false;
  uint8_t computed_chksum_byte_1 = 0;
  uint8_t computed_chksum_byte_2 = 0;

  void reset()
  {
    last_byte_was_sync_1 = false;
    first_sync_found = false;
    found_pkt_descriptor = false;
    found_payload_length = false;
    expected_payload_length = 0;
    rcvd_payload_length = 0;
    expected_current_field_length = 0;
    rcvd_current_field_length = 0;
    found_current_field_descriptor = false;
    computed_chksum_byte_1 = 0;
    computed_chksum_byte_2 = 0;
  }

  void setInitialSyncFoundState()
  {
    reset();
    first_sync_found = true;

    // Initialize the checksum as it was reset above
    updateComputedChksumForByte(LORD_MIP_PKT_SYNC1);
    updateComputedChksumForByte(LORD_MIP_PKT_SYNC2);
  }

  void resetCurrentFieldState()
  {
    expected_current_field_length = 0;
    rcvd_current_field_length = 0;
    found_current_field_descriptor = false;
  }

  void updateComputedChksumForByte(uint8_t byte)
  {
    computed_chksum_byte_1 += byte;
    computed_chksum_byte_2 += computed_chksum_byte_1;
  }
};

bool LORDAHRSDriver::rcvPktBlocking(LORDMIPPkt &pkt_out, const std::chrono::milliseconds &timeout_ms)
{
  RcvPktParsingState parsing_state;
  const std::chrono::time_point<std::chrono::steady_clock> function_start_time = std::chrono::steady_clock::now();
  LORDMIPField field_in_progress(LORD_MIP_FIELD_DESC_INVALID);
  pkt_out.reset();

  #ifdef PRINT_RX_BYTES
    printf("RX: ");
  #endif

  while (true) // We exit this function from directly within the loop
  {
    // First, check if a timeout has occurred
    const std::chrono::time_point<std::chrono::steady_clock> curr_time = std::chrono::steady_clock::now();
    const std::chrono::milliseconds elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(curr_time - function_start_time);
    if (elapsed >= timeout_ms)
    {
      // Timeout expired -- this is the only error for which we exit the function,
      // all other errors simply start looking for the next packet
      printf("Timed out while receiving a packet\n");
      return false;
    }

    // This sleep seems to be necessary, though I don't know why
    usleep(1);

    uint8_t byte;
    const ssize_t bytes_read = read(serial_fd, &byte, 1);
    if (-1 == bytes_read)
    {
      if (errno != EAGAIN)
      {
        printf("Failed to read from serial port (%s)\n", strerror(errno));
      }
      continue;
    }

    #ifdef PRINT_RX_BYTES
      printf("%02X ", byte);
    #endif

    // Check if this is a packet sync byte 1
    if (LORD_MIP_PKT_SYNC1 == byte)
    {
      parsing_state.last_byte_was_sync_1 = true;
    }
    // Otherwise, check if this is the second byte in a sync byte sequence
    else if (parsing_state.last_byte_was_sync_1 == true)
    {
      if (LORD_MIP_PKT_SYNC2 == byte)
      {
        // Reset everything else, so that if this was a sync sequence in the middle of a packet, we
        // simply start over
        parsing_state.setInitialSyncFoundState();
        continue;
      }
      // Otherwise, this isn't the sync set
      parsing_state.last_byte_was_sync_1 = false;
    }

    // If we haven't yet found the sync, just discard this byte and continue searching for sync
    if (false == parsing_state.first_sync_found)
    {
      continue;
    }

    // Check for packet descriptor
    if (false == parsing_state.found_pkt_descriptor)
    {
      // This byte must be the packet descriptor, but check that it is a valid value
      if (false == LORDMIPPkt::isValidPktDescriptor(byte))
      {
        printf("Received invalid packet descriptor (0x%02x)\n", byte);
        parsing_state.reset();
        pkt_out.reset();
        continue;
      }
      parsing_state.found_pkt_descriptor = true;
      pkt_out.descriptor = byte;
      parsing_state.updateComputedChksumForByte(byte);
    }
    else if (false == parsing_state.found_payload_length) // Check for payload length
    {
      // No bounds checking
      parsing_state.found_payload_length = true;
      parsing_state.expected_payload_length = byte;
      parsing_state.updateComputedChksumForByte(byte);
    }
    else if (parsing_state.rcvd_payload_length < parsing_state.expected_payload_length)
    {
      // All these bytes are checksum'd
      parsing_state.updateComputedChksumForByte(byte);

      // We are building a field
      if (0 == parsing_state.expected_current_field_length)
      {
        // Starting a new field, so reset it to ensure data array is cleared... could
        // still have bytes from an aborted partial field from before
        field_in_progress.reset();
        // Validate this as a field length: fields can't be longer than total payload
        // length for the packet or shorter than 2 bytes (field length and field descriptor)
        const uint8_t max_field_length = parsing_state.expected_payload_length - parsing_state.rcvd_payload_length;
        if (byte > max_field_length || byte < 2)
        {
          printf("Received an invalid field length (%u)\n", byte);
          parsing_state.reset();
          pkt_out.reset();
          continue;
        }
        // Otherwise we're okay
        parsing_state.expected_current_field_length = byte;
        ++parsing_state.rcvd_current_field_length;
      }
      else if (false == parsing_state.found_current_field_descriptor)
      {
        if (false == LORDMIPField::isValidResponseFieldDescriptor(pkt_out.descriptor, byte))
        {
          printf("Received an invalid descriptor set (0x%02X, 0x%02X)\n", pkt_out.descriptor, byte);
          parsing_state.reset();
          pkt_out.reset();
          continue;
        }
        field_in_progress.field_descriptor = byte;
        parsing_state.found_current_field_descriptor = true;
        ++parsing_state.rcvd_current_field_length;
      }
      else if (parsing_state.rcvd_current_field_length < parsing_state.expected_current_field_length)
      {
        field_in_progress.field_data.push_back(byte);
        ++parsing_state.rcvd_current_field_length;
      }

      // Now check if we've completed a field
      if ((0 != parsing_state.expected_current_field_length) &&
          (parsing_state.rcvd_current_field_length == parsing_state.expected_current_field_length))
      {
        pkt_out.fields.push_back(field_in_progress);
        parsing_state.resetCurrentFieldState();
        field_in_progress.reset();
      }

      ++parsing_state.rcvd_payload_length;
    }
    else if (parsing_state.rcvd_payload_length == parsing_state.expected_payload_length)
    {
      if (parsing_state.computed_chksum_byte_1 != byte)
      {
        printf("Invalid checksum byte 1 (0x%02X != 0x%02X) in received packet\n", byte, parsing_state.computed_chksum_byte_1);
        parsing_state.reset();
        pkt_out.reset();
        continue;
      }
      ++parsing_state.rcvd_payload_length; // Ensures we'll treat the next bytes as checksum 2
    }
    else // last byte of the packet
    {
      if (parsing_state.computed_chksum_byte_2 != byte)
      {
        printf("Invalid checksum byte 2 (0x%02X != 0x%02X) in received packet\n", byte, parsing_state.computed_chksum_byte_2);
        parsing_state.reset();
        pkt_out.reset();
        continue;
      }

      #ifdef PRINT_RX_BYTES
        printf("\n\n");
      #endif
      // If and only if we get here, a packet was successfully received
      // The pkt_out is already completely assigned, so just return
      return true;
    }
  } // while(true)
}

bool LORDAHRSDriver::parseDataPkts(const LORDMIPPkt &imu_pkt, const LORDMIPPkt &est_filt_pkt, AHRSDataSet &data_out)
{
  LORDAHRSIMUData imu_data;
  LORDAHRSEstFiltData est_filt_data;
  if ((false == parseIMUDataPkt(imu_pkt, imu_data)) ||
      (false == parseEstFilterDataPkt(est_filt_pkt, est_filt_data)))
  {
    return false;
  }

  // Now merge the individually parsed data sets into the output format
  mergeAHRSData(imu_data, est_filt_data, data_out);

  // Debugging
  //imu_data.print();
  //est_filt_data.print();
  //data_out.print();

  return true;
}

bool LORDAHRSDriver::parseIMUDataPkt(const LORDMIPPkt &imu_pkt, LORDAHRSIMUData &imu_data_out)
{
  // Validate the input packet
  if ((imu_pkt.descriptor != MIP_DESC_SET_IMU_DATA) ||
      (imu_pkt.fields.size() != IMU_NUM_FIELD_DESCRIPTORS))
  {
    printf("Failed to parse ill-formed IMU data packet\n");
    return false;
  }

  bool timestamp_parsed = false;
  bool mag_parsed = false;
  bool delta_v_parsed = false;
  for (const auto &field : imu_pkt.fields)
  {
    switch (field.field_descriptor)
    {
      case LORD_MIP_FIELD_DESC_IMU_GPS_TIMESTAMP:
        timestamp_parsed = parseIMUGPSTimestampField(field, imu_data_out.gps_timestamp);
        break;
      case LORD_MIP_FIELD_DESC_IMU_STABILIZED_MAG:
        mag_parsed = parseIMUStabilizedMagField(field, imu_data_out.stabilized_mag);
        break;
      case LORD_MIP_FIELD_DESC_IMU_DELTA_VELOCITY:
        delta_v_parsed = parseIMUDeltaVelocity(field, imu_data_out.delta_v_vector);
        break;
      default:
        printf("Failed to parse unexpected IMU data packet field (0x%02X)\n", field.field_descriptor);
        return false;
    }
  }
  if ((false == timestamp_parsed) || (false == mag_parsed) || (false == delta_v_parsed))
  {
    printf("Failed to parse an expected field from the IMU Data\n");
    return false;
  }

  return true;
}

bool LORDAHRSDriver::parseEstFilterDataPkt(const LORDMIPPkt &est_filt_pkt, LORDAHRSEstFiltData &est_filt_data_out)
{
  // Validate the input packet
  if ((est_filt_pkt.descriptor != MIP_DESC_SET_EST_FILT_DATA) ||
      (est_filt_pkt.fields.size() != EST_FILTER_NUM_FIELD_DESCRIPTORS))
  {
    printf("Failed to parse ill-formed Estimation Filter data packet\n");
    return false;
  }

  bool timestamp_parsed = false;
  bool status_parsed = false;
  bool quaternion_parsed = false;
  bool lin_accel_parsed = false;
  bool angular_rate_parsed = false;
  bool mag_solution_parsed = false;

  for (const auto &field : est_filt_pkt.fields)
  {
    switch (field.field_descriptor)
    {
      case LORD_MIP_FIELD_DESC_EST_FILTER_GPS_TIMESTAMP:
        timestamp_parsed = parseEstFilterGPSTimestamp(field, est_filt_data_out.gps_timestamp);
        break;
      case LORD_MIP_FIELD_DESC_EST_FILTER_STATUS:
        status_parsed = parseEstFilterStatus(field, est_filt_data_out.filt_status);
        break;
      case LORD_MIP_FIELD_DESC_EST_FILTER_QUATERNION:
        quaternion_parsed = parseEstFilterQuaternion(field, est_filt_data_out.quaternion);
        break;
      case LORD_MIP_FIELD_DESC_EST_FILTER_LIN_ACCEL:
        lin_accel_parsed = parseEstFilterLinAccel(field, est_filt_data_out.lin_accel);
        break;
      case LORD_MIP_FIELD_DESC_EST_FILTER_COMP_ANGULAR_RATE:
        angular_rate_parsed = parseEstFilterAngularRate(field, est_filt_data_out.angular_rate);
        break;
      case LORD_MIP_FIELD_DESC_EST_FILTER_MAG_SOLUTION:
        mag_solution_parsed = parseEstFilterMagSolution(field, est_filt_data_out.mag_solution);
        break;
      default:
        printf("Failed to parse unexpected Est. Filter data packet field (0x%02X)\n", field.field_descriptor);
        return false;
    }
  }

  if (!timestamp_parsed || !status_parsed || !quaternion_parsed || !lin_accel_parsed || !angular_rate_parsed || !mag_solution_parsed)
  {
    printf("Failed to parse an expected field from Estimation Filter data packet\n");
    return false;
  }
  return true;
}

bool LORDAHRSDriver::parseIMUGPSTimestampField(const LORDMIPField &field, LORDAHRSIMUGPSTimestampData &data_out)
{
  if (false == field.validate(LORD_MIP_FIELD_DESC_IMU_GPS_TIMESTAMP, LORD_MIP_FIELD_DATA_LENGTH_IMU_GPS_TIMESTAMP))
  {
    return false;
  }

  uint16_t flags = 0;
  if ((false == field.extractVal(data_out.gps_tow, 0)) ||
      (false == field.extractVal(data_out.gps_week_number, 8)) ||
      (false == field.extractVal(flags, 10)))
  {
    printf("Unable to parse a GPS Timestamp field\n");
    return false;
  }

  // Strip the values from the flags bitfield
  data_out.pps_present = flags & 0x1;
  data_out.gps_time_refresh_toggle = (flags & 0x2) >> 1;
  data_out.gps_time_initialized = flags & 0x4;

  return true;
}

bool LORDAHRSDriver::parseIMUStabilizedMagField(const LORDMIPField &field, LORDAHRSIMUStabilizedMagData &data_out)
{
  if (false == field.validate(LORD_MIP_FIELD_DESC_IMU_STABILIZED_MAG, LORD_MIP_FIELD_DATA_LENGTH_IMU_STABILIZED_MAG))
  {
    return false;
  }

  if ((false == field.extractVal(data_out.x, 0)) ||
      (false == field.extractVal(data_out.y, 4)) ||
      (false == field.extractVal(data_out.z, 8)))
  {
    printf("Unable to parse a Stabilized Mag field\n");
    return false;
  }

  return true;
}

bool LORDAHRSDriver::parseIMUDeltaVelocity(const LORDMIPField &field,  LORDAHRSIMUDeltaVelocity &data_out)
{
  if (false == field.validate(LORD_MIP_FIELD_DESC_IMU_DELTA_VELOCITY, LORD_MIP_FIELD_DATA_LENGTH_IMU_DELTA_VELOCITY))
  {
    return false;
  }

  if ((false == field.extractVal(data_out.x, 0)) ||
      (false == field.extractVal(data_out.y, 4)) ||
      (false == field.extractVal(data_out.z, 8)))
  {
    printf("Unable to parse a Delta Velocity field\n");
    return false;
  }

  // Convert units
  data_out.x *= LORDAHRSIMUDeltaVelocity::G_SEC_TO_MPS;
  data_out.y *= LORDAHRSIMUDeltaVelocity::G_SEC_TO_MPS;
  data_out.z *= LORDAHRSIMUDeltaVelocity::G_SEC_TO_MPS;

  return true;
}

bool LORDAHRSDriver::parseEstFilterGPSTimestamp(const LORDMIPField &field, LORDAHRSEstFiltGPSTimestampData &data_out)
{
  if (false == field.validate(LORD_MIP_FIELD_DESC_EST_FILTER_GPS_TIMESTAMP, LORD_MIP_FIELD_DATA_LENGTH_EST_FILTER_GPS_TIMESTAMP))
  {
    return false;
  }

  uint16_t flags;
  if ((false == field.extractVal(data_out.gps_tow, 0)) ||
      (false == field.extractVal(data_out.gps_week_number, 8)) ||
      (false == field.extractVal(flags, 10)))
  {
    printf("Unable to parse an Est. Filter GPS Timestamp field\n");
    return false;
  }
  data_out.valid = (flags & LORD_MIP_FIELD_GENERIC_VALID_FLAG);

  return true;
}

bool LORDAHRSDriver::parseEstFilterStatus(const LORDMIPField &field, LORDAHRSEstFiltStatus &data_out)
{
  if (false == field.validate(LORD_MIP_FIELD_DESC_EST_FILTER_STATUS, LORD_MIP_FIELD_DATA_LENGTH_EST_FILTER_STATUS))
  {
    return false;
  }

  if ((false == field.extractVal(data_out.state, 0)) ||
      (false == field.extractVal(data_out.dynamics_mode, 2)) ||
      (false == field.extractVal(data_out.flags, 4)))
  {
    printf("Unable to parse an Est. Filter Status field\n");
    return false;
  }

  return true;
}

bool LORDAHRSDriver::parseEstFilterQuaternion(const LORDMIPField &field, LORDAHRSEstFiltQuaternion &data_out)
{
  if (false == field.validate(LORD_MIP_FIELD_DESC_EST_FILTER_QUATERNION, LORD_MIP_FIELD_DATA_LENGTH_EST_FILTER_QUATERNION))
  {
    return false;
  }

  uint16_t flags;
  if ((false == field.extractVal(data_out.q0, 0)) ||
      (false == field.extractVal(data_out.q1_i, 4)) ||
      (false == field.extractVal(data_out.q2_j, 8)) ||
      (false == field.extractVal(data_out.q3_k, 12)) ||
      (false == field.extractVal(flags, 16)))
  {
    printf("Unable to parse an Est. Filter Quaternion field\n");
    return false;
  }
  data_out.valid = (flags & LORD_MIP_FIELD_GENERIC_VALID_FLAG);

  return true;
}

bool LORDAHRSDriver::parseEstFilterLinAccel(const LORDMIPField &field, LORDAHRSEstFiltLinearAccel &data_out)
{
  if (false == field.validate(LORD_MIP_FIELD_DESC_EST_FILTER_LIN_ACCEL, LORD_MIP_FIELD_DATA_LENGTH_EST_FILTER_LIN_ACCEL))
  {
    return false;
  }

  uint16_t flags;
  if ((false == field.extractVal(data_out.x, 0)) ||
      (false == field.extractVal(data_out.y, 4)) ||
      (false == field.extractVal(data_out.z, 8)) ||
      (false == field.extractVal(flags, 12)))
  {
    printf("Unable to parse a Linear Acceleration field\n");
    return false;
  }
  data_out.valid = (flags & 0x0001);

  return true;
}

bool LORDAHRSDriver::parseEstFilterAngularRate(const LORDMIPField &field, LORDAHRSEstFiltAngularRate &data_out)
{
  if (false == field.validate(LORD_MIP_FIELD_DESC_EST_FILTER_COMP_ANGULAR_RATE, LORD_MIP_FIELD_DATA_LENGTH_EST_FILTER_COMP_ANGULAR_RATE))
  {
    return false;
  }

  uint16_t flags;
  if ((false == field.extractVal(data_out.x, 0)) ||
      (false == field.extractVal(data_out.y, 4)) ||
      (false == field.extractVal(data_out.z, 8)) ||
      (false == field.extractVal(flags, 12)))
  {
    printf("Unable to parse an Angular Rate field\n");
    return false;
  }
  data_out.valid = (flags & LORD_MIP_FIELD_GENERIC_VALID_FLAG);

  return true;
}

bool LORDAHRSDriver::parseEstFilterMagSolution(const LORDMIPField &field, LORDAHRSEstFiltMagSolution &data_out)
{
  if (false == field.validate(LORD_MIP_FIELD_DESC_EST_FILTER_MAG_SOLUTION, LORD_MIP_FIELD_DATA_LENGTH_EST_FILTER_MAG_SOLUTION))
  {
    return false;
  }

  uint16_t flags;
  if ((false == field.extractVal(data_out.north_g, 0)) ||
      (false == field.extractVal(data_out.east_g, 4)) ||
      (false == field.extractVal(data_out.down_g, 8)) ||
      (false == field.extractVal(data_out.inclination_rad, 12)) ||
      (false == field.extractVal(data_out.declination_rad, 16)) ||
      (false == field.extractVal(flags, 20)))
  {
    printf("Unable to parse a Magnetic Solution field\n");
    return false;
  }
  data_out.valid = (flags & LORD_MIP_FIELD_GENERIC_VALID_FLAG);

  return true;
}

static AHRSFilterStatus lordAHRSFilterStateToGenericState(uint16_t lordEstFilterState)
{
  switch (lordEstFilterState)
  {
    case LORDAHRSEstFiltStatus::STATE_STARTUP:
      return AHRS_FILTER_STAT_STARTUP;
    case LORDAHRSEstFiltStatus::STATE_INIT:
      return AHRS_FILTER_STAT_INIT;
    case LORDAHRSEstFiltStatus::STATE_RUNNING_VALID:
      return AHRS_FILTER_STAT_RUN_VAL;
    case LORDAHRSEstFiltStatus::STATE_RUNNING_ERROR:
    default:
      return AHRS_FILTER_STAT_RUN_ERR;
  }
}

void LORDAHRSDriver::mergeAHRSData(const LORDAHRSIMUData &imu_data_in, const LORDAHRSEstFiltData &est_filt_data_in,
                                   AHRSDataSet &merged_out)
{
  // Timestamp -- just use the one from the est filter data set
  gpsToPosixTime(est_filt_data_in.gps_timestamp.gps_tow, imu_data_in.gps_timestamp.gps_week_number, merged_out.timestamp);

  // Filter State
  merged_out.filter_state = lordAHRSFilterStateToGenericState(est_filt_data_in.filt_status.state);
  merged_out.filter_flags = est_filt_data_in.filt_status.flags;

  // Linear acceleration
  merged_out.accel_x = est_filt_data_in.lin_accel.x;
  merged_out.accel_y = est_filt_data_in.lin_accel.y;
  merged_out.accel_z = est_filt_data_in.lin_accel.z;
  merged_out.accel_valid = est_filt_data_in.lin_accel.valid;

  // Linear velocity
  // Must keep a running tally, because we only get integrated acceleration, not true velocity of course
  // TODO: This is very fragile and relies on this function getting called exactly once for every IMU packet that
  // the AHRS produces -- hard to control.
  /*
  static float accumulated_velocity_x = 0.0f;
  accumulated_velocity_x += imu_data_in.delta_v_vector.x;
  merged_out.velocity_x = accumulated_velocity_x;
  static float accumulated_velocity_y = 0.0f;
  accumulated_velocity_y += imu_data_in.delta_v_vector.y;
  merged_out.velocity_y = accumulated_velocity_y;
  static float accumulated_velocity_z = 0.0f;
  accumulated_velocity_z += imu_data_in.delta_v_vector.z;
  merged_out.velocity_z = accumulated_velocity_z;
  */

  // Angular velocity
  merged_out.angular_velocity_x = est_filt_data_in.angular_rate.x;
  merged_out.angular_velocity_y = est_filt_data_in.angular_rate.y;
  merged_out.angular_velocity_z = est_filt_data_in.angular_rate.z;
  merged_out.angular_velocity_valid = est_filt_data_in.angular_rate.valid;

  // Orientation
  merged_out.orientation_q0 = est_filt_data_in.quaternion.q0;
  merged_out.orientation_q1_i = est_filt_data_in.quaternion.q1_i;
  merged_out.orientation_q2_j = est_filt_data_in.quaternion.q2_j;
  merged_out.orientation_q3_k = est_filt_data_in.quaternion.q3_k;
  merged_out.orientation_valid = est_filt_data_in.quaternion.valid;

  // Heading
  if (true == heading_override)
  {
    merged_out.heading = heading_override_deg;
    merged_out.heading_true_north = heading_override_true_north;
    merged_out.heading_valid = true;
  }
  else
  {
    // Convert to heading degrees true North via atan2, a declination adjustment,a unit conversion, and a range transformation!
    static constexpr float RAD_TO_DEG = 57.2957795f;
    merged_out.heading_true_north = (est_filt_data_in.mag_solution.valid && world_mag_model_enabled);
    const float declination_correction = (true == merged_out.heading_true_north)?
      est_filt_data_in.mag_solution.declination_rad : 0.0f;
    merged_out.heading = RAD_TO_DEG *
                        (-atan2(imu_data_in.stabilized_mag.y, imu_data_in.stabilized_mag.x) -
                         declination_correction);
    if (merged_out.heading < 0.0) // Want non-negative heading
    {
      merged_out.heading += 360.0f;
    }
    merged_out.heading_valid = est_filt_data_in.mag_solution.valid;
  }
}
} //namespace Numurus
