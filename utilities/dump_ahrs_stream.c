#include <stdio.h>
#include <stdint.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#define MIP_DESCRIPTOR_UNINIT   0x0
#define MIP_PAYLOAD_LENGTH_UNINIT   0x0

typedef struct MIPPkt
{
  uint8_t descriptor;
  uint8_t payload_length;

  uint8_t payload_bytes[256];
  uint8_t payload_bytes_rcvd;

  uint8_t chksum_msb;
  uint8_t chksum_lsb;
  uint8_t chksum_bytes_rcvd;
} MIPPkt_t;

void initMIPPkt(MIPPkt_t *pkt)
{
  pkt->descriptor = MIP_DESCRIPTOR_UNINIT;
  pkt->payload_length = MIP_PAYLOAD_LENGTH_UNINIT;
  pkt->payload_bytes_rcvd = 0;
  pkt->chksum_bytes_rcvd = 0;
}

void processMIPPkt(MIPPkt_t *pkt)
{
  static uint32_t pkt_cnt = 0;
  ++pkt_cnt;

  printf("Pkt %u\n", pkt_cnt);
  printf("\tDescriptor: %02X\n", pkt->descriptor);
  printf("\tPayload Length: %u\n", pkt->payload_length);
  printf("\tPayload:\n\t\t");
  for (uint8_t i = 0; i < pkt->payload_length; ++i)
  {
    printf("%02X ", pkt->payload_bytes[i]);
    if (i % 8 == 7)
    {
      printf("\n\t\t");
    }
  }
  printf("\n");
  printf("\tChecksum: %02X %02X\n\n", pkt->chksum_msb, pkt->chksum_lsb);
}

int main(int argc, char** argv)
{
  int fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (-1 == fd)
  {
    printf("Failed to open the serial port\n");
    return -1;
  }

  // Set up serial port as a "raw" port -- no special character interpretation, etc.
  struct termios config;
  cfmakeraw(&config);
  cfsetspeed(&config, B115200);
  if (tcsetattr(fd, TCSAFLUSH, &config) < 0)
  {
    printf("Failed to setup the serial port\n");
    return -1;
  }

  // Test setting a single set-idle packet
  printf("Setting device to IDLE\n");
  unsigned char buffer[8] = {0x75, 0x65, 0x01, 0x02, 0x02, 0x02, 0xE1, 0xC7};
  if (-1 == write(fd, buffer, 8))
  {
    printf("Unable to write to the serial port\n");
    return -1;
  }

  printf("Dumping Serial Stream:\n\n");
  uint8_t byte;
  MIPPkt_t pkt;
  uint8_t last_byte_was_sync_1 = 0;
  uint8_t first_sync_found = 0;
  while (1)
  {
    const ssize_t bytes_read = read(fd, &byte, 1);
    if (bytes_read > 0)
    {
      if (byte == 0x75)
      {
        last_byte_was_sync_1 = 1;
      }
      else if ((last_byte_was_sync_1 == 1) && (byte == 0x65))
      {
        last_byte_was_sync_1 = 0;
        first_sync_found = 1;
        initMIPPkt(&pkt);
        continue;
      }

      if (0 == first_sync_found)
      {
        continue;
      }

      if (MIP_DESCRIPTOR_UNINIT == pkt.descriptor)
      {
        pkt.descriptor = byte;
      }
      else if (MIP_PAYLOAD_LENGTH_UNINIT == pkt.payload_length)
      {
        pkt.payload_length = byte;
      }
      else if (pkt.payload_bytes_rcvd < pkt.payload_length)
      {
        pkt.payload_bytes[pkt.payload_bytes_rcvd] = byte;
        ++(pkt.payload_bytes_rcvd);
      }
      else if (0 == pkt.chksum_bytes_rcvd)
      {
        pkt.chksum_msb = byte;
        ++(pkt.chksum_bytes_rcvd);
      }
      else if (1 == pkt.chksum_bytes_rcvd)
      {
        pkt.chksum_lsb = byte;
        ++(pkt.chksum_bytes_rcvd);
        processMIPPkt(&pkt);
      }
      else if (last_byte_was_sync_1 != 1)
      {
        printf("Error - still getting non-sync bytes (%02X) past the end of the packet", byte);
      }
    }
  }
  close(fd);
  return 0;
}
