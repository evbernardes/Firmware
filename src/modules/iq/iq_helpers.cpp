#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <parameters/param.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
// #include <math.h>
#include <cmath>
#include <fcntl.h>
#include <termios.h>
#define clrscr() PX4_INFO("\e[1;1H\e[2J")

#include <uORB/uORB.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/sensor_combined.h>

#include "generic_interface.hpp"
#include "voltage_super_position_client.hpp"
#include "system_control_client.hpp"
#include "propeller_motor_control_client.hpp"

/**
 * setup_uart initializes a uart port to 115200 8N1
 * uart_name is the port string descriptor (/dev/ttySx)
 * serial_fd is the resulting file descriptor for the port
 * returns 0 if successful, -1 if setup is unable to setup the port
 */
int setup_uart(char *uart_name, int &serial_fd)
{

  PX4_INFO("opening port %s", uart_name);

  serial_fd = open(uart_name, O_RDWR | O_NOCTTY);

  if (serial_fd < 0) {
    PX4_WARN("failed to open port: %s", uart_name);
    return -1;
  }

  /* Try to set baud rate */
  struct termios uart_config;
  struct termios uart_config_original;
  int termios_state;

  /* Back up the original uart configuration to restore it after exit */
  if ((termios_state = tcgetattr(serial_fd, &uart_config_original)) < 0) {
    PX4_WARN("ERR GET CONF %s: %d\n", uart_name, termios_state);
    close(serial_fd);
    return -1;
  }

  tcgetattr(serial_fd, &uart_config);
  uart_config.c_cflag = (uart_config.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
  // disable IGNBRK for mismatched speed tests; otherwise receive break
  // as \000 chars
  uart_config.c_iflag &= ~IGNBRK;         // disable break processing
  uart_config.c_lflag = 0;                // no signaling chars, no echo,
  uart_config.c_iflag &= ~INLCR;
  uart_config.c_iflag &= ~ICRNL;
  uart_config.c_oflag &= ~OCRNL;
  uart_config.c_oflag &= ~ONLCR;
  // no canonical processing
  uart_config.c_oflag = 0;                // no remapping, no delays
  uart_config.c_cc[VMIN]  = 0;            // read doesn't block
  uart_config.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

  uart_config.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

  uart_config.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
  // enable reading
  uart_config.c_cflag &= ~(PARENB | PARODD);      // shut off parity
  uart_config.c_cflag |= 0;
  uart_config.c_cflag &= ~CSTOPB;
  uart_config.c_cflag &= ~CRTSCTS;

  /* Set baud rate */
  const speed_t speed = B115200;
  // const speed_t speed = B57600;

  /* Set baud rate */
  if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
    PX4_WARN("ERR SET BAUD %s: %d\n", uart_name, termios_state);
    close(serial_fd);
    return -1;
  }

  if ((termios_state = tcsetattr(serial_fd, TCSANOW, &uart_config)) < 0) {
    PX4_WARN("ERR SET CONF %s\n", uart_name);
    close(serial_fd);
    return -1;
  }
  return 0;
}


int send_msgs_to_uart(GenericInterface& comm, int serial_fd)
{
  // This buffer is for passing around messages.
  uint8_t communication_buffer[256];
  // Stores length of message to send or receive
  uint8_t communication_length;

  // Grab outbound messages in the com queue, store into buffer
  // If it transferred something to communication_buffer...
  if(serial_fd >= 0 && comm.GetTxBytes(communication_buffer,communication_length))
  {
    //TODO::do the write in a while loop, decrementing com_length by written and incrementing buffer address
    uint8_t written = ::write(serial_fd, communication_buffer, communication_length);
    ::fsync(serial_fd);
    if(written != communication_length)
      return -1;
    return 0;
  }
  return -2;
}
