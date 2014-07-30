/**
 * @file   serial_glove.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Thu May  5 15:30:17 2011
 *
 * Copyright 2011 Shadow Robot Company Ltd.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @brief Communicate via the serial port with the Cyberglove.
 *
 */

#include "cyberglove/serial_glove.hpp"

#include <ros/ros.h>
#include <termios.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>

using namespace std;
using boost::function;

namespace cyberglove
{

CybergloveSerial::CybergloveSerial(const char *serial_port, function<void(vector<float>, bool) > callback) :
  init_success_(false),
  nb_msgs_received_(0),
  glove_pos_index_(0),
  current_value_(0),
  glove_positions_(glove_size_, 0.0),
  callback_function_(callback),
  stream_thread_(&cyberglove::CybergloveSerial::read_thread, this),
  stream_paused_(true),
  stop_stream_(false),
  light_on_(true),
  button_on_(true),
  no_errors_(true),
  // Make IO non blocking. This way there are no race conditions that
  // cause blocking when a badly behaving process does a read at the same
  // time as us. Will need to switch to blocking for writes or errors
  // occur just after a re-plug event

  fd_(open(serial_port, O_RDWR | O_NONBLOCK | O_NOCTTY))
{
  if (fd_ == -1)
  {
    ROS_FATAL("Failed to open port: %s\n%s (errno = %d)\n", serial_port, strerror(errno), errno);
    if (errno == EACCES)
      ROS_FATAL("You probably don't have permission to open the port for reading and writing");
    else if (errno == ENOENT)
      ROS_FATAL("The requested port does not exist. Is the cyberglove connected ? Was the port name misspelled ?");
    return;
  }

  struct flock fl = {};
  fl.l_type = F_WRLCK;
  fl.l_whence = SEEK_SET;
  fl.l_pid = getpid();

  if (fcntl(fd_, F_SETLK, &fl) != 0)
  {
    ROS_FATAL("Device %s is already locked. Try 'lsof | grep %s' to find other processes that own the port",
              serial_port,
              serial_port);
    close(fd_);
    return;
  }

  struct termios newtio = {};

  if (tcgetattr(fd_, &newtio) != 0)
  {
    ROS_FATAL("failed to get serial port attributes");
    close(fd_);
    return;
  }

  newtio.c_cflag = CS8 | CLOCAL | CREAD;
  newtio.c_iflag = IGNPAR;
  if (cfsetspeed(&newtio, B1152000) != 0)
  {
    ROS_FATAL("failed to set serial baud rate");
    close(fd_);
    return;
  }

  // Activate new settings
  if (tcflush(fd_, TCIFLUSH) != 0)
  {
    ROS_FATAL("failed to flush serial port");
    close(fd_);
    return;
  }

  if (tcsetattr(fd_, TCSANOW, &newtio) != 0)
  {
    ROS_FATAL("Unable to set serial port attributes. %s may be an invalid port",
              serial_port);
    close(fd_);
    return;
  }

  usleep(200000);
  init_success_ = true;
}

CybergloveSerial::~CybergloveSerial()
{
  stop_stream_ = true;
  stream_thread_.join();

  //stop the cyberglove transmission
  if (write(fd_, "^c", 2) != 2)
    ROS_FATAL("failed to stop cyberglove streaming");
  close(fd_);
}

bool CybergloveSerial::set_params(int frequency, bool filtering, bool transmit_info)
{
  // IO is currently non-blocking. This is what we want for the more common read case.
  int origflags = fcntl(fd_, F_GETFL, 0);
  fcntl(fd_, F_SETFL, origflags & ~O_NONBLOCK);

  int ret = 0;

  if (frequency == 45)
    ret = write(fd_, "t 2560 1\r", 9);
  else if (frequency == 10)
    ret = write(fd_, "t 11520 1\r", 10);
  else if (frequency == 1)
    ret = write(fd_, "t 57600 2\r", 10);
  else // 100
    ret = write(fd_, "t 1152 1\r", 9);

  if (ret < 9)
  {
    ROS_FATAL("failed to set frequency");
    return false;
  }

  //wait for the command to be applied
  sleep(1);

  if (filtering)
  {
    ret = write(fd_, "f 1\r", 4);
    ROS_INFO(" - Data filtered");
  }
  else // Filtering off
  {
    ret = write(fd_, "f 0\r", 4);
    ROS_INFO(" - Data not filtered");
  }

  if (ret != 4)
  {
    ROS_FATAL("failed to set filtering");
    return false;
  }

  //wait for the command to be applied
  sleep(1);

  if (transmit_info)
  {
    ret = write(fd_, "u 1\r", 4);
    ROS_INFO(" - Additional info transmitted");
  }
  else // transmit info off
  {
    ret = write(fd_, "u 0\r", 4);
    ROS_INFO(" - Additional info not transmitted");
  }

  if (tcflush(fd_, TCIOFLUSH) != 0)
    ROS_FATAL("tcflush failed");

  if (ret != 4)
  {
    ROS_FATAL("failed to set the transmit info");
    return false;
  }

  //wait for the command to be applied
  sleep(1);

  fcntl(fd_, F_SETFL, origflags | O_NONBLOCK);

  return true;
}

void CybergloveSerial::start_stream()
{
  ROS_INFO("Starting cyberglove serial port stream");

  stream_paused_ = false;

  //start streaming by writing S to the serial port
  if (write(fd_, "S", 1) != 1)
    ROS_FATAL("Failed to start cyberglove serial port stream");
}

void CybergloveSerial::stream_callback(char* world, int length)
{
  //read each received char.
  for (int i = 0; i < length; ++i)
  {
    current_value_ = (unsigned char) world[i];
    if (current_value_ == 'S')
    {
      //the line starts with S, followed by the sensors values
      ++nb_msgs_received_;
      //reset the index to 0
      glove_pos_index_ = 0;
      //reset no_errors to true for the new message
      no_errors_ = true;
      break;
    }
    else
    {
      //this is a glove sensor value, a status byte or a "message end"
      switch (glove_pos_index_)
      {
        case glove_size_:
          //the last char of the msg is the status byte

          //the status bit 1 corresponds to the button
          if (current_value_ & 2)
            button_on_ = true;
          else
            button_on_ = false;
          //the status bit 2 corresponds to the light
          if (current_value_ & 4)
            light_on_ = true;
          else
            light_on_ = false;

          break;

        case glove_size_ + 1:
          //the last char of the line should be 0
          //if it is 0, then the full message has been received,
          //and we call the callback function.
          if (current_value_ == 0 && no_errors_)
            callback_function_(glove_positions_, light_on_);
          break;

        default:
          //this is a joint data from the glove
          //the value in the message should never be 0.
          if (current_value_ == 0)
            no_errors_ = false;
          // the values sent by the glove are in the range [1;254]
          //   -> we convert them to float in the range [0;1]
          glove_positions_[glove_pos_index_] = (((float) current_value_) - 1.0) / 254.0;
          break;
      }
      ++glove_pos_index_;
    }
  }
}

void CybergloveSerial::read_thread()
{
  char data[128];

  struct pollfd ufd;
  ufd.fd = fd_;
  ufd.events = POLLIN;

  while (!stop_stream_)
  {
    if (!stream_paused_)
    {
      int poll_ret = poll(&ufd, 1, 10);

      ROS_INFO_STREAM_THROTTLE(1, "poll_ret " << poll_ret << " ufd.revents " << ufd.revents);

      if (poll_ret > 0 && ufd.revents != POLLERR)
      {
        int ret = read(fd_, data, 128);
        ROS_INFO_STREAM_THROTTLE(1, "read_ret " << ret);
        if (ret > 0)
          stream_callback(data, ret);
      }
    }
  }
}
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
 */
