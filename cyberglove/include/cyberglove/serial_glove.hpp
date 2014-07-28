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
 *
 * List of available commands, gathered from an old specification:
 * (http://www.cyberglovesystems.com/forum/viewtopic.php?f=14&t=247)
 *   - 'F0' -> stops filter
 *   - 'F1' -> starts filter
 *   - '?F' -> current filter status
 *
 *   - 'G'  -> reads data from the cyberglove once
 *   - 'S'  -> streams data from the cyberglove at given period
 *   - 't 1152 1'  -> specifies period of 100Hz
 *   - '?t' -> get period
 *   - '^C' -> stop streaming ??
 *
 *   - 'L0' -> turns light off
 *   - 'L1' -> turns light on
 *   - '?L' -> light status
 *
 *   - 'u1' -> transmits the status each time:
 *               o Bit 0: cyberglove plugged in
 *               o Bit 1: switch on/off
 *               o Bit 2: light on/off
 *   - 'u0' -> stop transmitting the status
 *
 *   - 'd1' -> transmits the timestamp as well
 *
 *   - '?r' -> right handed or not
 *   - '?n' -> number of sensors
 *   - '?i' -> info about the cyberglove
 *   - '^I' or '^R' -> reinitialize / restart the cyberglove firmware (wait 1-2s)
 *
 *
 *
 * Proposed strategy:
 *   - F,1 : set filtering to 0 (faster, we're going to do oversampling anyway)
 *   - U,1 : transmit the status each time
 *   - T,1152,1: set transmit period to 100Hz
 *   - S : streams data
 *         o If data[2] = 0 (light off), ignore
 *         o Else (light on): read the data (start from 5: 2 first letters are 'G' and ' ',
 *           then we have the status bits)
 */

#ifndef _SERIAL_GLOVE_HPP_
#define _SERIAL_GLOVE_HPP_

#include <boost/smart_ptr.hpp>
#include <boost/function.hpp>
#include <boost/thread/thread.hpp>

namespace cyberglove
{

/**
 * This class uses the Cereal Port ROS package to connect to
 * and interact with the Cyberglove.
 */
class CybergloveSerial
{
public:
  /**
   * Initializes the connection with the cyberglove through the given serial port.
   *
   * @param serial_port the path to the serial port, /dev/ttyS0 by default
   * @param callback a pointer to a callback function, which will be called each time a
   *                 complete joint message is received.
   */
  CybergloveSerial(const char *serial_port, boost::function<void(std::vector<float>, bool) > callback);
  ~CybergloveSerial();

  /**
   * Set the transmit frequency for the cyberglove.

   * Turns on or off the filtering (done directly in the cyberglove). By default the filtering
   * is activated. We recommend turning it off if you want to do oversampling, to get the fastest
   * rate (the rate is divided by 2-3 if the filtering is on)
   *
   * Turns on or off the status transmission: if it's on, then a char is added to the message
   * to describe the current status of the glove. For this status byte, the bit 1 corresponds
   * to the button status, and the bit 2 corresponds to the light status.
   *
   * @param frequency (100,45,10,1) default 100
   * @param value true if you want filtering on.
   * @param value true if you want transmit_info on.
   *
   * @return true on success
   *
   */
  bool set_params(int frequency, bool filtering, bool transmit_info);

  /**
   * Start streaming the data from the cyberglove, calling the
   * callback function each time the full message is received.
   *
   */
  void start_stream();

  /**
   * We keep the count of all the messages received for the glove.
   *
   * @return the number of received messages.
   */
  int get_nb_msgs_received()
  {
    return nb_msgs_received_;
  }

  /**
   * The number of sensors in the glove.
   */
  static const size_t glove_size_ = 22;

  bool init_success_;

private:

  /**
   * The callback function for the raw data coming from the
   * serial port, bound to the cereal_port callback. The data received
   * here is not received message by messages: it's a stream of data, coming
   * at different intervals (the whole messages are received at a given frequency
   * though)
   *
   * @param world a table of char containing the binary values from the serial port
   * @param length the length of the received message.
   */
  void stream_callback(char* world, int length);

  /// function for the stream_thread_
  void read_thread();

  int nb_msgs_received_;
  int glove_pos_index_;
  int current_value_;

  /// A vector containing the current joints positions.
  std::vector<float> glove_positions_;

  /**
   * The pointer to the function called each time a full message is received.
   * This function is linked when instantiating the class.
   */
  boost::function<void(std::vector<float>, bool) > callback_function_;

  /// Stream thread
  boost::thread stream_thread_;

  bool stream_paused_;
  bool stop_stream_;

  bool light_on_;
  bool button_on_;

  ///Did we get any garbage in the received message?
  bool no_errors_;

  /// File descriptor
  int fd_;
};
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
 */

#endif
