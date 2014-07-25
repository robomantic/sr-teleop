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

#include <iostream>
using namespace std;

namespace cyberglove_freq
{
const string CybergloveFreq::fastest = "t 1152 0\r"; //fastest speed, just for testing
const string CybergloveFreq::hundred_hz = "t 1152 1\r"; //100Hz
const string CybergloveFreq::fourtyfive_hz = "t 2560 1\r"; //45Hz
const string CybergloveFreq::ten_hz = "t 11520 1\r"; //10Hz
const string CybergloveFreq::one_hz = "t 57600 2\r"; //1Hz
}

namespace cyberglove
{
const unsigned short CybergloveSerial::glove_size = 22;

CybergloveSerial::CybergloveSerial(const string &serial_port, boost::function<void(vector<float>, bool) > callback) :
  cereal_port(),
  nb_msgs_received(0),
  glove_pos_index(0),
  current_value(0),
  glove_positions(glove_size, 0.0),
  callback_function(callback),
  light_on(true),
  button_on(true),
  no_errors(true)
{
  cereal_port.open(serial_port.c_str());
}

CybergloveSerial::~CybergloveSerial()
{
  cereal_port.stopStream();
  //stop the cyberglove transmission
  cereal_port.write("^c", 2);
}

int CybergloveSerial::set_filtering(bool value)
{
  if (value) //Filtering will be on
  {
    cereal_port.write("f 1\r", 4);
    cout << " - Data filtered" << '\n';
  }
  else // Filtering off
  {
    cereal_port.write("f 0\r", 4);
    cout << " - Data not filtered" << '\n';
  }
  cereal_port.flush();

  //wait for the command to be applied
  sleep(1);

  return 0;
}

int CybergloveSerial::set_transmit_info(bool value)
{
  if (value) //transmit info will be on
  {
    cereal_port.write("u 1\r", 4);
    cout << " - Additional info transmitted" << '\n';
  }
  else // transmit info off
  {
    cereal_port.write("u 0\r", 4);
    cout << " - Additional info not transmitted" << '\n';
  }
  cereal_port.flush();

  //wait for the command to be applied
  sleep(1);

  return 0;
}

int CybergloveSerial::set_frequency(string frequency)
{
  cereal_port.write(frequency.c_str(), frequency.size());
  cereal_port.flush();

  //wait for the command to be applied
  sleep(1);
  return 0;
}

int CybergloveSerial::start_stream()
{
  cout << "starting stream" << '\n';

  cereal_port.startReadStream(boost::bind(&CybergloveSerial::stream_callback, this, _1, _2));

  //start streaming by writing S to the serial port
  cereal_port.write("S", 1);
  cereal_port.flush();

  return 0;
}

void CybergloveSerial::stream_callback(char* world, int length)
{
  //read each received char.
  for (int i = 0; i < length; ++i)
  {
    current_value = (int) (unsigned char) world[i];
    switch (current_value)
    {
      case 'S':
        //the line starts with S, followed by the sensors values
        ++nb_msgs_received;
        //reset the index to 0
        glove_pos_index = 0;
        //reset no_errors to true for the new message
        no_errors = true;
        break;

      default:
        //this is a glove sensor value, a status byte or a "message end"
        switch (glove_pos_index)
        {
          case glove_size:
            //the last char of the msg is the status byte

            //the status bit 1 corresponds to the button
            if (current_value & 2)
              button_on = true;
            else
              button_on = false;
            //the status bit 2 corresponds to the light
            if (current_value & 4)
              light_on = true;
            else
              light_on = false;

            break;

          case glove_size + 1:
            //the last char of the line should be 0
            //if it is 0, then the full message has been received,
            //and we call the callback function.
            if (current_value == 0 && no_errors)
              callback_function(glove_positions, light_on);
            break;

          default:
            //this is a joint data from the glove
            //the value in the message should never be 0.
            if (current_value == 0)
              no_errors = false;
            // the values sent by the glove are in the range [1;254]
            //   -> we convert them to float in the range [0;1]
            glove_positions[glove_pos_index] = (((float) current_value) - 1.0f) / 254.0f;
            break;
        }

        ++glove_pos_index;
        break;
    }
  }
}

int CybergloveSerial::get_nb_msgs_received()
{
  return nb_msgs_received;
}
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
 */
