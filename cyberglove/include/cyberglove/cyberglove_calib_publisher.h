/**
 * 
 * @file   cyberglove_calib_publisher.h
 * @author Guillaume Walck <gwalck@techfak.uni-bielefeld.de>
 * @date   Oct 2017
* 
* original file 
 * @file   cyberglove_publisher.h
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Thu Apr 22 10:25:55 2010
 *
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
 * @brief The goal of this ROS publisher is to publish raw and calibrated
 * joint positions from the cyberglove at a regular time interval. We're
 * oversampling to get a better accuracy on our data.
 * To publish those data, just call the publish()
 * function.
 *
 *
 */

#ifndef CYBERGLOVE_CALIB_PUBLISHER_H
#define CYBERGLOVE_CALIB_PUBLISHER_H

#include <ros/ros.h>
#include <boost/smart_ptr.hpp>
#include <string>
#include <vector>


// messages
#include <sensor_msgs/JointState.h>
#include "cyberglove/xml_calibration_parser.h"

namespace cyberglove
{
class CybergloveCalibPublisher
{
public:
  /// Constructor
  CybergloveCalibPublisher();

  /// Destructor
  ~CybergloveCalibPublisher();

  ros::Publisher cyberglove_pub;
  ros::Subscriber cyberglove_raw_sub;
  void initialize_calibration(std::string path_to_calibration);

private:
  /////////////////
  //  CALLBACKS  //
  /////////////////

  // ros node handle
  ros::NodeHandle node, n_tilde;

  /**
   * The callback function: called each time a full message
   * is received. 
   */
  void callback(const sensor_msgs::JointStateConstPtr &msg);

  /// the calibration parser
  xml_calibration_parser::XmlCalibrationParser calibration_parser;

  std::vector<float> calibration_values;

};  // end class CybergloveCalibPublisher

}  // namespace cyberglove
#endif  // CYBERGLOVE_CALIB_PUBLISHER_H

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
