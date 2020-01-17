/**
 * 
 * @file   cyberglove_calib_publisher.cpp
 * @author Guillaume Walck <gwalck@techfak.uni-bielefeld.de>
 * @date   Oct 2017
* 
* original file 
* @file   shadowhand_publisher.cpp
* @author Ugo Cupcic <ugo@shadowrobot.com>
* @date   Thu Mar 25 15:36:41 2010
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
*
*
*/

// ROS include
#include <ros/ros.h>

// generic C/C++ include
#include <sstream>
#include <string>
#include <vector>

#include "cyberglove/cyberglove_calib_publisher.h"


namespace cyberglove
{
/////////////////////////////////
//    CONSTRUCTOR/DESTRUCTOR   //
/////////////////////////////////

CybergloveCalibPublisher::CybergloveCalibPublisher()
  : node(""), n_tilde("~")
{
  std::string path_to_calibration;
  n_tilde.param("path_to_calibration", path_to_calibration, std::string("/etc/robot/calibration.d/cyberglove.cal"));
  ROS_INFO("Calibration file loaded for the Cyberglove: %s", path_to_calibration.c_str());

  initialize_calibration(path_to_calibration);

  // publishes calibrated JointState messages
  std::string prefix;
  std::string searched_param;
  n_tilde.searchParam("cyberglove_prefix", searched_param);
  n_tilde.param(searched_param, prefix, std::string());
  std::string full_topic = prefix + "/calibrated/joint_states";
  cyberglove_pub = node.advertise<sensor_msgs::JointState>(full_topic, 2);

  // publishes raw JointState messages
  full_topic = prefix + "/raw/joint_states";
  cyberglove_raw_sub = node.subscribe(full_topic, 1, &CybergloveCalibPublisher::callback ,this);
}

CybergloveCalibPublisher::~CybergloveCalibPublisher()
{
}

void CybergloveCalibPublisher::initialize_calibration(std::string path_to_calibration)
{
  calibration_parser = xml_calibration_parser::XmlCalibrationParser(path_to_calibration);
}


/////////////////////////////////
//       CALLBACK METHOD       //
/////////////////////////////////
void CybergloveCalibPublisher::callback(const sensor_msgs::JointStateConstPtr &msg)
{
  sensor_msgs::JointState cal_js;
  // int the cal_js message
  cal_js.name = msg->name;
  cal_js.header = msg->header;
  ROS_INFO_ONCE("was there");
  // fill the joint_state msg with the calibrated data
  for (unsigned int index_joint = 0; index_joint < msg->name.size(); ++index_joint)
  {
	  ROS_DEBUG_STREAM("calib " << msg->name[index_joint] << " "<< msg->position[index_joint]);
    float calibration_value = calibration_parser.get_calibration_value(msg->position[index_joint], msg->name[index_joint]);
     ROS_DEBUG_STREAM("calib value " << calibration_value);
    cal_js.position.push_back(calibration_value);
    // set velocity to 0.
    cal_js.velocity.push_back(0.0);
  }

  // publish the msgs
  cyberglove_pub.publish(cal_js);
}

}  // namespace cyberglove

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
