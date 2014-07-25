/**
 * @file   leaphand_to_cyberglove_remapper.h
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Thu May 13 09:44:52 2010
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
 * @brief This program remaps the force information contained in
 * /joint_states coming from the hand to the /cybergraspforces topic
 * used to control the cybergrasp.
 *
 *
 */

#ifndef   	LEAPHAND_TO_CYBERGLOVE_REMAPPER_H_
#define   	LEAPHAND_TO_CYBERGLOVE_REMAPPER_H_

//messages
#include <sensor_msgs/JointState.h>
#include "sr_remappers/calibration_parser.h"

using namespace ros;

namespace leaphand_to_cyberglove_remapper
{

/**
 * This program remaps the force information contained in
 * /joint_states coming from the hand to the /cybergraspforces topic
 * used to control the cybergrasp.
 */
class LeaphandToCybergloveRemapper
{
public:
  /**
   * Init the publisher / subscriber, the joint names, read the calibration matrix
   */
  LeaphandToCybergloveRemapper();
private:
  /**
   * Number of joints in the hand
   */
  static const unsigned int number_hand_joints;

  /**
   * Init the vector containing the joints names
   *
   */
  void init_names();
  /// ROS node handles
  NodeHandle node, n_tilde;
  /// Vector containing all the joints names for the leaphand.
  std::vector<std::string> joints_names;
  /// subscriber to the jointstates topic from the cyberglove
  Subscriber cyberglove_jointstates_sub;
  ///publish to the leaphand sendupdate topic
  Publisher leaphand_pub;
  ///the calibration parser containing the mapping matrix
  CalibrationParser* calibration_parser;

  /////////////////
  //  CALLBACKS  //
  /////////////////

  /**
   * process the joint_states callback: receives the message from the cyberglove node, remap it to the Dextrous hand and
   * publish this message on a given topic
   *
   * @param msg the joint_states message
   */
  void jointstatesCallback(const sensor_msgs::JointStateConstPtr& msg);

}; // end class

} //end namespace

#endif 	    /* !LEAPHAND_TO_CYBERGLOVE_REMAPPER_H_ */
