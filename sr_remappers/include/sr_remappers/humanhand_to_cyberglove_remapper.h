/**
 * @file   shadowhand_to_cyberglove_remapper.h
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Thu May 13 09:44:52 2010
 *
 * @author Guillaume Walck <gwalck@techfak.uni-bielefeld.de>
 * @date   Fri Apr 28 10:20 2010
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
 * @brief This program remaps the cyberglove joint information to 
 * joint_states in of a human hand display
 *
 *
 */

#ifndef   	HUMANHAND_TO_CYBERGLOVE_REMAPPER_H_
# define   	HUMANHAND_TO_CYBERGLOVE_REMAPPER_H_

//messages
#include <sensor_msgs/JointState.h>
#include "sr_remappers/calibration_parser.h"

using namespace ros;

namespace humanhand_to_cyberglove_remapper{

/**
 * This program remaps the cyberglove joint information to 
 * joint_states in of a human hand display
 */
class HumanhandToCybergloveRemapper
{
 public:
  /**
   * Init the publisher / subscriber, the joint names, read the calibratin matrix
   */
  HumanhandToCybergloveRemapper();
  ~HumanhandToCybergloveRemapper(){};
 private:
  /**
   * Number of joints in the hand
   */
  static const unsigned int number_hand_joints;

  sensor_msgs::JointState js_msg;

  /**
   * Init the vector containing the joints names
   *
   */
  void init_names();
  /// ROS node handles
  NodeHandle node, n_tilde;
  /// in order to limit or not the abduction
  double abduction_max;
  /// Vector containing all the joints names for the shadowhand.
  std::vector<std::string> joints_names;
  /// subscriber to the jointstates topic from the cyberglove
  Subscriber cyberglove_jointstates_sub;
  ///publish to the shadowhand joint_states topic
  Publisher js_pub;
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

  /**
   * process the joint_states callback for the finger abductions: processes the message from the cyberglove node, remap it to the Dextrous hand J4s
   * It overwrites whatever was written for the J4s by the calibration parser get_remapped_vector
   *
   * @param msg the joint_states message
   * @param vect the vector where the result is written (only J4s are written)
   */
  void getAbductionJoints( const sensor_msgs::JointStateConstPtr& msg, std::vector<double>& vect);

}; // end class

} //end namespace

#endif 	    /* !HUMANHAND_TO_CYBERGLOVE_REMAPPER_H_ */
