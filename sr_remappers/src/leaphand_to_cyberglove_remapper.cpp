/**
 * @file   leaphand_to_cyberglove_remapper.cpp
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
 * @brief This program remapps the force information contained in
 * /joint_states coming from the hand to the /cybergraspforces topic
 * used to control the cybergrasp.
 *
 *
 */

//ROS include
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

//generic include
#include <string>

//own .h
#include "sr_remappers/leaphand_to_cyberglove_remapper.h"
#include <sr_robot_msgs/joint.h>

using namespace ros;
using namespace std;

namespace leaphand_to_cyberglove_remapper
{

const unsigned int LeaphandToCybergloveRemapper::number_hand_joints = 15;

LeaphandToCybergloveRemapper::LeaphandToCybergloveRemapper() :
  n_tilde("~"),
  joints_names(number_hand_joints)
{
  LeaphandToCybergloveRemapper::init_names();

  string param;
  string path;
  n_tilde.searchParam("leaphand_cyberglove_mapping_path", param);
  n_tilde.param(param, path, string());
  calibration_parser = new CalibrationParser(path);
  ROS_INFO("Mapping file loaded for the Cyberglove: %s", path.c_str());

  string full_topic = "/cyberglove/calibrated/joint_states";
  cyberglove_jointstates_sub = node.subscribe(full_topic, 10, &LeaphandToCybergloveRemapper::jointstatesCallback, this);

  // n_tilde.searchParam("sendupdate_prefix", searched_param);
  // n_tilde.param(searched_param, prefix, string());
  // full_topic = prefix + "sendupdate";

  leaphand_pub = node.advertise<sensor_msgs::JointState> ("/leap_hand_joint_states", 5);
}

void LeaphandToCybergloveRemapper::init_names()
{
  joints_names[0] = "THJ1";
  joints_names[1] = "THJ2";
  joints_names[2] = "THJ3";
  joints_names[3] = "FFJ1";
  joints_names[4] = "FFJ2";
  joints_names[5] = "FFJ3";
  joints_names[6] = "MFJ1";
  joints_names[7] = "MFJ2";
  joints_names[8] = "MFJ3";
  joints_names[9] = "RFJ1";
  joints_names[10] = "RFJ2";
  joints_names[11] = "RFJ3";
  joints_names[12] = "LFJ1";
  joints_names[13] = "LFJ2";
  joints_names[14] = "LFJ3";
}

void LeaphandToCybergloveRemapper::jointstatesCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  sensor_msgs::JointState pub;

  //Do conversion
  vector<double> vect = calibration_parser->get_remapped_vector(msg->position);

  vector<sr_robot_msgs::joint> table(number_hand_joints);
  for (unsigned int i = 0; i < number_hand_joints; ++i)
  {
    pub.name.push_back(joints_names[i]);
    pub.position.push_back(vect[i]);
    ROS_ERROR_STREAM(joints_names[i] << " :: " << vect[i]);
  }

  leaphand_pub.publish(pub);
}
}//end namespace
