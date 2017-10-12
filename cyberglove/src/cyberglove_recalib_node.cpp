/**
 * 
 * @file   cyberglove_recalib_node.cpp
 * @author Guillaume Walck <gwalck@techfak.uni-bielefeld.de>
 * @date   Oct 2017
* derived from
 * @file   cyberglove_node.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Thu Apr 22 10:21:50 2010
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
 * @brief  The cyberglove recali node calibrates incoming joint_states data
 *
 *
 */

#include <ros/ros.h>
#include <time.h>
#include <boost/smart_ptr.hpp>

#include "cyberglove/cyberglove_calib_publisher.h"

using namespace cyberglove;

/////////////////////////////////
//           MAIN              //
/////////////////////////////////

/**
 *  Start the cyberglove recalib publisher.
 *
 * @param argc
 * @param argv
 *
 * @return -1 if error
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "cyberglove_recalib");
  // NodeHandle n;
  try
  {
    boost::shared_ptr<CybergloveCalibPublisher> cyberglove_calib_pub(new CybergloveCalibPublisher());
    ros::spin();
  }
  catch (int e)
  {
    ROS_FATAL("could not create recalib publisher, leaving");
    return -1;
  }

  return 0;
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
