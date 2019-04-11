#!/usr/bin/env python

# derived from original file joint_trajectory_controller.py
# of rqt_joint_trajectory_controller
#
# Copyright (C) 2014, PAL Robotics S.L.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
# * Neither the name of PAL Robotics S.L. nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os
import rospy
import rospkg

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from urdf_parser_py import urdf

cmd_pub_freq = 10.0  # Hz

class JointStateToJointTrajectoryPoint(object):
    """
    Conversion to JointTrajectoryPoint commands from JointState
    """
    _min_traj_dur = 5.0 / cmd_pub_freq  # Minimum trajectory duration

    def __init__(self, jtc_name):

        # Initialize members
        self._joint_pos = {}  # name->pos map for joints of selected controller
        self._joint_names = []  # Ordered list of selected controller joints
        self._robot_joint_limits = {}  # Lazily evaluated on first use

        self._cmd_pub = None    # Controller command publisher
        self._state_sub = None  # Controller state subscriber

        if not self._robot_joint_limits:
            self._robot_joint_limits = self.get_joint_limits()  # Lazy evaluation

        state_topic='cmd_joint_states'
        cmd_topic = jtc_name + '/command'
        self._state_sub = rospy.Subscriber(state_topic,
                                           JointState,
                                           self._state_cb,
                                           queue_size=1)
        self._cmd_pub = rospy.Publisher(cmd_topic,
                                        JointTrajectory,
                                        queue_size=1)

    def _on_joint_state_change(self, actual_pos):
        # TODO:lock
        for name in actual_pos.keys():
            self._joint_pos[name] = actual_pos[name]
        # TODO:unlock

    def _state_cb(self, msg):
        actual_pos = {}
        self._joint_names = msg.name
        for i in range(len(msg.name)):
            joint_name = msg.name[i]
            actual_pos[joint_name] = msg.position[i]
        self._on_joint_state_change(actual_pos)

    def update_cmd_cb(self):
        dur = []
        traj = JointTrajectory()
        traj.joint_names = self._joint_names
        point = JointTrajectoryPoint()
        if len(self._joint_names) > 0:
            for name in traj.joint_names:
                pos = self._joint_pos[name]
                cmd = pos

                if len(self._robot_joint_limits) and name in self._robot_joint_limits:
                    max_vel = self._robot_joint_limits[name]['max_velocity']
                    dur.append(max(abs(cmd - pos) / max_vel, self._min_traj_dur))
                else:
                    dur.append(self._min_traj_dur)
                point.positions.append(cmd)
            point.time_from_start = rospy.Duration(max(dur))
            traj.points.append(point)

            self._cmd_pub.publish(traj)

    def get_joint_limits(self):
        robot_joint_limits = {}
        # load the urdf from the parameter server and parse it
        try:
            robot = urdf.from_parameter_server()
            # extract joint velocity limits
            for joint in robot.joints:
                if joint.name not in robot_joint_limits:
                    robot_joint_limits[joint.name] = {}
                robot_joint_limits[joint.name]['max_velocity'] = joint.limit.velocity
        except Exception, e:
            rospy.logerr("no joint velocity limits found, not adapting to max velocity")
        return robot_joint_limits


# main
if __name__ == "__main__":
    rospy.init_node("js_to_jtp")
    jtc_name = rospy.get_param("~jtc_name", "rh_hand_trajectory_controller")
    js2jtp = JointStateToJointTrajectoryPoint(jtc_name)

    rate = rospy.Rate(10)
    rospy.loginfo("js_to_jtp running")
    while not rospy.is_shutdown():
      js2jtp.update_cmd_cb()
      rate.sleep()

