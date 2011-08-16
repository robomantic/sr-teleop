/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "sr_edc_mechanism_controllers/srh_mixed_position_velocity_controller.hpp"
#include "angles/angles.h"
#include "pluginlib/class_list_macros.h"
#include <sstream>
#include <math.h>
#include "sr_utilities/sr_math_utils.hpp"

#include <std_msgs/Float64.h>

PLUGINLIB_DECLARE_CLASS(sr_edc_mechanism_controllers, SrhMixedPositionVelocityJointController, controller::SrhMixedPositionVelocityJointController, pr2_controller_interface::Controller)

using namespace std;

namespace controller {

  SrhMixedPositionVelocityJointController::SrhMixedPositionVelocityJointController()
    : joint_state_(NULL), command_(0),
      loop_count_(0),  initialized_(false), robot_(NULL), last_time_(0),
      n_tilde_("~"),
      max_velocity_(1.0), min_velocity_(-1.0), slope_velocity_(10.0),
      max_position_error_(0.0), min_position_error_(0.0),
      max_force_demand(1000.)
  {
    set_min_max_position_errors_();
  }

  SrhMixedPositionVelocityJointController::~SrhMixedPositionVelocityJointController()
  {
    sub_command_.shutdown();
  }

  bool SrhMixedPositionVelocityJointController::init(pr2_mechanism_model::RobotState *robot, const std::string &joint_name,
                                                     const control_toolbox::Pid &pid_velocity)
  {
    ROS_DEBUG(" --------- ");
    ROS_DEBUG_STREAM("Init: " << joint_name);

    assert(robot);
    robot_ = robot;
    last_time_ = robot->getTime();

    joint_state_ = robot_->getJointState(joint_name);
    if (!joint_state_)
    {
      ROS_ERROR("SrhMixedPositionVelocityController could not find joint named \"%s\"\n",
                joint_name.c_str());
      return false;
    }
    if (!joint_state_->calibrated_)
    {
      ROS_ERROR("Joint %s not calibrated for SrhMixedPositionVelocityJointController", joint_name.c_str());
      return false;
    }

    friction_interpoler = boost::shared_ptr<shadow_robot::JointCalibration>( new shadow_robot::JointCalibration( read_friction_map() ) );

    pid_controller_velocity_ = pid_velocity;

    serve_set_gains_ = node_.advertiseService("set_gains", &SrhMixedPositionVelocityJointController::setGains, this);

    ROS_DEBUG_STREAM(" joint_state name: " << joint_state_->joint_->name);
    ROS_DEBUG_STREAM(" In Init: " << getJointName() << " This: " << this
                     << " joint_state: "<<joint_state_ );

    std::stringstream ss;
    ss << getJointName() << "/set_velocity";

    if( std::string("FFJ3").compare(getJointName()) == 0)
    {
      ROS_INFO("Publishing debug infor for FFJ3 mixed position/velocity controller");
      std::stringstream ss2;
      ss2 << getJointName() << "debug_velocity";
      debug_pub = n_tilde_.advertise<std_msgs::Float64>(ss2.str(), 2);
    }

    return true;
  }

  bool SrhMixedPositionVelocityJointController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
  {
    assert(robot);
    node_ = n;

    std::string joint_name;
    if (!node_.getParam("joint", joint_name)) {
      ROS_ERROR("No joint given (namespace: %s)", node_.getNamespace().c_str());
      return false;
    }

    control_toolbox::Pid pid_velocity;
    if (!pid_velocity.init(ros::NodeHandle(node_, "pid")))
      return false;


    controller_state_publisher_.reset(
      new realtime_tools::RealtimePublisher<pr2_controllers_msgs::JointControllerState>
      (node_, "state", 1));

    sub_command_ = node_.subscribe<std_msgs::Float64>("command", 1, &SrhMixedPositionVelocityJointController::setCommandCB, this);

    return init(robot, joint_name, pid_velocity);
  }


  void SrhMixedPositionVelocityJointController::starting()
  {
    command_ = joint_state_->position_;
    pid_controller_velocity_.reset();
    ROS_WARN("Reseting PID");
  }

  bool SrhMixedPositionVelocityJointController::setGains(sr_robot_msgs::SetMixedPositionVelocityPidGains::Request &req,
                                                         sr_robot_msgs::SetMixedPositionVelocityPidGains::Response &resp)
  {
    pid_controller_velocity_.setGains(req.p,req.i,req.d,req.i_clamp,-req.i_clamp);
    max_force_demand = req.max_force;

    //setting the position controller parameters
    min_velocity_ = req.min_velocity;
    max_velocity_ = req.max_velocity;
    slope_velocity_ = req.velocity_slope;

    if( slope_velocity_ != 0.0 )
    {
      set_min_max_position_errors_();
      return true;
    }

    min_velocity_ = -1.0;
    max_velocity_ = 1.0;
    slope_velocity_ = 1.0;
    set_min_max_position_errors_();
    return false;
  }

  void SrhMixedPositionVelocityJointController::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
  {
    pid_controller_velocity_.getGains(p,i,d,i_max,i_min);
  }


  std::string SrhMixedPositionVelocityJointController::getJointName()
  {
    ROS_DEBUG_STREAM(" joint_state: "<<joint_state_ << " This: " << this);

    return joint_state_->joint_->name;
  }

// Set the joint position command
  void SrhMixedPositionVelocityJointController::setCommand(double cmd)
  {
    command_ = cmd;
  }

// Return the current position command
  void SrhMixedPositionVelocityJointController::getCommand(double & cmd)
  {
    cmd = command_;
  }

  void SrhMixedPositionVelocityJointController::update()
  {
    if (!joint_state_->calibrated_)
      return;

    assert(robot_ != NULL);
    ros::Time time = robot_->getTime();
    assert(joint_state_->joint_);
    dt_= time - last_time_;

    if (!initialized_)
    {
      initialized_ = true;
      command_ = joint_state_->position_;
    }

    //Compute velocity demand from position error:
    double error_position = joint_state_->position_ - command_;
    double commanded_velocity = compute_velocity_demand(error_position);

    if( std::string("FFJ3").compare(getJointName()) == 0)
    {
      std_msgs::Float64 msg;
      msg.data = commanded_velocity;
      debug_pub.publish(msg);
    }

    //velocity loop:
    double error_velocity = joint_state_->velocity_ - commanded_velocity;
    double commanded_effort = pid_controller_velocity_.updatePid(error_velocity, dt_);

    //Friction compensation
    //if( std::string("FFJ3").compare( getJointName() ) == 0 )
    //  ROS_INFO_STREAM(getJointName() << ": before fc: velocity demand=" << commanded_velocity << " force demand=" << commanded_effort << " / error: " << error_velocity );
    commanded_effort += friction_compensation( joint_state_->position_ );

    //if( std::string("FFJ3").compare( getJointName() ) == 0 )
    //  ROS_INFO_STREAM(getJointName() << ": after fc: effort=" << commanded_effort );

    commanded_effort += joint_state_->commanded_effort_;

    commanded_effort = min( commanded_effort, max_force_demand );
    commanded_effort = max( commanded_effort, -max_force_demand );

    joint_state_->commanded_effort_ = commanded_effort;


    if(loop_count_ % 10 == 0)
    {
      if(controller_state_publisher_ && controller_state_publisher_->trylock())
      {
        controller_state_publisher_->msg_.header.stamp = time;
        controller_state_publisher_->msg_.set_point = command_;
        controller_state_publisher_->msg_.process_value = joint_state_->position_;
        controller_state_publisher_->msg_.process_value_dot = joint_state_->velocity_;
        controller_state_publisher_->msg_.error = error_velocity;
        controller_state_publisher_->msg_.time_step = dt_.toSec();
        controller_state_publisher_->msg_.command = commanded_effort;

        double dummy;
        getGains(controller_state_publisher_->msg_.p,
                 controller_state_publisher_->msg_.i,
                 controller_state_publisher_->msg_.d,
                 controller_state_publisher_->msg_.i_clamp,
                 dummy);
        controller_state_publisher_->unlockAndPublish();
      }
    }
    loop_count_++;

    last_time_ = time;
  }

  double SrhMixedPositionVelocityJointController::friction_compensation( double position )
  {
    double compensation = 0.0;
    compensation = friction_interpoler->compute( position );
    return compensation;
  }

  void SrhMixedPositionVelocityJointController::setCommandCB(const std_msgs::Float64ConstPtr& msg)
  {
    command_ = msg->data;
  }


  std::vector<joint_calibration::Point> SrhMixedPositionVelocityJointController::read_friction_map()
  {
    std::vector<joint_calibration::Point> friction_map;
    std::string param_name = "/sr_friction_map";

    bool joint_not_found = true;

    XmlRpc::XmlRpcValue calib;
    node_.getParam(param_name, calib);

    ROS_DEBUG_STREAM("  Reading friction for: " <<  getJointName());
    ROS_DEBUG_STREAM(" value: " << calib);

    ROS_ASSERT(calib.getType() == XmlRpc::XmlRpcValue::TypeArray);
    //iterate on all the joints
    for(int32_t index_cal = 0; index_cal < calib.size(); ++index_cal)
    {
      //check the calibration is well formatted:
      // first joint name, then calibration table
      ROS_ASSERT(calib[index_cal][0].getType() == XmlRpc::XmlRpcValue::TypeString);
      ROS_ASSERT(calib[index_cal][1].getType() == XmlRpc::XmlRpcValue::TypeArray);

      std::string joint_name = static_cast<std::string> (calib[index_cal][0]);

      ROS_DEBUG_STREAM("  Checking joint name: "<< joint_name << " / " << getJointName());
      if(  joint_name.compare( getJointName() ) != 0 )
        continue;

      ROS_DEBUG_STREAM("   OK: joint name = "<< joint_name);

      joint_not_found = false;
      //now iterates on the calibration table for the current joint
      for(int32_t index_table=0; index_table < calib[index_cal][1].size(); ++index_table)
      {
        ROS_ASSERT(calib[index_cal][1][index_table].getType() == XmlRpc::XmlRpcValue::TypeArray);
        //only 2 values per calibration point: raw and calibrated (doubles)
        ROS_ASSERT(calib[index_cal][1][index_table].size() == 2);
        ROS_ASSERT(calib[index_cal][1][index_table][0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        ROS_ASSERT(calib[index_cal][1][index_table][1].getType() == XmlRpc::XmlRpcValue::TypeDouble);


        joint_calibration::Point point_tmp;
        point_tmp.raw_value = static_cast<double> (calib[index_cal][1][index_table][0]);
        point_tmp.calibrated_value = static_cast<double> (calib[index_cal][1][index_table][1]);
        friction_map.push_back(point_tmp);
      }

      break;
    }

    if( joint_not_found )
    {
      ROS_WARN_STREAM("  No friction compensation for: " << getJointName() );

      joint_calibration::Point point_tmp;
      point_tmp.raw_value = 0.0;
      point_tmp.calibrated_value = 0.0;
      friction_map.push_back(point_tmp);
      point_tmp.raw_value = 1.0;
      friction_map.push_back(point_tmp);
    }

    ROS_DEBUG_STREAM(" Friction map[" << getJointName() << "]");
    for( unsigned int i=0; i<friction_map.size(); ++i )
      ROS_DEBUG_STREAM("    -> position=" << friction_map[i].raw_value << " compensation: " << friction_map[i].calibrated_value);

    return friction_map;
  } //end read_friction_map

  double SrhMixedPositionVelocityJointController::compute_velocity_demand(double position_error)
  {
    if( position_error < min_position_error_ )
      return min_velocity_;

    if( position_error > max_position_error_ )
      return max_velocity_;

    return sr_math_utils::linear_interpolate_(position_error,
                                              min_position_error_, min_velocity_,
                                              max_position_error_, max_velocity_);
  }

  void SrhMixedPositionVelocityJointController::set_min_max_position_errors_()
  {
    //Because the slope goes through (0,0), we have:
    // velocity = slope_velocity_ * position_error
    min_position_error_ = min_velocity_ / slope_velocity_;
    max_position_error_ = max_velocity_ / slope_velocity_;
  }

}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/


