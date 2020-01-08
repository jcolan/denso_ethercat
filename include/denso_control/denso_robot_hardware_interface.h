/****************************************************************************
# denso_hardware_interface.h:  DENSO EtherCAT Motor Controller              #
# Copyright (C) 2017                                                          #
# Hasegawa Laboratory at Nagoya University                                    #
#                                                                             #
# Redistribution and use in source and binary forms, with or without          #
# modification, are permitted provided that the following conditions are met: #
#                                                                             #
#     - Redistributions of source code must retain the above copyright        #
#       notice, this list of conditions and the following disclaimer.         #
#     - Redistributions in binary form must reproduce the above copyright     #
#       notice, this list of conditions and the following disclaimer in the   #
#       documentation and/or other materials provided with the distribution.  #
#     - Neither the name of the Hasegawa Laboratory nor the                   #
#       names of its contributors may be used to endorse or promote products  #
#       derived from this software without specific prior written permission. #
#                                                                             #
# This program is free software: you can redistribute it and/or modify        #
# it under the terms of the GNU Lesser General Public License LGPL as         #
# published by the Free Software Foundation, either version 3 of the          #
# License, or (at your option) any later version.                             #
#                                                                             #
# This program is distributed in the hope that it will be useful,             #
# but WITHOUT ANY WARRANTY; without even the implied warranty of              #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the                #
# GNU Lesser General Public License LGPL for more details.                    #
#                                                                             #
# You should have received a copy of the GNU Lesser General Public            #
# License LGPL along with this program.                                       #
# If not, see <http://www.gnu.org/licenses/>.                                 #
#                                                                             #
# #############################################################################
#                                                                             #
#   Author: Jacinto E. Colan, email: colan@robo.mein.nagoya-u.ac.jp           #
#                                                                             #
# ###########################################################################*/

#ifndef DENSO_CONTROL_DENSO_ROBOT_HARDWARE_INTERFACE_
#define DENSO_CONTROL_DENSO_ROBOT_HARDWARE_INTERFACE_

// ROS
#include <ros/ros.h>

// Eigen
#include <Eigen/Dense>

// ros_control
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

// denso_client
#include <denso_control/denso_client.h>
#include <ethercat_manager/ethercat_manager.h>

using namespace Eigen;

namespace denso_control
{

struct JointData {
  std::string name_;
  int         slave_no_;
  int         joint_no_;

  double cmd_;
  double pos_;
  double vel_;
  double eff_;

  // Input
  int32 position_actual_value; // 6064h : Position actual value
  int32 velocity_actual_value; // 606Ch : Velocity actual value
  int16 current_actual_value;  // 6078h : Current actual value
};

class JointControlInterface
{
public:
  JointControlInterface(int slave_no, int joint_no,
                        hardware_interface::JointStateInterface &   jnt_stat,
                        hardware_interface::PositionJointInterface &jnt_cmd)
  {
    // create joint name
    std::stringstream ss;
    ss << "joint" << ((slave_no - 1) * 6 + joint_no);
    joint.name_ = ss.str();
    joint.joint_no_ = joint_no;
    joint.slave_no_ = slave_no;

    hardware_interface::JointStateHandle state_handle(joint.name_, &joint.pos_, &joint.vel_,
                                                      &joint.eff_);
    jnt_stat.registerHandle(state_handle);

    hardware_interface::JointHandle pos_handle(jnt_stat.getHandle(joint.name_), &joint.cmd_);
    jnt_cmd.registerHandle(pos_handle);

    ROS_INFO_STREAM("Joint: " << joint.name_ << " added");
  }

  JointControlInterface(int slave_no, int joint_no,
                        hardware_interface::JointStateInterface &   jnt_stat,
                        hardware_interface::VelocityJointInterface &jnt_cmd)
  {
    // create joint name
    std::stringstream ss;
    ss << "joint" << ((slave_no - 1) * 6 + joint_no);
    joint.name_ = ss.str();
    joint.joint_no_ = joint_no;
    joint.slave_no_ = slave_no;

    hardware_interface::JointStateHandle state_handle(joint.name_, &joint.pos_, &joint.vel_,
                                                      &joint.eff_);
    jnt_stat.registerHandle(state_handle);

    hardware_interface::JointHandle vel_handle(jnt_stat.getHandle(joint.name_), &joint.cmd_);
    jnt_cmd.registerHandle(vel_handle);

    ROS_INFO_STREAM("Joint: " << joint.name_ << " added");
  }

  ~JointControlInterface(){};
  JointData read();
  void      write(double cmd);
  void      write(double cmd, double pos, double vel, double eff);
  void      write(double pos, double vel, double eff, int32 pos_actual, int32 vel_actual,
                  int16 eff_actual);
  void      write(double pos, double eff, int32 pos_actual, int16 eff_actual);
  void      shutdown();

  void getInputActualValueToStatus(std::string &joint_name, int32 &position_actual_value,
                                   int32 &velocity_actual_value, int16 &current_actual_value)
  {
    joint_name = joint.name_;
    position_actual_value = joint.position_actual_value;
    velocity_actual_value = joint.velocity_actual_value;
    current_actual_value = joint.current_actual_value;
  }

  double getJointValue() { return joint.pos_; }

protected:
  JointData joint;
};

class EtherCATDensoRobot
{
private:
  int          control_mode_;
  unsigned int n_dof_;
  unsigned int slave_no_;

  typedef std::vector<JointControlInterface *> JointControlContainer;
  JointControlContainer                        controllers;

private:
  denso_control::DensoClient *client;
  denso_control::RC8Input     input;
  denso_control::RC8Output    output;

public:
  EtherCATDensoRobot(ethercat::EtherCatManager *manager, int slave_no, int n_dof, int control_mode,
                     hardware_interface::JointStateInterface &   jnt_stat,
                     hardware_interface::PositionJointInterface &jnt_cmd_pos,
                     hardware_interface::VelocityJointInterface &jnt_cmd_vel);

  ~EtherCATDensoRobot();
  // void registerControl(JointControlInterface *);
  void read();
  void write();
  void writeCmd(Eigen::Matrix<double, 6, 1> qdot);
  void shutdown();
  int  getInputActualValueToStatus(std::vector<std::string> &joint_names,
                                   std::vector<int32> &      position_actual_values,
                                   std::vector<int32> &      velocity_actual_values,
                                   std::vector<int16> &      current_actual_values);

  Eigen::Matrix<double, 6, 1> getJointValues();
};

} // namespace denso_control

#endif // DENSO_CONTROL_DENSO_ROBOT_HARDWARE_INTERFACE_
