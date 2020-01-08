/****************************************************************************
# denso_hardware_interface.h:  DENSO EtherCAT Motor Controller                #
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
#ifndef DENSO_CONTROL_DENSO_HARDWARE_INTERFACE_
#define DENSO_CONTROL_DENSO_HARDWARE_INTERFACE_

// ROS
#include <ros/ros.h>

// ros_control
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

// denso_client
#include <denso_control/denso_client.h>
#include <ethercat_manager/ethercat_manager.h>
#include <denso_control/denso_robot_hardware_interface.h>

namespace denso_control
{

class DensoHardwareInterface : public hardware_interface::RobotHW
{
private:
  // Node Handles
  ros::NodeHandle nh_; // no namespace
  int             control_mode_;
  unsigned int    n_dof_;
  unsigned int    n_slave_;

  hardware_interface::JointStateInterface    joint_state_interface;
  hardware_interface::PositionJointInterface joint_position_interface;
  hardware_interface::VelocityJointInterface joint_velocity_interface;

  // Ethercat Manager
  ethercat::EtherCatManager *manager_;

  typedef std::vector<EtherCATDensoRobot *> EtherCATDensoRobotContainer;
  EtherCATDensoRobotContainer               denso_robots;

public:
  /**
   * \brief Constructor/Descructor
   */
  DensoHardwareInterface(std::string ifname, int control_mode, ethercat::EtherCatManager *manager,
                         ros::NodeHandle &nh);
  ~DensoHardwareInterface();
  bool read();
  bool read(const ros::Time time, const ros::Duration period);
  void write(const ros::Time time, const ros::Duration period);
  void shutdown();

  ros::Time     getTime();
  ros::Duration getPeriod();

  int  getInputActualValueToStatus(std::vector<std::string> &joint_names,
                                   std::vector<int32> &      position_actual_values,
                                   std::vector<int32> &      velocity_actual_values,
                                   std::vector<int16> &      current_actual_values);
  void getParamFromROS(int joint_no);
};

} // namespace denso_control

#endif // DENSO_CONTROL_DENSO_HARDWARE_INTERFACE_
