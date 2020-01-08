/****************************************************************************
# # denso_hardware_interface.cpp:  DENSO VS050 EtherCAT Hardware Interface  #
# Copyright (c) 2017                                                        #
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

#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <denso_control/denso_hardware_interface.h>
#include <denso_control/denso_robot_hardware_interface.h>
#include <getopt.h>

namespace denso_control
{

#define PULSE_PER_DEG 65536 // 20 bit / 101 reduction

JointData JointControlInterface::read() { return joint; }

void JointControlInterface::write(double cmd)
{
  // ROS_INFO_STREAM("Cmd: " << cmd);
  joint.cmd_ = cmd;
}

void JointControlInterface::write(double cmd, double pos, double vel, double eff)
{
  joint.cmd_ = cmd;
  joint.pos_ = pos;
  joint.vel_ = vel;
  joint.eff_ = eff;
}

void JointControlInterface::write(double pos, double vel, double eff, int32 pos_actual,
                                  int32 vel_actual, int16 eff_actual)
{
  joint.pos_ = pos;
  joint.vel_ = vel;
  joint.eff_ = eff;

  joint.position_actual_value = pos_actual;
  joint.velocity_actual_value = vel_actual;
  joint.current_actual_value = eff_actual;
}

void JointControlInterface::write(double pos, double eff, int32 pos_actual, int16 eff_actual)
{
  joint.pos_ = pos;
  joint.eff_ = eff;

  joint.position_actual_value = pos_actual;
  joint.current_actual_value = eff_actual;
}

EtherCATDensoRobot::EtherCATDensoRobot(ethercat::EtherCatManager *manager, int slave_no, int n_dof,
                                       int                                         control_mode,
                                       hardware_interface::JointStateInterface &   jnt_stat,
                                       hardware_interface::PositionJointInterface &jnt_cmd_pos,
                                       hardware_interface::VelocityJointInterface &jnt_cmd_vel)
    : control_mode_(control_mode), slave_no_(slave_no), n_dof_(n_dof)
{
  std::string name;
  int         eep_man, eep_id, eep_rev;
  int         obits, ibits, state, pdelay, hasdc;
  int         activeports, configadr;

  manager->getStatus(slave_no, name, eep_man, eep_id, eep_rev, obits, ibits, state, pdelay, hasdc,
                     activeports, configadr);

  ROS_INFO("Initialize EtherCATDensoRobot (%d) %s(man:%x, id:%x, rev:%x, port:%x, "
           "addr:%x)",
           slave_no, name.c_str(), eep_man, eep_id, eep_rev, activeports, configadr);
  // EtherCAT

  for (int i = 0; i < n_dof_; i++) {
    if (control_mode_)
      controllers.push_back(new JointControlInterface(slave_no_, i, jnt_stat, jnt_cmd_vel));
    else
      controllers.push_back(new JointControlInterface(slave_no_, i, jnt_stat, jnt_cmd_pos));
  }

  client = new DensoClient(*manager, slave_no, control_mode);

  ROS_INFO("Initialize EtherCATDensoRobot (reset)");

  client->reset();

  // get current position
  ROS_INFO("Initialize EtherCATDensoRobot (readInputs)");
  input = client->readInputs();
  int32 current_position_1 = input.J1_position_actual_value;
  int32 current_position_2 = input.J2_position_actual_value;
  int32 current_position_3 = input.J3_position_actual_value;
  int32 current_position_4 = input.J4_position_actual_value;
  int32 current_position_5 = input.J5_position_actual_value;
  int32 current_position_6 = input.J6_position_actual_value;
  int32 current_position_7 = input.J7_position_actual_value;
  int32 current_position_8 = input.J8_position_actual_value;

  ROS_INFO("                         (Position Actual Value %d %d %d %d %d %d)",
           input.J1_position_actual_value, input.J2_position_actual_value,
           input.J3_position_actual_value, input.J4_position_actual_value,
           input.J5_position_actual_value, input.J6_position_actual_value);

  if (control_mode_) {
    ROS_INFO("                         (Velocity Actual Value %d %d %d %d %d %d)",
             input.J1_velocity_actual_value, input.J2_velocity_actual_value,
             input.J3_velocity_actual_value, input.J4_velocity_actual_value,
             input.J5_velocity_actual_value, input.J6_velocity_actual_value);
  }

  ROS_INFO("                         (Torque Demand Value   %d %d %d %d %d %d)",
           input.J1_torque_demand_value, input.J2_torque_demand_value, input.J3_torque_demand_value,
           input.J4_torque_demand_value, input.J5_torque_demand_value,
           input.J6_torque_demand_value);

  ROS_INFO("Initialize EtherCATDensoRobot (set target)");
  // set target
  memset(&output, 0x00, sizeof(denso_control::RC8Output));

  if (control_mode_) { // Velocity Control
    output.J1_target_velocity = 0;
    output.J2_target_velocity = 0;
    output.J3_target_velocity = 0;
    output.J4_target_velocity = 0;
    output.J5_target_velocity = 0;
    output.J6_target_velocity = 0;
    output.J7_target_velocity = 0;
    output.J8_target_velocity = 0;

  } else { // Position Control
    output.J1_target_position = current_position_1;
    output.J2_target_position = current_position_2;
    output.J3_target_position = current_position_3;
    output.J4_target_position = current_position_4;
    output.J5_target_position = current_position_5;
    output.J6_target_position = current_position_6;
    output.J7_target_position = current_position_7;
    output.J8_target_position = current_position_8;
  }

  double cmd, pos, vel, eff;
  BOOST_FOREACH (JointControlInterface *control, controllers) {
    JointData joint(control->read());
    switch (joint.joint_no_) {
    case 0:
      pos = (double)(current_position_1) / PULSE_PER_DEG;
      break;
    case 1:
      pos = (double)(current_position_2) / PULSE_PER_DEG;
      break;
    case 2:
      pos = (double)(current_position_3) / PULSE_PER_DEG;
      break;
    case 3:
      pos = (double)(current_position_4) / PULSE_PER_DEG;
      break;
    case 4:
      pos = (double)(current_position_5) / PULSE_PER_DEG;
      break;
    case 5:
      pos = (double)(current_position_6) / PULSE_PER_DEG;
      break;
    }

    vel = eff = 0;

    if (control_mode) {
      ROS_WARN("Target velocity = %08x %08x %08x %08x %08x %08x", output.J1_target_velocity,
               output.J2_target_velocity, output.J3_target_velocity, output.J4_target_velocity,
               output.J5_target_velocity, output.J6_target_velocity);

      cmd = vel;
    } else {
      ROS_WARN("Target position = %08x %08x %08x %08x %08x %08x", output.J1_target_position,
               output.J2_target_position, output.J3_target_position, output.J4_target_position,
               output.J5_target_position, output.J6_target_position);

      cmd = pos;
    }
    ROS_INFO_STREAM("WR: J" << joint.name_ << " CMD:" << cmd << " POS:" << pos << " VEL:" << vel
                            << " EFF:" << eff);
    control->write(cmd, pos, vel, eff);
  }

  // servo on
  ROS_INFO("Initialize EtherCATDensoRobot (servoOn)");
  client->servoOn();

  ROS_INFO("Initialize EtherCATDensoRobot done");
}

EtherCATDensoRobot::~EtherCATDensoRobot()
{
  ROS_INFO_STREAM_NAMED("DensoHw", "~EtherCATDensoRobot()");
  shutdown();
  delete (client);
}

void EtherCATDensoRobot::shutdown()
{

  ROS_INFO_STREAM_NAMED("DensoHw", slave_no_ << " shutdown()");

  client->reset();
  client->servoOff();

  controllers.clear();
}

void EtherCATDensoRobot::read()
{
  double pos = 0;
  double eff = 0;
  double vel = 0;
  double cmd = 0;
  int32  pos_actual = 0;
  int16  eff_actual = 0;
  int32  vel_actual = 0;

  input = client->readInputs();
  output = client->readOutputs();
  BOOST_FOREACH (JointControlInterface *control, controllers) {
    JointData joint(control->read());
    switch (joint.joint_no_) {
    case 0:
      pos = (double)(input.J1_position_actual_value) / PULSE_PER_DEG;
      eff = (double)(input.J1_current_actual_value) * 0.1;
      pos_actual = input.J1_position_actual_value;
      eff_actual = input.J1_current_actual_value;
      if (control_mode_) {
        vel = (double)(input.J1_velocity_actual_value) / PULSE_PER_DEG;
        vel_actual = input.J1_velocity_actual_value;
      }
      // ROS_INFO_STREAM("J1: " << pos);
      break;
    case 1:
      pos = (double)(input.J2_position_actual_value) / PULSE_PER_DEG;
      eff = (double)(input.J2_current_actual_value) * 0.1;
      pos_actual = input.J2_position_actual_value;
      eff_actual = input.J2_current_actual_value;
      if (control_mode_) {
        vel = (double)(input.J2_velocity_actual_value) / PULSE_PER_DEG;
        vel_actual = input.J2_velocity_actual_value;
      }
      // ROS_INFO_STREAM("J2: " << pos);
      break;
    case 2:
      pos = (double)(input.J3_position_actual_value) / PULSE_PER_DEG;
      eff = (double)(input.J3_current_actual_value) * 0.1;
      pos_actual = input.J3_position_actual_value;
      eff_actual = input.J3_current_actual_value;
      if (control_mode_) {
        vel = (double)(input.J3_velocity_actual_value) / PULSE_PER_DEG;
        vel_actual = input.J3_velocity_actual_value;
      }
      // ROS_INFO_STREAM("J3: " << pos);
      break;
    case 3:
      pos = (double)(input.J4_position_actual_value) / PULSE_PER_DEG;
      eff = (double)(input.J4_current_actual_value) * 0.1;
      pos_actual = input.J4_position_actual_value;
      eff_actual = input.J4_current_actual_value;
      if (control_mode_) {
        vel = (double)(input.J4_velocity_actual_value) / PULSE_PER_DEG;
        vel_actual = input.J4_velocity_actual_value;
      }
      // ROS_INFO_STREAM("J4: " << pos);
      break;
    case 4:
      pos = (double)(input.J5_position_actual_value) / PULSE_PER_DEG;
      eff = (double)(input.J5_current_actual_value) * 0.1;
      pos_actual = input.J5_position_actual_value;
      eff_actual = input.J5_current_actual_value;
      if (control_mode_) {
        vel = (double)(input.J5_velocity_actual_value) / PULSE_PER_DEG;
        vel_actual = input.J5_velocity_actual_value;
      }
      // ROS_INFO_STREAM("J5: " << pos);
      break;
    case 5:
      pos = (double)(input.J6_position_actual_value) / PULSE_PER_DEG;
      eff = (double)(input.J6_current_actual_value) * 0.1;
      pos_actual = input.J6_position_actual_value;
      eff_actual = input.J6_current_actual_value;
      if (control_mode_) {
        vel = (double)(input.J6_velocity_actual_value) / PULSE_PER_DEG;
        vel_actual = input.J6_velocity_actual_value;
      }
      // ROS_INFO_STREAM("J6: " << pos);
      break;
    }
    if (control_mode_)
      control->write(pos, vel, eff, pos_actual, vel_actual, eff_actual);
    else
      control->write(pos, eff, pos_actual, eff_actual);
  }
}

void EtherCATDensoRobot::writeCmd(Eigen::Matrix<double, 6, 1> qdot)
{
  int i = 0;
  BOOST_FOREACH (JointControlInterface *control, controllers) {
    control->write(qdot[i]);
    i++;
  }
}

void EtherCATDensoRobot::write()
{
  BOOST_FOREACH (JointControlInterface *control, controllers) {
    JointData joint(control->read());
    // ROS_INFO_STREAM("Cmd" << joint.cmd_ << " " << int32_t(joint.cmd_ * PULSE_PER_DEG));
    switch (joint.joint_no_) {
    case 0:
      if (control_mode_)
        output.J1_target_velocity = int32_t(joint.cmd_ * PULSE_PER_DEG);
      else
        output.J1_target_position = int32_t(joint.cmd_ * PULSE_PER_DEG);
      break;
    case 1:
      if (control_mode_)
        output.J2_target_velocity = int32_t(joint.cmd_ * PULSE_PER_DEG);
      else
        output.J2_target_position = int32_t(joint.cmd_ * PULSE_PER_DEG);
      break;
    case 2:
      if (control_mode_)
        output.J3_target_velocity = int32_t(joint.cmd_ * PULSE_PER_DEG);
      else
        output.J3_target_position = int32_t(joint.cmd_ * PULSE_PER_DEG);
      break;
    case 3:
      if (control_mode_)
        output.J4_target_velocity = int32_t(joint.cmd_ * PULSE_PER_DEG);
      else
        output.J4_target_position = int32_t(joint.cmd_ * PULSE_PER_DEG);
      break;
    case 4:
      if (control_mode_)
        output.J5_target_velocity = int32_t(joint.cmd_ * PULSE_PER_DEG);
      else
        output.J5_target_position = int32_t(joint.cmd_ * PULSE_PER_DEG);
      break;
    case 5:
      if (control_mode_)
        output.J6_target_velocity = int32_t(joint.cmd_ * PULSE_PER_DEG);
      else
        output.J6_target_position = int32_t(joint.cmd_ * PULSE_PER_DEG);
      break;
    }
    client->writeOutputs(output);
  }
}

int EtherCATDensoRobot::getInputActualValueToStatus(std::vector<std::string> &joint_names,
                                                    std::vector<int32> &position_actual_values,
                                                    std::vector<int32> &velocity_actual_values,
                                                    std::vector<int16> &current_actual_values)
{
  int         n_dof = 0;
  std::string joint_name;
  int32       position_actual_value;
  int32       velocity_actual_value;
  int16       current_actual_value;
  BOOST_FOREACH (JointControlInterface *control, controllers) {
    control->getInputActualValueToStatus(joint_name, position_actual_value, velocity_actual_value,
                                         current_actual_value);
    joint_names.push_back(joint_name);
    position_actual_values.push_back(position_actual_value);
    velocity_actual_values.push_back(velocity_actual_value);
    current_actual_values.push_back(current_actual_value);
  }

  n_dof++;
  return n_dof;
}

Eigen::Matrix<double, 6, 1> EtherCATDensoRobot::getJointValues()
{
  Eigen::Matrix<double, 6, 1> joint_values;
  int                         n_dof = 0;
  double                      joint_actual_value = 0.0;

  BOOST_FOREACH (JointControlInterface *control, controllers) {
    joint_actual_value = control->getJointValue();
    joint_values[n_dof] = joint_actual_value;
    n_dof++;
  }

  return joint_values;
}

DensoHardwareInterface::DensoHardwareInterface(std::string ifname, int control_mode,
                                               ethercat::EtherCatManager *manager,
                                               ros::NodeHandle &          nh)
    : manager_(manager), control_mode_(control_mode), nh_(nh)
{
  n_dof_ = 6; // VS050
  n_slave_ = manager->getNumClients();

  int i, j;
  for (i = 1; i <= n_slave_; i++) {
    denso_robots.push_back(new EtherCATDensoRobot(manager_, i, n_dof_, control_mode,
                                                  joint_state_interface, joint_position_interface,
                                                  joint_velocity_interface));
  }

  registerInterface(&joint_state_interface);
  if (control_mode_)
    registerInterface(&joint_velocity_interface);
  else
    registerInterface(&joint_position_interface);

  ROS_INFO("Initialize DensoHardwareInterface done");
}

DensoHardwareInterface::~DensoHardwareInterface() { shutdown(); }

void DensoHardwareInterface::getParamFromROS(int joint_no)
{
  std::string joint_name("~joint" + boost::lexical_cast<std::string>(joint_no));
}

void DensoHardwareInterface::shutdown()
{
  BOOST_FOREACH (EtherCATDensoRobot *densoRobot, denso_robots) {
    densoRobot->shutdown();
  }

  if (manager_ != NULL) {
    ROS_INFO_STREAM_NAMED("densoHW", "Delete manager");
    delete (manager_);
  }
  manager_ = NULL;
}

bool DensoHardwareInterface::read()
{
  BOOST_FOREACH (EtherCATDensoRobot *densoRobot, denso_robots) {
    densoRobot->read();
  }
}

bool DensoHardwareInterface::read(const ros::Time time, const ros::Duration period)
{
  BOOST_FOREACH (EtherCATDensoRobot *densoRobot, denso_robots) {
    densoRobot->read();
  }
}

void DensoHardwareInterface::write(const ros::Time time, const ros::Duration period)
{
  BOOST_FOREACH (EtherCATDensoRobot *densoRobot, denso_robots) {
    densoRobot->write();
  }
}

int DensoHardwareInterface::getInputActualValueToStatus(std::vector<std::string> &joint_names,
                                                        std::vector<int32> &position_actual_values,
                                                        std::vector<int32> &velocity_actual_values,
                                                        std::vector<int16> &current_actual_values)
{
  int n_dof = 0;

  BOOST_FOREACH (EtherCATDensoRobot *densoRobot, denso_robots) {
    densoRobot->getInputActualValueToStatus(joint_names, position_actual_values,
                                            velocity_actual_values, current_actual_values);
    n_dof = n_dof + n_dof_;
  }
  return n_dof;
}

inline ros::Time DensoHardwareInterface::getTime() { return ros::Time::now(); }

inline ros::Duration DensoHardwareInterface::getPeriod() { return ros::Duration(0.001); }

} // namespace
