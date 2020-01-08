/****************************************************************************
# denso_client.h:  DENSO VS050 EtherCAT Controller                          #
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

#ifndef DENSO_CLIENT_H
#define DENSO_CLIENT_H

#include <ethercat_manager/ethercat_manager.h>
#include <soem/osal.h>

// Forward declaration of EtherCatManager
namespace ethercat
{
class EtherCatManager;
}

namespace denso_control
{

/**
 * \brief This class provides a client for the EtherCAT manager object that
 *        can translate robot input/output messages and translate them to
 *        the underlying IO Map.
 */

// Default PDO maping
// Position control, Velocity Control

typedef struct {
  // input
  uint16 J1_statusword;            // 6041h : Statusword
  int32  J1_position_actual_value; // 6064h : Position actual value
  int32  J1_velocity_actual_value; // 606Ch : Velocity actual value
  int16  J1_current_actual_value;  // 6078h : Current actual value
  int16  J1_torque_demand_value;   // 6074h : Torque Demand Value

  uint16 J2_statusword;            // 6841h : Statusword
  int32  J2_position_actual_value; // 6864h : Position actual value
  int32  J2_velocity_actual_value; // 686Ch : Velocity actual value
  int16  J2_current_actual_value;  // 6878h : Current actual value
  int16  J2_torque_demand_value;   // 6874h : Torque Demand Value

  uint16 J3_statusword;            // 7041h : Statusword
  int32  J3_position_actual_value; // 7064h : Position actual value
  int32  J3_velocity_actual_value; // 706Ch : Velocity actual value
  int16  J3_current_actual_value;  // 7078h : Current actual value
  int16  J3_torque_demand_value;   // 7074h : Torque Demand Value

  uint16 J4_statusword;            // 7841h : Statusword
  int32  J4_position_actual_value; // 7864h : Position actual value
  int32  J4_velocity_actual_value; // 786Ch : Velocity actual value
  int16  J4_current_actual_value;  // 7878h : Current actual value
  int16  J4_torque_demand_value;   // 7874h : Torque Demand Value

  uint16 J5_statusword;            // 8041h : Statusword
  int32  J5_position_actual_value; // 8064h : Position actual value
  int32  J5_velocity_actual_value; // 806Ch : Velocity actual value
  int16  J5_current_actual_value;  // 8078h : Current actual value
  int16  J5_torque_demand_value;   // 8074h : Torque Demand Value

  uint16 J6_statusword;            // 8841h : Statusword
  int32  J6_position_actual_value; // 8864h : Position actual value
  int32  J6_velocity_actual_value; // 886Ch : Velocity actual value
  int16  J6_current_actual_value;  // 8878h : Current actual value
  int16  J6_torque_demand_value;   // 8874h : Torque Demand Value

  uint16 J7_statusword;            // 9041h : Statusword
  int32  J7_position_actual_value; // 9064h : Position actual value
  int32  J7_velocity_actual_value; // 906Ch : Velocity actual value
  int16  J7_current_actual_value;  // 9078h : Current actual value
  int16  J7_torque_demand_value;   // 9074h : Torque Demand Value

  uint16 J8_statusword;            // 9841h : Statusword
  int32  J8_position_actual_value; // 9864h : Position actual value
  int32  J8_velocity_actual_value; // 986Ch : Velocity actual value
  int16  J8_current_actual_value;  // 9878h : Current actual value
  int16  J8_torque_demand_value;   // 9874h : Torque Demand Value

  uint16 miniIO_in; // 3000h : Mini IO
  uint8  handIO_in; // 3001h : Hand IO
  uint8  statusIO;  // 3002h : Status IO

} RC8Input;

typedef struct {
  uint16 J1_controlword;     // 6040h : Controlword
  int32  J1_target_position; // 607Ah : Target Position
  int32  J1_target_velocity; // 60FFh : Target Velocity

  uint16 J2_controlword;     // 6840h : Controlword
  int32  J2_target_position; // 687Ah : Target Position
  int32  J2_target_velocity; // 68FFh : Target Velocity

  uint16 J3_controlword;     // 7040h : Controlword
  int32  J3_target_position; // 707Ah : Target Position
  int32  J3_target_velocity; // 70FFh : Target Velocity

  uint16 J4_controlword;     // 7840h : Controlword
  int32  J4_target_position; // 787Ah : Target Position
  int32  J4_target_velocity; // 78FFh : Target Velocity

  uint16 J5_controlword;     // 8040h : Controlword
  int32  J5_target_position; // 807Ah : Target Position
  int32  J5_target_velocity; // 80FFh : Target Velocity

  uint16 J6_controlword;     // 8840h : Controlword
  int32  J6_target_position; // 887Ah : Target Position
  int32  J6_target_velocity; // 88FFh : Target Velocity

  uint16 J7_controlword;     // 9040h : Controlword
  int32  J7_target_position; // 907Ah : Target Position
  int32  J7_target_velocity; // 90FFh : Target Velocity

  uint16 J8_controlword;     // 9840h : Controlword
  int32  J8_target_position; // 987Ah : Target Position
  int32  J8_target_velocity; // 98FFh : Target Velocity

  uint16 miniIO_out; // 3010h : Mini IO
  uint8  handIO_out; // 3011h : Hand IO

} RC8Output;

typedef enum {
  NOT_READY,
  SWITCH_DISABLED,
  READY_SWITCH,
  SWITCHED_ON,
  OPERATION_ENABLED,
  FAULT_REACTION,
  FAULT,
  MAIN_POWER_ON,
  UNKNOWN,
} PDS_STATUS; // Statusword

typedef enum {
  CYCLIC_SYNCHRONOUS_POSITION_MODE,
  CYCLIC_SYNCHRONOUS_VELOCITY_MODE,
} PDS_OPERATION; // Mode of operation

typedef enum {
  SHUTDOWN,
  SWITCH_ON,
  ENABLE_OPERATION,
  DISABLE_VOLTAGE,
  DISABLE_OPERATION,
  FAULT_RESET,
} PDS_CONTROL; // Controlworld

class DensoClient
{
public:
  /**
   * \brief Constructs a control interface to a DENSO RC8 on
   *        the given ethercat network and the given slave_no.
   *
   * @param[in] manager The interface to an EtherCAT network that the RC8 Slave
   *                    is connected to.
   *
   * @param[in] slave_no The slave number of the RC8 on the EtherCAT network
   *                     (>= 1)
   *
   * @param[in] control_mode Control mode of the RC8 on the EtherCAT Slave mode
   *                          [0] Position Control
   *                          [1] Velocity Control
   */
  DensoClient(ethercat::EtherCatManager &manager, int slave_no, int control_mode);

  /**
   * \brief Write the given set of control flags to the memory of the controller
   *
   * @param[in] output The set of output-register values to write to the gripper
   */
  void writeOutputs(const RC8Output &output);

  /**
   * \brief Reads set of input-register values from the controller.
   * \return The AC servo input registers as read from the controller IOMap
   */
  RC8Input readInputs() const;

  /**
   * \brief Reads set of output-register values from the controller.
   * \return The AC servo output registers as read from the controller IOMap
   */
  RC8Output readOutputs() const;

  /**
   * \brief Reset alarm
   * \return void
   */
  void reset();

  /**
   * \brief Send servo on sequence to the controller
   * \return void
   * @param[in] joint The joint number [1-6] to be reset
   */
  void servoOn();

  /**
   * \brief Send servo off sequence to the controller
   * \return void
   * @param[in] joint The joint number [1-6] to be turned on
   */
  void servoOff();

  /**
   * \brief print status from input data
   */
  void printPDSStatus(const RC8Input input, int joint) const;

  /**
   * \brief Verify Operation is enabled
   */
  bool isOperationEnabled(const RC8Input input);

  /**
   * \brief Verify Switch is disabled
   */
  bool isSwitchDisabled(const RC8Input input);

private:
  /**
   * \brief get status from input data
   * \return status
   */
  PDS_STATUS getPDSStatus(const RC8Input input, int joint) const;

  ethercat::EtherCatManager &manager_;

  const int slave_no_;
  const int control_mode_;
};

// /** TO DO
//  * \brief Table of error code and text
//  */
// const struct {
//   unsigned int code;
//   const char * text;
// } error_map[] = {
//     {11, "Control power supply under-voltage protection"},
//     {12, "Over voltage protection"},
// };

} // namespace denso_control

#endif
