/******************************************************************************
# denso_client.cpp:  DENSO VS050 EtherCAT SlaveMotion Controller                    #
# Copyright (c) 2017                                                          #
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

#include <denso_control/denso_client.h>
#include <stdio.h>

namespace denso_control
{
static const unsigned SLEEP_TIME_MS = 1; // 1 ms

DensoClient::DensoClient(ethercat::EtherCatManager &manager, int slave_no, int control_mode)
    : manager_(manager), slave_no_(slave_no), control_mode_(control_mode)
{
}

void DensoClient::writeOutputs(const RC8Output &output)
{
  // Default PDO Mapping 408 bit = 51  byte
  uint8_t map[51] = {0}; // array containing all 18 output registers

  if (control_mode_) {
    // Velocity Control
    map[0] = (output.J1_controlword) & 0x00ff;
    map[1] = (output.J1_controlword >> 8) & 0x00ff;
    map[2] = (output.J1_target_velocity) & 0x00ff;
    map[3] = (output.J1_target_velocity >> 8) & 0x00ff;
    map[4] = (output.J1_target_velocity >> 16) & 0x00ff;
    map[5] = (output.J1_target_velocity >> 24) & 0x00ff;

    map[6] = (output.J2_controlword) & 0x00ff;
    map[7] = (output.J2_controlword >> 8) & 0x00ff;
    map[8] = (output.J2_target_velocity) & 0x00ff;
    map[9] = (output.J2_target_velocity >> 8) & 0x00ff;
    map[10] = (output.J2_target_velocity >> 16) & 0x00ff;
    map[11] = (output.J2_target_velocity >> 24) & 0x00ff;

    map[12] = (output.J3_controlword) & 0x00ff;
    map[13] = (output.J3_controlword >> 8) & 0x00ff;
    map[14] = (output.J3_target_velocity) & 0x00ff;
    map[15] = (output.J3_target_velocity >> 8) & 0x00ff;
    map[16] = (output.J3_target_velocity >> 16) & 0x00ff;
    map[17] = (output.J3_target_velocity >> 24) & 0x00ff;

    map[18] = (output.J4_controlword) & 0x00ff;
    map[19] = (output.J4_controlword >> 8) & 0x00ff;
    map[20] = (output.J4_target_velocity) & 0x00ff;
    map[21] = (output.J4_target_velocity >> 8) & 0x00ff;
    map[22] = (output.J4_target_velocity >> 16) & 0x00ff;
    map[23] = (output.J4_target_velocity >> 24) & 0x00ff;

    map[24] = (output.J5_controlword) & 0x00ff;
    map[25] = (output.J5_controlword >> 8) & 0x00ff;
    map[26] = (output.J5_target_velocity) & 0x00ff;
    map[27] = (output.J5_target_velocity >> 8) & 0x00ff;
    map[28] = (output.J5_target_velocity >> 16) & 0x00ff;
    map[29] = (output.J5_target_velocity >> 24) & 0x00ff;

    map[30] = (output.J6_controlword) & 0x00ff;
    map[31] = (output.J6_controlword >> 8) & 0x00ff;
    map[32] = (output.J6_target_velocity) & 0x00ff;
    map[33] = (output.J6_target_velocity >> 8) & 0x00ff;
    map[34] = (output.J6_target_velocity >> 16) & 0x00ff;
    map[35] = (output.J6_target_velocity >> 24) & 0x00ff;

    map[36] = (output.J7_controlword) & 0x00ff;
    map[37] = (output.J7_controlword >> 8) & 0x00ff;
    map[38] = (output.J7_target_velocity) & 0x00ff;
    map[39] = (output.J7_target_velocity >> 8) & 0x00ff;
    map[40] = (output.J7_target_velocity >> 16) & 0x00ff;
    map[41] = (output.J7_target_velocity >> 24) & 0x00ff;

    map[42] = (output.J8_controlword) & 0x00ff;
    map[43] = (output.J8_controlword >> 8) & 0x00ff;
    map[44] = (output.J8_target_velocity) & 0x00ff;
    map[45] = (output.J8_target_velocity >> 8) & 0x00ff;
    map[46] = (output.J8_target_velocity >> 16) & 0x00ff;
    map[47] = (output.J8_target_velocity >> 24) & 0x00ff;

    map[48] = (output.miniIO_out) & 0x00ff;
    map[49] = (output.miniIO_out >> 8) & 0x00ff;
    map[50] = (output.handIO_out) & 0x00ff;

  } else {
    // Position Control (default)
    map[0] = (output.J1_controlword) & 0x00ff;
    map[1] = (output.J1_controlword >> 8) & 0x00ff;
    map[2] = (output.J1_target_position) & 0x00ff;
    map[3] = (output.J1_target_position >> 8) & 0x00ff;
    map[4] = (output.J1_target_position >> 16) & 0x00ff;
    map[5] = (output.J1_target_position >> 24) & 0x00ff;

    map[6] = (output.J2_controlword) & 0x00ff;
    map[7] = (output.J2_controlword >> 8) & 0x00ff;
    map[8] = (output.J2_target_position) & 0x00ff;
    map[9] = (output.J2_target_position >> 8) & 0x00ff;
    map[10] = (output.J2_target_position >> 16) & 0x00ff;
    map[11] = (output.J2_target_position >> 24) & 0x00ff;

    map[12] = (output.J3_controlword) & 0x00ff;
    map[13] = (output.J3_controlword >> 8) & 0x00ff;
    map[14] = (output.J3_target_position) & 0x00ff;
    map[15] = (output.J3_target_position >> 8) & 0x00ff;
    map[16] = (output.J3_target_position >> 16) & 0x00ff;
    map[17] = (output.J3_target_position >> 24) & 0x00ff;

    map[18] = (output.J4_controlword) & 0x00ff;
    map[19] = (output.J4_controlword >> 8) & 0x00ff;
    map[20] = (output.J4_target_position) & 0x00ff;
    map[21] = (output.J4_target_position >> 8) & 0x00ff;
    map[22] = (output.J4_target_position >> 16) & 0x00ff;
    map[23] = (output.J4_target_position >> 24) & 0x00ff;

    map[24] = (output.J5_controlword) & 0x00ff;
    map[25] = (output.J5_controlword >> 8) & 0x00ff;
    map[26] = (output.J5_target_position) & 0x00ff;
    map[27] = (output.J5_target_position >> 8) & 0x00ff;
    map[28] = (output.J5_target_position >> 16) & 0x00ff;
    map[29] = (output.J5_target_position >> 24) & 0x00ff;

    map[30] = (output.J6_controlword) & 0x00ff;
    map[31] = (output.J6_controlword >> 8) & 0x00ff;
    map[32] = (output.J6_target_position) & 0x00ff;
    map[33] = (output.J6_target_position >> 8) & 0x00ff;
    map[34] = (output.J6_target_position >> 16) & 0x00ff;
    map[35] = (output.J6_target_position >> 24) & 0x00ff;

    map[36] = (output.J7_controlword) & 0x00ff;
    map[37] = (output.J7_controlword >> 8) & 0x00ff;
    map[38] = (output.J7_target_position) & 0x00ff;
    map[39] = (output.J7_target_position >> 8) & 0x00ff;
    map[40] = (output.J7_target_position >> 16) & 0x00ff;
    map[41] = (output.J7_target_position >> 24) & 0x00ff;

    map[42] = (output.J8_controlword) & 0x00ff;
    map[43] = (output.J8_controlword >> 8) & 0x00ff;
    map[44] = (output.J8_target_position) & 0x00ff;
    map[45] = (output.J8_target_position >> 8) & 0x00ff;
    map[46] = (output.J8_target_position >> 16) & 0x00ff;
    map[47] = (output.J8_target_position >> 24) & 0x00ff;

    map[48] = (output.miniIO_out) & 0x00ff;
    map[49] = (output.miniIO_out >> 8) & 0x00ff;
    map[50] = (output.handIO_out) & 0x00ff;
  }

  for (unsigned i = 0; i < 51; ++i) {
    manager_.write(slave_no_, i, map[i]);
  }
}

RC8Input DensoClient::readInputs() const
{
  RC8Input input;
  uint8_t  map[84];
  for (unsigned i = 0; i < 84; ++i) {
    map[i] = manager_.readInput(slave_no_, i);
  }

  if (control_mode_) {
    // Velocity control
    input.J1_statusword = *(uint16 *)(map + 0);
    input.J1_position_actual_value = *(int32 *)(map + 2);
    input.J1_velocity_actual_value = *(int32 *)(map + 6);
    input.J1_current_actual_value = *(int16 *)(map + 10);
    input.J1_torque_demand_value = *(int16 *)(map + 12);

    input.J2_statusword = *(uint16 *)(map + 14);
    input.J2_position_actual_value = *(int32 *)(map + 16);
    input.J2_velocity_actual_value = *(int32 *)(map + 20);
    input.J2_current_actual_value = *(int16 *)(map + 24);
    input.J2_torque_demand_value = *(int16 *)(map + 26);

    input.J3_statusword = *(uint16 *)(map + 28);
    input.J3_position_actual_value = *(int32 *)(map + 30);
    input.J3_velocity_actual_value = *(int32 *)(map + 34);
    input.J3_current_actual_value = *(int16 *)(map + 38);
    input.J3_torque_demand_value = *(int16 *)(map + 40);

    input.J4_statusword = *(uint16 *)(map + 42);
    input.J4_position_actual_value = *(int32 *)(map + 44);
    input.J4_velocity_actual_value = *(int32 *)(map + 48);
    input.J4_current_actual_value = *(int16 *)(map + 52);
    input.J4_torque_demand_value = *(int16 *)(map + 54);

    input.J5_statusword = *(uint16 *)(map + 56);
    input.J5_position_actual_value = *(int32 *)(map + 58);
    input.J5_velocity_actual_value = *(int32 *)(map + 62);
    input.J5_current_actual_value = *(int16 *)(map + 66);
    input.J5_torque_demand_value = *(int16 *)(map + 68);

    input.J6_statusword = *(uint16 *)(map + 70);
    input.J6_position_actual_value = *(int32 *)(map + 72);
    input.J6_velocity_actual_value = *(int32 *)(map + 76);
    input.J6_current_actual_value = *(int16 *)(map + 80);
    input.J6_torque_demand_value = *(int16 *)(map + 82);

    input.J7_statusword = *(uint16 *)(map + 84);
    input.J7_position_actual_value = *(int32 *)(map + 86);
    input.J7_velocity_actual_value = *(int32 *)(map + 90);
    input.J7_current_actual_value = *(int16 *)(map + 94);
    input.J7_torque_demand_value = *(int16 *)(map + 96);

    input.J8_statusword = *(uint16 *)(map + 98);
    input.J8_position_actual_value = *(int32 *)(map + 100);
    input.J8_velocity_actual_value = *(int32 *)(map + 104);
    input.J8_current_actual_value = *(int16 *)(map + 108);
    input.J8_torque_demand_value = *(int16 *)(map + 110);

    input.miniIO_in = *(uint16 *)(map + 112);
    input.handIO_in = *(uint8 *)(map + 114);
    input.statusIO = *(uint8 *)(map + 115);

  } else {

    input.J1_statusword = *(uint16 *)(map + 0);
    input.J1_position_actual_value = *(int32 *)(map + 2);
    input.J1_current_actual_value = *(int16 *)(map + 6);
    input.J1_torque_demand_value = *(int16 *)(map + 8);

    input.J2_statusword = *(uint16 *)(map + 10);
    input.J2_position_actual_value = *(int32 *)(map + 12);
    input.J2_current_actual_value = *(int16 *)(map + 16);
    input.J2_torque_demand_value = *(int16 *)(map + 18);

    input.J3_statusword = *(uint16 *)(map + 20);
    input.J3_position_actual_value = *(int32 *)(map + 22);
    input.J3_current_actual_value = *(int16 *)(map + 26);
    input.J3_torque_demand_value = *(int16 *)(map + 28);

    input.J4_statusword = *(uint16 *)(map + 30);
    input.J4_position_actual_value = *(int32 *)(map + 32);
    input.J4_current_actual_value = *(int16 *)(map + 36);
    input.J4_torque_demand_value = *(int16 *)(map + 38);

    input.J5_statusword = *(uint16 *)(map + 40);
    input.J5_position_actual_value = *(int32 *)(map + 42);
    input.J5_current_actual_value = *(int16 *)(map + 46);
    input.J5_torque_demand_value = *(int16 *)(map + 48);

    input.J6_statusword = *(uint16 *)(map + 50);
    input.J6_position_actual_value = *(int32 *)(map + 52);
    input.J6_current_actual_value = *(int16 *)(map + 56);
    input.J6_torque_demand_value = *(int16 *)(map + 58);

    input.J7_statusword = *(uint16 *)(map + 60);
    input.J7_position_actual_value = *(int32 *)(map + 62);
    input.J7_current_actual_value = *(int16 *)(map + 66);
    input.J7_torque_demand_value = *(int16 *)(map + 68);

    input.J8_statusword = *(uint16 *)(map + 70);
    input.J8_position_actual_value = *(int32 *)(map + 72);
    input.J8_current_actual_value = *(int16 *)(map + 76);
    input.J8_torque_demand_value = *(int16 *)(map + 78);

    input.miniIO_in = *(uint16 *)(map + 80);
    input.handIO_in = *(uint8 *)(map + 82);
    input.statusIO = *(uint8 *)(map + 83);
  }

  /* TO DO: IMPEMENT ERROR VERIFICATION*/

  return input;
}

RC8Output DensoClient::readOutputs() const
{

  RC8Output output;

  uint8_t map[51];
  for (unsigned i = 0; i < 51; ++i) {
    map[i] = manager_.readOutput(slave_no_, i);
  }

  if (control_mode_) {
    output.J1_controlword = *(uint16 *)(map + 0);
    output.J1_target_velocity = *(int32 *)(map + 2);

    output.J2_controlword = *(uint16 *)(map + 6);
    output.J2_target_velocity = *(int32 *)(map + 8);

    output.J3_controlword = *(uint16 *)(map + 12);
    output.J3_target_velocity = *(int32 *)(map + 14);

    output.J4_controlword = *(uint16 *)(map + 18);
    output.J4_target_velocity = *(int32 *)(map + 20);

    output.J5_controlword = *(uint16 *)(map + 24);
    output.J5_target_velocity = *(int32 *)(map + 26);

    output.J6_controlword = *(uint16 *)(map + 30);
    output.J6_target_velocity = *(int32 *)(map + 32);

    output.J7_controlword = *(uint16 *)(map + 36);
    output.J7_target_velocity = *(int32 *)(map + 38);

    output.J8_controlword = *(uint16 *)(map + 42);
    output.J8_target_velocity = *(int32 *)(map + 44);

    output.miniIO_out = *(uint16 *)(map + 48);
    output.handIO_out = *(uint8 *)(map + 50);

  } else {

    output.J1_controlword = *(uint16 *)(map + 0);
    output.J1_target_position = *(int32 *)(map + 2);

    output.J2_controlword = *(uint16 *)(map + 6);
    output.J2_target_position = *(int32 *)(map + 8);

    output.J3_controlword = *(uint16 *)(map + 12);
    output.J3_target_position = *(int32 *)(map + 14);

    output.J4_controlword = *(uint16 *)(map + 18);
    output.J4_target_position = *(int32 *)(map + 20);

    output.J5_controlword = *(uint16 *)(map + 24);
    output.J5_target_position = *(int32 *)(map + 26);

    output.J6_controlword = *(uint16 *)(map + 30);
    output.J6_target_position = *(int32 *)(map + 32);

    output.J7_controlword = *(uint16 *)(map + 36);
    output.J7_target_position = *(int32 *)(map + 38);

    output.J8_controlword = *(uint16 *)(map + 42);
    output.J8_target_position = *(int32 *)(map + 44);

    output.miniIO_out = *(uint16 *)(map + 48);
    output.handIO_out = *(uint8 *)(map + 50);
  }

  return output;
}

void DensoClient::reset()
{
  RC8Input input = readInputs();
  // if (input.error_code == 0)
  //     return;

  RC8Output output;

  memset(&output, 0x00, sizeof(RC8Output));
  output.J1_controlword = 0x0080; // fault reset
  output.J2_controlword = 0x0080; // fault reset
  output.J3_controlword = 0x0080; // fault reset
  output.J4_controlword = 0x0080; // fault reset
  output.J5_controlword = 0x0080; // fault reset
  output.J6_controlword = 0x0080; // fault reset
  output.J7_controlword = 0x0080; // fault reset
  output.J8_controlword = 0x0080; // fault reset
  writeOutputs(output);

  /*TO DO: IMPLEMENT ERROR VERIFICATION*/
}

bool DensoClient::isOperationEnabled(const RC8Input input)
{
  if (getPDSStatus(input, 1) == OPERATION_ENABLED && getPDSStatus(input, 2) == OPERATION_ENABLED &&
      getPDSStatus(input, 3) == OPERATION_ENABLED && getPDSStatus(input, 4) == OPERATION_ENABLED &&
      getPDSStatus(input, 5) == OPERATION_ENABLED && getPDSStatus(input, 6) == OPERATION_ENABLED)
    return true;
  else
    return false;
}

bool DensoClient::isSwitchDisabled(const RC8Input input)
{
  if (getPDSStatus(input, 1) == SWITCH_DISABLED && getPDSStatus(input, 2) == SWITCH_DISABLED &&
      getPDSStatus(input, 3) == SWITCH_DISABLED && getPDSStatus(input, 4) == SWITCH_DISABLED &&
      getPDSStatus(input, 5) == SWITCH_DISABLED && getPDSStatus(input, 6) == SWITCH_DISABLED)
    return true;
  else
    return false;
}

void DensoClient::servoOn()
{
  RC8Input input = readInputs();
  // for (int i = 1; i < 7; i++) {
  //   printPDSStatus(input, i);
  // }
  RC8Output output;

  memset(&output, 0x00, sizeof(RC8Output));

  int32 current_position_1 = input.J1_position_actual_value;
  int32 current_position_2 = input.J2_position_actual_value;
  int32 current_position_3 = input.J3_position_actual_value;
  int32 current_position_4 = input.J4_position_actual_value;
  int32 current_position_5 = input.J5_position_actual_value;
  int32 current_position_6 = input.J6_position_actual_value;

  output.J1_target_position = current_position_1;
  output.J2_target_position = current_position_2;
  output.J3_target_position = current_position_3;
  output.J4_target_position = current_position_4;
  output.J5_target_position = current_position_5;
  output.J6_target_position = current_position_6;

  int loop = 0;

  int enable_ = -1;

  while (!isOperationEnabled(input)) {

    for (int i = 1; i < 7; i++) {
      uint16 *controlword;

      switch (i) {
      case 1:
        controlword = &output.J1_controlword;
        break;
      case 2:
        controlword = &output.J2_controlword;
        break;
      case 3:
        controlword = &output.J3_controlword;
        break;
      case 4:
        controlword = &output.J4_controlword;
        break;
      case 5:
        controlword = &output.J5_controlword;
        break;
      case 6:
        controlword = &output.J6_controlword;
        break;
      case 7:
        controlword = &output.J7_controlword;
        break;
      case 8:
        controlword = &output.J8_controlword;
        break;
      }

      switch (getPDSStatus(input, i)) {
      case SWITCH_DISABLED:
        *controlword = 0x0006; // move to ready to switch on
        break;
      case READY_SWITCH:
        *controlword = 0x0007; // move to switched on
        break;
      case SWITCHED_ON:
        *controlword = 0x000f; // move to operation enabled
        break;
      case OPERATION_ENABLED:
        break;
      default:
        printf("unknown status");
        return;
      }
    }

    writeOutputs(output);

    usleep(SLEEP_TIME_MS * 1000);
    input = readInputs();

    if (loop++ % 100 == 1) {
      // for (int i = 1; i < 7; i++) {
      //   printPDSStatus(input, i);
      // }
    }
  }
}

void DensoClient::servoOff()
{
  RC8Input input = readInputs();
  // for (int i = 1; i < 7; i++) {
  //   printPDSStatus(input, i);
  // }
  RC8Output output;
  memset(&output, 0x00, sizeof(RC8Output));

  int loop = 0;
  while (!isSwitchDisabled(input)) {

    for (int i = 1; i < 7; i++) {
      uint16 *controlword;

      switch (i) {
      case 1:
        controlword = &output.J1_controlword;
        break;
      case 2:
        controlword = &output.J2_controlword;
        break;
      case 3:
        controlword = &output.J3_controlword;
        break;
      case 4:
        controlword = &output.J4_controlword;
        break;
      case 5:
        controlword = &output.J5_controlword;
        break;
      case 6:
        controlword = &output.J6_controlword;
        break;
      case 7:
        controlword = &output.J7_controlword;
        break;
      case 8:
        controlword = &output.J8_controlword;
        break;
      }

      switch (getPDSStatus(input, i)) {
      case READY_SWITCH:
        *controlword = 0x0000; // move to ready to switch on
        break;
      case SWITCHED_ON:
        *controlword = 0x0006; // move to operation enabled
        break;
      case OPERATION_ENABLED:
        *controlword = 0x0007; // disable operation
        break;
      default:
        printf("unknown status");
        *controlword = 0x0000; // move to ready to switch on
        return;
      }
    }

    writeOutputs(output);
    usleep(SLEEP_TIME_MS * 1000);
    input = readInputs();
    if (loop++ % 100 == 1) {
      // for (int i = 1; i < 7; i++) {
      //   printPDSStatus(input, i);
      // }
    }
  }
}

PDS_STATUS DensoClient::getPDSStatus(const RC8Input input, int joint) const
{
  uint16 statusword;
  switch (joint) {
  case 1:
    statusword = input.J1_statusword;
    break;
  case 2:
    statusword = input.J2_statusword;
    break;
  case 3:
    statusword = input.J3_statusword;
    break;
  case 4:
    statusword = input.J4_statusword;
    break;
  case 5:
    statusword = input.J5_statusword;
    break;
  case 6:
    statusword = input.J6_statusword;
    break;
  case 7:
    statusword = input.J7_statusword;
    break;
  case 8:
    statusword = input.J8_statusword;
    break;
  }
  if (((statusword)&0x004f) == 0x0000) { // x0xx 0000
    return NOT_READY;
  } else if (((statusword)&0x006f) == 0x0060) { // x11x 0000
    return SWITCH_DISABLED;
  } else if (((statusword)&0x006f) == 0x0021) { // x01x 0001
    return READY_SWITCH;
  } else if (((statusword)&0x006f) == 0x0023) { // x01x 0011
    return SWITCHED_ON;
  } else if (((statusword)&0x006f) == 0x0027) { // x01x 0111
    return OPERATION_ENABLED;
  } else if (((statusword)&0x004f) == 0x000f) { // x0xx 1111
    return FAULT_REACTION;
  } else if (((statusword)&0x004f) == 0x0008) { // x0xx 1000
    return FAULT;
  } else if (((statusword)&0x0010) == 0x0010) { // xx1x xxxx
    return MAIN_POWER_ON;
  } else {
    return UNKNOWN;
  }
}

void DensoClient::printPDSStatus(const RC8Input input, int joint) const
{
  switch (joint) {
  case 1:
    printf("Statusword J1 (6041h): %04x\n ", input.J1_statusword);
    break;
  case 2:
    printf("Statusword J2 (6841h): %04x\n ", input.J2_statusword);
    break;
  case 3:
    printf("Statusword J3 (7041h): %04x\n ", input.J3_statusword);
    break;
  case 4:
    printf("Statusword J4 (7841h): %04x\n ", input.J4_statusword);
    break;
  case 5:
    printf("Statusword J5 (8041h): %04x\n ", input.J5_statusword);
    break;
  case 6:
    printf("Statusword J6 (8841h): %04x\n ", input.J6_statusword);
    break;
  case 7:
    printf("Statusword J7 (9041h): %04x\n ", input.J6_statusword);
    break;
  case 8:
    printf("Statusword J8 (9841h): %04x\n ", input.J6_statusword);
    break;
  }
  switch (getPDSStatus(input, joint)) {
  case NOT_READY:
    printf("Not ready to switch on\n");
    break;
  case SWITCH_DISABLED:
    printf("Switch on disabled\n");
    break;
  case READY_SWITCH:
    printf("Ready to switch on\n");
    break;
  case SWITCHED_ON:
    printf("Switched on\n");
    break;
  case OPERATION_ENABLED:
    printf("Operation enabled\n");
    break;
  case FAULT_REACTION:
    printf("Fault reaction active\n");
    break;
  case FAULT:
    printf("Fault\n");
    break;
  case MAIN_POWER_ON:
    printf("Main Power ON\n");
    break;
  case UNKNOWN:
    // printf("Unknown status %04x\n", input.statusword);
    printf("Unknown status");
    break;
  }
}

} // namespace denso_control
