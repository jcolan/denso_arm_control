/******************************************************************************
# main.cpp:  Main DENSO Arm Control                                         #
# Copyright (c) 2020                                                          #
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
#   Author: Jacinto Colan, email: colan@robo.mein.nagoya-u.ac.jp           #
#                                                                             #
# ###########################################################################*/

#include <denso_arm_control/denso_arm_control.h>

#include <signal.h>
#include <thread>

#define DEBUG_ENABLED false

using namespace denso_nu;

bool kill_this_process = false;

void SigIntHandler(int signal)
{
  kill_this_process = true;
  ROS_WARN_STREAM("SHUTDOWN SIGNAL RECEIVED");
}

int main(int argc, char **argv)
{
  // Ros related
  ros::init(argc, argv, "denso_arm_control");
  ros::NodeHandle node_handle;
  signal(SIGINT, SigIntHandler);

  DensoArmControl dc(node_handle, &kill_this_process);

  RealtimeSchedulerInterface::activateRRScheduler(99);
  RealtimeSchedulerInterface::display_thread_sched_attr(
      "Trying to upgrade to real-time SCHED_DEADLINE scheduler...");

  ros::AsyncSpinner spinner(2);
  spinner.start();

  std::thread t(std::bind(&DensoArmControl::control_loop, &dc));

  t.join();
  spinner.stop();

  return 0;
}
