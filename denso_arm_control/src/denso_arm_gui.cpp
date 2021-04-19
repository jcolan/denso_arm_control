/******************************************************************************
# denso_arm_gui.cpp: Denso Arm GUI                                          # #
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
#   Author: Jacinto E. Colan, email: colan@robo.mein.nagoya-u.ac.jp           #
#                                                                             #
# ###########################################################################*/

#include "qt/mainwindow.h"
#include <QApplication>
#include <signal.h>

QApplication *q_application;

bool kill_process = false;
void SigIntHandler(int signal)
{
  kill_process = true;
  ROS_WARN_STREAM("GUI: SHUTDOWN SIGNAL RECEIVED");
  q_application->quit();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "denso_arm_gui");
  ros::NodeHandle node_handle;
  signal(SIGINT, SigIntHandler);

  QApplication a(argc, argv);

  q_application = &a;

  MainWindow w(node_handle);

  w.show();

  return a.exec();
}
