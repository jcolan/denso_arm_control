/******************************************************************************
# mainwindow.h: Denso Arm Robot GUI                                         # #
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

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QLabel>
#include <QMainWindow>
#include <QString>
#include <QTimer>
#include <fstream>

// ROS related
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32MultiArray.h>

// Eigen
#include <Eigen/Dense>

// Project related
#include <denso_arm_control/status.h>
#include <denso_arm_control/robot_command.h>

using namespace Eigen;

namespace Ui
{
class MainWindow;
}

class MainWindow : public QMainWindow
{
  Q_OBJECT

  public:
  explicit MainWindow(ros::NodeHandle &node_handle, QWidget *parent = nullptr);
  ~MainWindow();

  QTimer *ui_update_timer_;

  ros::NodeHandle nh_;

  std::vector<QLabel *> joint_pos_labels_;

  private slots:

  void cyclic_update();

  void on_button_connect_clicked();
  void on_button_motoron_clicked();
  void on_button_slave_clicked();

  int call_robot_srv_();

  void R_connection_on();
  void R_connection_off();
  void R_motor_on();
  void R_motor_off();
  void R_slave_on();
  void R_slave_off();

  void update_joint_state_labels();
  void update_robot_state_label();

  // Callbacks
  void updateJointStateCb(const sensor_msgs::JointState::ConstPtr &msg);
  void updateRobotStateCb(const std_msgs::Int32MultiArray::ConstPtr &msg);

  private:
  Ui::MainWindow *ui;

  std::string prefix_;

  // Subscribers
  ros::Subscriber sub_robot_joint_state_;
  ros::Subscriber sub_robot_state_;

  // Service Client
  ros::ServiceClient               srv_client_cmd_;
  denso_arm_control::robot_command srv_robot_req_;

  // Flags
  bool flag_connected_;
  bool flag_motor_on_;
  bool flag_slave_on_;

  int      robot_status_;
  VectorXd joint_status_;
};

#endif // MAINWINDOW_H
