/******************************************************************************
# denso_arm_control.h:  DENSO Arm Control                               #
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

#ifndef DENSO_ARM_CONTROL_H
#define DENSO_ARM_CONTROL_H

#include <denso_arm_control/denso_arm_hw.h>

#include <pthread.h>
#include <time.h>

// ROS related
#include <ros/ros.h>

#include <denso_communication/DensoRobot.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Wrench.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <denso_arm_control/status.h>

#include <denso_arm_control/robot_command.h>

// RT
#include <realtime_scheduler_interface/realtime_clock.h>
#include <realtime_scheduler_interface/realtime_scheduler_interface.h>

// C++ Std 11
#include <pwd.h>
#include <sys/types.h>
#include <unistd.h>

#include <cmath>

using namespace Eigen;
using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;
using joint_limits_interface::SoftJointLimits;
namespace denso_nu
{

class DensoArmControl : public DensoArmHW
{
  const int NSEC2SEC = 1000000000;
  const int NSEC2SEC_D = 1000000000.0;

  const double DEG2RAD = (M_PI) / 180;
  const double RAD2DEG = 180 / (M_PI);

  /* TODO */
  // Communication modes
  // enum
  // {
  //   BCAP_MODE = 0,
  //   ETHERCAT,
  // };

  // Slave modes
  // enum
  // {
  //   JOINT_MODE = 0,
  //   JOINT_SMOOTH_MODE,
  //   CART3D_MODE,
  //   CART3D_SMOOTH_MODE,
  //   CART6D_MODE,
  //   CART6D_SMOOTH_MODE,

  // };

  // Robot states
  // enum
  // {
  //   DISCONNECTED = 0,
  //   CONNECTED,
  //   MOTOR_ON,
  //   SLAVE_ON,
  // };

  struct ArmJointLimits
  {
    std::string name;
    bool        has_position_limits = {false};
    double      min_pos = {0.0};
    double      max_pos = {0.0};
    bool        has_velocity_limits = {false};
    double      max_vel = {0.0};
  };

  public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Constructor
  DensoArmControl(ros::NodeHandle &node_handle, bool *kill_this_node);
  ~DensoArmControl();

  // Callbacks

  bool
  DensoArmControlRobotCommandCb(denso_arm_control::robot_command::Request & request,
                                denso_arm_control::robot_command::Response &response);

  // Functions
  int control_loop();

  void publishRobotState();

  // Change Modes
  bool connect();
  bool disconnect();
  void MotorOn();
  void MotorOff();
  bool motor_on();
  bool motor_off();
  bool slave_mode_on();
  bool slave_mode_off();

  // Callbacks
  void updateSimJointStateCb(const sensor_msgs::JointState::ConstPtr &msg);

  // bool getArmJointLimits(const std::string &joint_name, const ros::NodeHandle &nh,
  //                        DensoArmControl::ArmJointLimits &limits);

  void safeSleepSeconds(double seconds);
  void blockingSleepSeconds(double seconds);

  void sendCmdRobot();
  void sendCmdSim();

  // Ros control
  void read();
  void write();
  bool read(const ros::Time time, const ros::Duration period);
  void write(const ros::Time time, const ros::Duration period);

  private:
  // ROS
  ros::NodeHandle nh_;

  // Robot variables
  std::string prefix_;
  int         robot_id;
  int         n_joints_;

  // Suscribers
  ros::Subscriber sub_sim_joint_state_;

  // Publishers
  ros::Publisher            pub_state_;
  std_msgs::Int32MultiArray pub_state_msg_;

  ros::Publisher          pub_sim_joint_cmd_;
  sensor_msgs::JointState pub_sim_joint_cmd_msg_;

  // Services
  ros::ServiceServer service_server_robot_command_;

  // Joint variables
  VectorXd des_joint_pos_;
  VectorXd act_joint_pos_;
  VectorXd sim_joint_pos_;

  // RT
  RealtimeClock real_time_clock_;
  int           cycle_t_ns_;
  double        cycle_t_ns_d_;
  double        cycle_t_;

  // Current state of this process
  int state_;

  // b-Cap error storage
  int bcap_error_code_;

  // Bool to kill loops
  bool *kill_this_node_;

  // Denso comm
  DensoRobot robot_;

  // Flags
  bool flag_buffer_full;
  int  buffer_full_counter;

  // Asynchronous spinner
  ros::AsyncSpinner *spinner;

  // Mutex
  boost::mutex m_mtxAct;

  // Joint limits vector
  std::vector<ArmJointLimits> joint_limits_;

  // Joint names
  std::vector<std::string> joint_names_;

  // Simulator
  bool robot_sim_;

  // Time variables
  ros::Time     timestamp_k_;
  ros::Time     timestamp_km1_;
  ros::Duration period_;
};
} // namespace denso_nu

#endif
