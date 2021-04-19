/******************************************************************************
# robot.cpp:  DENSO Arm Control                               #
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

#include <ros/ros.h>
#include <ros/package.h>

#define DEBUG_ENABLED false

namespace denso_nu
{

// Constructor
DensoArmControl::DensoArmControl(ros::NodeHandle &node_handle, bool *kill_this_node)
{
  nh_ = node_handle;

  //* ROS PARAMETER SERVER
  if (!nh_.getParam("prefix", prefix_))
  {
    prefix_ = "";
  }

  // Robot Identifier
  if (nh_.getParam(prefix_ + "/robot_id", robot_id))
  {
    ROS_INFO_STREAM("Robot ID obtained from ROS Parameter server as: " << robot_id);
  }
  else
  {
    ROS_INFO_STREAM("No ID information in parameter server, using default: [0]");
    robot_id = 0;
  }

  // Robot IP Address
  std::string robot_ip_address;
  if (nh_.getParam(prefix_ + "/robot_ip_address", robot_ip_address))
  {
    ROS_INFO_STREAM(
        "Ip address obtained from ROS Parameter server as: " << robot_ip_address);
    robot_ = DensoRobot(robot_ip_address, 5007, kill_this_node_);
  }
  else
  {
    ROS_INFO_STREAM("No ip information in parameter server, using default 192.168.0.1");
    robot_ = DensoRobot(std::string("192.168.0.1"), 5007, kill_this_node_);
  }

  // Thread Sampling Time
  if (nh_.getParam(prefix_ + "/thread_sampling_time_nsec", cycle_t_ns_))
  {
    ROS_INFO_STREAM(
        "Thread sampling time obtained from ROS Parameter server as: " << cycle_t_ns_);
  }
  else
  {
    ROS_INFO_STREAM("No sampling time information in parameter server, using "
                    "default 2ms");
    cycle_t_ns_ = 2000000;
  }

  // Robot simulaton
  if (nh_.getParam(prefix_ + "/robot_sim_", robot_sim_))
  {
    ROS_INFO_STREAM("Robot simulator: " << robot_sim_);
  }
  else
  {
    ROS_INFO_STREAM("No sampling robot simulator option set in parameter server, using "
                    "default: Robot simulator on");
    robot_sim_ = true;
  }

  // Joint names
  if (!nh_.getParam(prefix_ + "/position_controller/joints", joint_names_))
  {
    joint_names_.clear();
  }

  n_joints_ = joint_names_.size();
  ROS_INFO_STREAM("Number of Joints:" << n_joints_);

  // Subscribers
  if (robot_sim_)
  {
    sub_sim_joint_state_ = nh_.subscribe(prefix_ + "/sim/joint/state", 1,
                                         &DensoArmControl::updateSimJointStateCb, this);
  }

  // Publishers
  pub_state_ = nh_.advertise<std_msgs::Int32MultiArray>(prefix_ + "/robot/state", 1);
  pub_sim_joint_cmd_ =
      nh_.advertise<sensor_msgs::JointState>(prefix_ + "/sim/joint/cmd", 1);

  // Services
  service_server_robot_command_ =
      nh_.advertiseService(prefix_ + "/robot/robot_command",
                           &DensoArmControl::DensoArmControlRobotCommandCb, this);

  // Resize publishing messages
  pub_sim_joint_cmd_msg_.name.resize(n_joints_);
  pub_sim_joint_cmd_msg_.position.resize(n_joints_);
  pub_sim_joint_cmd_msg_.velocity.resize(n_joints_);
  pub_sim_joint_cmd_msg_.effort.resize(n_joints_);

  // Control loop timekeeping
  cycle_t_ns_d_ = (double)cycle_t_ns_;
  cycle_t_ = cycle_t_ns_d_ / NSEC2SEC_D;

  // Real time clock intialization
  real_time_clock_ = RealtimeClock(cycle_t_ns_);

  kill_this_node_ = kill_this_node;

  // Flags
  flag_buffer_full = false;

  buffer_full_counter = 0;

  // Command frame
  des_joint_pos_ = VectorXd::Zero(6);
  act_joint_pos_ = VectorXd::Zero(6);
  sim_joint_pos_ = VectorXd::Zero(6);

  // Initialize ros messages
  pub_state_msg_.data.resize(1);

  // Temporarily
  state_ = R_UNINITIALIZED;

  // Resize vectors
  joint_position_.resize(n_joints_);
  joint_velocity_.resize(n_joints_);
  joint_effort_.resize(n_joints_);
  joint_position_cmd_.resize(n_joints_);
  joint_velocity_cmd_.resize(n_joints_);
  joint_effort_cmd_.resize(n_joints_);

  jointInterfaceSetZero();

  // Initialize controllers
  for (int i = 0; i < n_joints_; i++)
  {
    // Create Joint state interface
    JointStateHandle jointStateHandle(joint_names_[i], &joint_position_[i],
                                      &joint_velocity_[i], &joint_effort_[i]);
    joint_state_interface_.registerHandle(jointStateHandle);

    // Create Position Joint interface
    JointHandle jointPositionHandle(jointStateHandle, &joint_position_cmd_[i]);
    position_interface_.registerHandle(jointPositionHandle);

    // Create Velocity Joint interface
    JointHandle jointVelocityHandle(jointStateHandle, &joint_velocity_cmd_[i]);
    velocity_interface_.registerHandle(jointVelocityHandle);

    // Create Effort joint interface
    JointHandle jointEffortHandle(jointStateHandle, &joint_effort_cmd_[i]);
    effort_interface_.registerHandle(jointEffortHandle);

    JointLimits limits;
    getJointLimits(joint_names_[i], nh_, limits);

    joint_limits_interface::SoftJointLimits               softLimits;
    joint_limits_interface::PositionJointSoftLimitsHandle jointLimitsHandle(
        jointEffortHandle, limits, softLimits);
    position_limits_interface_.registerHandle(jointLimitsHandle);
  }

  registerInterface(&joint_state_interface_);
  registerInterface(&position_interface_);
  registerInterface(&velocity_interface_);
  registerInterface(&effort_interface_);
  registerInterface(&position_limits_interface_);

  timestamp_k_ = ros::Time::now();
  timestamp_km1_ = ros::Time::now();

  ROS_INFO_STREAM("Denso controller succesfully loaded");
}

DensoArmControl::~DensoArmControl() {}

// Callbacks
void DensoArmControl::updateSimJointStateCb(const sensor_msgs::JointState::ConstPtr &msg)
{
  boost::mutex::scoped_lock lockAct(m_mtxAct);
  sim_joint_pos_ = VectorXd::Map(&msg->position[0], 6) * RAD2DEG;
  return;
}

// bool DensoArmControl::getArmJointLimits(const std::string &    joint_name,
//                                         const ros::NodeHandle &nh, ArmJointLimits
//                                         &limits)
// {
//   // Node handle scoped where the joint limits are defined
//   ros::NodeHandle limits_nh;
//   try
//   {
//     const std::string limits_namespace = "joint_limits/" + joint_name;
//     if (!nh.hasParam(limits_namespace))
//     {
//       ROS_WARN_STREAM("No joint limits specification found for joint '"
//                       << joint_name << "' in the parameter server (namespace "
//                       << nh.getNamespace() + "/" + limits_namespace << ").");
//       return false;
//     }
//     limits_nh = ros::NodeHandle(nh, limits_namespace);
//   }
//   catch (const ros::InvalidNameException &ex)
//   {
//     ROS_ERROR_STREAM(ex.what());
//     return false;
//   }

//   // Position limits
//   bool has_position_limits = false;
//   if (limits_nh.getParam("has_position_limits", has_position_limits))
//   {
//     if (!has_position_limits)
//     {
//       limits.has_position_limits = false;
//     }
//     double min_pos, max_pos;
//     if (has_position_limits && limits_nh.getParam("min_position", min_pos) &&
//         limits_nh.getParam("max_position", max_pos))
//     {
//       limits.has_position_limits = true;
//       limits.min_pos = min_pos;
//       limits.max_pos = max_pos;
//     }
//   }

//   // Velocity limits
//   bool has_velocity_limits = false;
//   if (limits_nh.getParam("has_velocity_limits", has_velocity_limits))
//   {
//     if (!has_velocity_limits)
//     {
//       limits.has_velocity_limits = false;
//     }
//     double max_vel;
//     if (has_velocity_limits && limits_nh.getParam("max_velocity", max_vel))
//     {
//       limits.has_velocity_limits = true;
//       limits.max_vel = max_vel;
//     }
//   }

//   return true;
// }

int DensoArmControl::control_loop()
{
  controller_manager::ControllerManager cm(this);

  real_time_clock_.init();

  while (not(*kill_this_node_))
  {
    timestamp_k_ = ros::Time::now();
    period_ = timestamp_k_ - timestamp_km1_;
    timestamp_km1_ = timestamp_k_;

    if (state_ >= R_CONNECTED)
    {
      read();
      cm.update(timestamp_k_, period_);
      if (state_ == R_SLAVE)
        write();
    }

    real_time_clock_.updateAndSleep();
    publishRobotState();
    // End while not kill this node
  }

  // }
  // Just to be safe, call this two functions in any circumstance.
  ROS_WARN_STREAM("Robot_node will try to turn off motors even "
                  "if they are already off.");
  motor_off();
  disconnect();

  return 0;
}

// Ros Control functions
void DensoArmControl::read()
{
  if (!robot_sim_)
  {
    if (state_ >= R_CONNECTED && state_ < R_SLAVE_ON)
    {
      act_joint_pos_ = robot_.GetJointPositions(&bcap_error_code_);
    }
  }
  else
  {
    act_joint_pos_ = sim_joint_pos_;
  }
  joint_position_ = act_joint_pos_;
}

void DensoArmControl::write()
{
  des_joint_pos_ = joint_position_cmd_;
  if (!robot_sim_)
  {
    sendCmdRobot();
  }
  else
  {
    sendCmdSim();
  }
}

//* Robot Command Functions
bool DensoArmControl::connect()
{
  if (!robot_sim_)
  {
    ROS_INFO_STREAM("DensoArmControl::ConnectCallback called " << state_);
    if (state_ < R_CONNECTED)
    {
      int res = robot_.Connect();
      if (res == -1)
        return false;

      state_ = R_CONNECTED;
      ROS_INFO_STREAM("Robot State change to: CONNECTED");
    }

    /*Get current joint values*/
    int i = 0; // An upper limit of tries so that it doesn't get locked here if
               // something goes wrong.

    while (act_joint_pos_.norm() < 1 && not(*kill_this_node_))
    {
      act_joint_pos_ = robot_.GetJointPositions(&bcap_error_code_);
      safeSleepSeconds(0.1);
      if (i > 1000)
      {
        ROS_ERROR_STREAM("Timeout reached when getting robot's initial posture.");
      }
      i++;
    }

    des_joint_pos_ = act_joint_pos_;
    joint_position_cmd_ = act_joint_pos_;
    ROS_INFO_STREAM("Current joint positions are: " << act_joint_pos_.transpose());
  }
  else
  {
    act_joint_pos_ = sim_joint_pos_;
    des_joint_pos_ = sim_joint_pos_;
    joint_position_cmd_ = sim_joint_pos_;

    ROS_INFO_STREAM("Robot State change to: CONNECTED");
    ROS_INFO_STREAM("Current joint positions are: " << act_joint_pos_.transpose());

    state_ = R_CONNECTED;
  }

  return true;
}

bool DensoArmControl::disconnect()
{
  if (!robot_sim_)
  {
    ROS_INFO_STREAM("DensoArmControl::DisconnectCallback called" << state_);
    if (state_ == R_CONNECTED)
    {
      robot_.Disconnect();
    }
    else if (state_ > R_CONNECTED)
    {
      robot_.MotorOff();
      robot_.Disconnect();
    }
  }
  ROS_INFO_STREAM("Robot State change to: UNINITIALIZED");
  state_ = R_UNINITIALIZED;

  return true;
}

bool DensoArmControl::motor_on()
{
  if (!robot_sim_)
  {
    if (state_ == R_CONNECTED)
    {
      MotorOn();
    }
  }

  ROS_INFO_STREAM("Robot State change to: R_READY");
  state_ = R_READY;
  return true;
}

bool DensoArmControl::motor_off()
{
  if (!robot_sim_)
  {
    if (state_ < R_READY)
      ROS_WARN_STREAM("Motors shouldnt be ON but will try to turn them off anyways...");
    MotorOff();
  }
  state_ = R_CONNECTED;
  ROS_INFO_STREAM("Robot State change to: R_CONNECTED");
  return true;
}

bool DensoArmControl::slave_mode_on()
{
  if (!robot_sim_)
  {
    ROS_INFO_STREAM("Calling SetRecvFormat");
    robot_.SetRecvFormat(0x002); // Receive Joint position (see b-CAP Guide RC8 p. 33)

    ROS_INFO_STREAM("Calling SlaveModeOn");
    int res = robot_.SlaveModeOn(
        0x102); // Joint Command Mode 1-Asynchronous (See b-CAP Guide RC8 p.40)
    if (res == -1)
      return false;
  }
  ROS_INFO_STREAM("Robot State change to: R_SLAVE");
  state_ = R_SLAVE;
  return true;
}

bool DensoArmControl::slave_mode_off()
{
  if (!robot_sim_)
  {
    robot_.SlaveModeOff();
    blockingSleepSeconds(2.);
  }
  ROS_INFO_STREAM("Robot State change to: R_READY");
  state_ = R_READY;
  return true;
}

void DensoArmControl::MotorOn()
{
  ROS_INFO_STREAM("Calling MotorOn");
  int res = robot_.MotorOn();
  if (res == -1)
    return;
  safeSleepSeconds(1.);
  ROS_INFO_STREAM("Calling SetSpeed");
  robot_.SetSpeed(50., 30., 30.);

  safeSleepSeconds(1.);
}

void DensoArmControl::MotorOff()
{
  robot_.SetSpeed(1, 1, 1);
  blockingSleepSeconds(1.);
  robot_.MotorOff();
  blockingSleepSeconds(1.);
}

void DensoArmControl::safeSleepSeconds(double seconds)
{
  for (int i = 0; i < int(seconds / cycle_t_) && not(*kill_this_node_); i++)
  {
    real_time_clock_.updateAndSleep();
  }
}

void DensoArmControl::blockingSleepSeconds(double seconds)
{
  for (int i = 0; i < int(seconds / cycle_t_); i++)
  {
    real_time_clock_.updateAndSleep();
  }
}

//* Callbacks

// Service Server
bool DensoArmControl::DensoArmControlRobotCommandCb(
    denso_arm_control::robot_command::Request & request,
    denso_arm_control::robot_command::Response &response)
{
  bool res;
  ROS_INFO_STREAM("Robot Command Called with request: " << request.message);
  if (request.message == "connect")
    res = connect();
  else if (request.message == "disconnect")
    res = disconnect();
  else if (request.message == "motor_on")
    res = motor_on();
  else if (request.message == "motor_off")
    res = motor_off();
  else if (request.message == "slave_on")
    res = slave_mode_on();
  else if (request.message == "slave_off")
    res = slave_mode_off();
  else
    return false;

  return res;
}

//* Auxiliar Functions

void DensoArmControl::sendCmdRobot()
{
  if (!flag_buffer_full && buffer_full_counter == 0)
  {
    HRESULT hr = robot_.SetJointPositions(des_joint_pos_);
    if (hr == S_OK)
    {
      act_joint_pos_ = robot_.GetJointPositionsBuffer();
    }
    else
    {
      ROS_INFO_STREAM("Actual Joint pos: " << act_joint_pos_);
      ROS_INFO_STREAM("Desired Joint pos: " << des_joint_pos_);

      if (hr == 0xF200501)
      {
        ROS_WARN_STREAM("Buffer full");
        flag_buffer_full = true;
        buffer_full_counter = 2;
      }
      else if (hr == 0x83201483)
      {
        ROS_WARN_STREAM("Buffer overflow");
      }
      else if (hr == 0x83500121)
      {
        ROS_ERROR_STREAM("Emergency Stop. Check Pendant for details.");
        if (state_ == R_CONNECTED)
        {
          ROS_WARN_STREAM("Disconnecting Robot");
          disconnect();
          state_ = R_UNINITIALIZED;
        }
        else if (state_ > R_CONNECTED)
        {
          ROS_WARN_STREAM("Exitting Slave Mode");
          slave_mode_off();
          ROS_WARN_STREAM("Stoppping Robot motors");
          motor_off();
          ROS_WARN_STREAM("Disconnecting Robot");
          disconnect();
          state_ = R_UNINITIALIZED;
        }
      }
      else
      {
        ROS_ERROR("Error sending Position Command. Error: 0x%X", hr);
        ROS_ERROR_COND(hr == 0x84204051, "J1 command speed limit over");
        ROS_ERROR_COND(hr == 0x84204052, "J2 command speed limit over");
        ROS_ERROR_COND(hr == 0x84204053, "J3 command speed limit over");
        ROS_ERROR_COND(hr == 0x84204054, "J4 command speed limit over");
        ROS_ERROR_COND(hr == 0x84204055, "J5 command speed limit over");
        ROS_ERROR_COND(hr == 0x84204056, "J6 command speed limit over");
        ROS_ERROR_COND(hr == 0x84204041, "J1 command accel limit over");
        ROS_ERROR_COND(hr == 0x84204042, "J2 command accel limit over");
        ROS_ERROR_COND(hr == 0x84204043, "J3 command accel limit over");
        ROS_ERROR_COND(hr == 0x84204044, "J4 command accel limit over");
        ROS_ERROR_COND(hr == 0x84204045, "J5 command accel limit over");
        ROS_ERROR_COND(hr == 0x84204046, "J6 command accel limit over");

        if (state_ == R_CONNECTED)
        {
          disconnect();
          state_ = R_UNINITIALIZED;
        }
        else if (state_ > R_CONNECTED)
        {
          ROS_WARN_STREAM("Exit Cooperative Mode");
          ROS_WARN_STREAM("Stoppping Robot motors");
          motor_off();
          ROS_WARN_STREAM("Disconnecting Robot");
          disconnect();
          state_ = R_UNINITIALIZED;
        }
      }
    }
  }
  else
  {
    flag_buffer_full = false;
    buffer_full_counter = buffer_full_counter - 1;
  }
}

void DensoArmControl::publishRobotState()
{
  pub_state_msg_.data[0] = state_;
  pub_state_msg_.data[1] = state_;
  pub_state_.publish(pub_state_msg_);
}

void DensoArmControl::sendCmdSim()
{
  pub_sim_joint_cmd_msg_.header.stamp = ros::Time::now();
  VectorXd::Map(&pub_sim_joint_cmd_msg_.position[0],
                pub_sim_joint_cmd_msg_.position.size()) = des_joint_pos_;
  pub_sim_joint_cmd_.publish(pub_sim_joint_cmd_msg_);
}

} // namespace denso_nu
