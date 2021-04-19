/*
# Copyright (c) 2016
# Mitsuishi Sugita Laboratory (NML) at University of Tokyo
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     - Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     - Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     - Neither the name of the Mitsuishi Sugita Laboratory (NML) nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as
# published by the Free Software Foundation, either version 3 of the
# License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License LGPL for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License LGPL along with this program.
# If not, see <http://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Murilo M. Marinho, email: murilo@nml.t.u-tokyo.ac.jp
#
# ################################################################
#
#    Modified by: Jacinto Colan, email: colan@robo.mein.nagoya-u.ac.jp
#
#*/

#ifndef DENSO_ROBOT_HEADER_GUARD
#define DENSO_ROBOT_HEADER_GUARD

#include <ros/ros.h>

#include "BCAP.h"

// Eigen
#include <pthread.h>
#include <time.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Householder>
#include <sstream>

using namespace Eigen;

class DensoRobot
{
  public:
  // Constants
  int nsec2sec;
  // const static int SLAVE_MODE_JOINT_CONTROL;
  // const static int SLAVE_MODE_END_EFFECTOR_CONTROL;

  // BCAP driver
  BCAP::Driver bCapDriver_;
  // Thread-safe access.
  pthread_mutex_t mutex_;

  // Joint positions
  VectorXd joint_positions_;
  //    VectorXd end_effector_pose_;
  std::vector<double> joint_positions_buffer_;
  //    std::vector<double> end_effector_pose_euler_buffer_;
  //    std::vector<double> end_effector_pose_homogenous_transformation_buffer_;

  // P-pose
  VectorXd            p_pose_;
  std::vector<double> p_pose_buffer_;

  // T-pose
  VectorXd            t_pose_;
  std::vector<double> t_pose_buffer_;

  // F Value
  VectorXd            f_value_;
  std::vector<double> f_value_buffer_;

  // Timing related (to use nanosleep)
  int             thread_sampling_time_nsec_;
  struct timespec time_spec_;

  // Shutdown signal indicator
  bool *kill_threads_;

  // Controller status
  int         controller_status_;
  std::string controller_error_message_;

  DensoRobot(std::string server_ip_address, const int server_port_number,
             bool *kill_threads, int thread_sampling_time_nsec = 8000000)
  {
    bCapDriver_ = BCAP::Driver(server_ip_address, server_port_number);

    joint_positions_.resize(6);
    joint_positions_buffer_.resize(8, 0);

    p_pose_.resize(7);
    p_pose_buffer_.resize(7, 0);

    t_pose_.resize(10);
    t_pose_buffer_.resize(10, 0);

    f_value_.resize(10);
    f_value_buffer_.resize(10, 0);

    thread_sampling_time_nsec_ = thread_sampling_time_nsec;

    nsec2sec = 1000000000;

    kill_threads_ = kill_threads;

    pthread_mutex_init(&mutex_, NULL);
  }

  DensoRobot(){};

  int GetEmergencyStop()
  {
    int     result;
    HRESULT error_code;
    pthread_mutex_lock(&mutex_);
    error_code = bCapDriver_.GetEmergencyStop(result);
    pthread_mutex_unlock(&mutex_);
    if ((int)error_code < 0)
    {
      ROS_ERROR_STREAM("  FAILED TO GetEmergencyStop() . Error code = " << error_code);
      return error_code;
    }
    return result;
  }

  VectorXd GetJointPositions(int *error_code)
  {
    // If the code is slow I need to remove this intermediary std::vector.
    (*error_code) = bCapDriver_.GetJointPositions(joint_positions_buffer_);
    Map<VectorXd> joint_positions_(joint_positions_buffer_.data(), 6);
    return joint_positions_;
  }

  VectorXd GetPPose(int *error_code)
  {
    // If the code is slow I need to remove this intermediary std::vector.
    (*error_code) = bCapDriver_.GetPPose(p_pose_buffer_);
    Map<VectorXd> p_pose_(p_pose_buffer_.data(), 7);
    return p_pose_;
  }

  VectorXd GetTPose(int *error_code)
  {
    (*error_code) = bCapDriver_.GetTPose(t_pose_buffer_);
    Map<VectorXd> t_pose_(t_pose_buffer_.data(), 10);
    return t_pose_;
  }

  VectorXd GetFValue(int *error_code)
  {
    (*error_code) = bCapDriver_.GetFValue(f_value_buffer_);
    Map<VectorXd> f_value_(f_value_buffer_.data(), 3);
    return f_value_;
  }

  HRESULT SetJointPositions(VectorXd &desired_joint_positions)
  {
    // If the code is slow I need to remove this intermediary std::vector.
    std::vector<double> joint_pos(desired_joint_positions.data(),
                                  desired_joint_positions.data() + 6);
    return bCapDriver_.SetJointPositions(joint_pos, joint_positions_buffer_);
  }

  int SetPPose(VectorXd &desired_p_pose)
  {
    // If the code is slow I need to remove this intermediary std::vector.
    std::vector<double> p_pose_buffer_(desired_p_pose.data(), desired_p_pose.data() + 7);
    return bCapDriver_.SetPPose(p_pose_buffer_);
  }

  HRESULT SetTPose(VectorXd &desired_t_pose)
  {
    // If the code is slow I need to remove this intermediary std::vector.
    std::vector<double> t_pose_buffer(desired_t_pose.data(), desired_t_pose.data() + 10);
    return bCapDriver_.SetTPose(t_pose_buffer, t_pose_buffer_);
  }
  VectorXd GetJointPositionsBuffer()
  {
    Map<VectorXd> joint_positions_(joint_positions_buffer_.data(), 6);
    return joint_positions_;
  }

  VectorXd GetTValueBuffer()
  {
    Map<VectorXd> t_pose_(t_pose_buffer_.data(), 10);
    return t_pose_;
  }

  int StartTask()
  {
    bCapDriver_.StartTask();
    return 0;
  }

  int StopTask()
  {
    bCapDriver_.StopTask();
    return 0;
  }

  int Connect()
  {
    bool worked; // bCap communication error code

    pthread_mutex_lock(&mutex_);
    worked = bCapDriver_.Open();
    pthread_mutex_unlock(&mutex_);
    if (!worked)
    {
      ROS_ERROR_STREAM(
          "  FAILED TO Open() IN FUNCTION Connect(). Error code = " << worked);
      return -1;
    }

    pthread_mutex_lock(&mutex_);
    worked = bCapDriver_.ServiceStart();
    pthread_mutex_unlock(&mutex_);
    if (!worked)
    {
      ROS_ERROR("  FAILED TO ServiceStart() IN FUNCTION Connect(). Error code = %x",
                worked);
      return -1;
    }
    // ROS_INFO_STREAM("bCap Service started.");

    pthread_mutex_lock(&mutex_);
    worked = bCapDriver_.ControllerConnect();
    pthread_mutex_unlock(&mutex_);
    if (!worked)
    {
      ROS_ERROR_STREAM("  FAILED TO ControllerConnect() IN FUNCTION "
                       "Connect(). Error code = "
                       << worked);
      return -1;
    }
    // ROS_INFO_STREAM("bCap Controller connected.");

    pthread_mutex_lock(&mutex_);
    bCapDriver_.InitilizeControllerVariableHandles();
    pthread_mutex_unlock(&mutex_);
    if (!worked)
    {
      ROS_ERROR_STREAM("  FAILED TO InitializedControllerVariableHandles() IN "
                       "FUNCTION Connect(). Error code = "
                       << worked);
      return -1;
    }
    // ROS_INFO_STREAM("bCap Controller variable handles initialized.");

    pthread_mutex_lock(&mutex_);
    worked = bCapDriver_.GetRobot();
    pthread_mutex_unlock(&mutex_);
    if (!worked)
    {
      ROS_ERROR_STREAM(
          "  FAILED TO GetRobot() IN FUNCTION Connect(). Error code = " << worked);
      return -1;
    }
    // ROS_INFO_STREAM("bCap Got robot.");

    pthread_mutex_lock(&mutex_);
    worked = bCapDriver_.TakeArm();
    pthread_mutex_unlock(&mutex_);
    if (!worked)
    {
      ROS_ERROR_STREAM(
          "  FAILED TO TakeArm() IN FUNCTION Connect(). Error code = " << worked);
      return -1;
    }

    return 0;
  }

  int MotorOn()
  {
    bool worked; // bCap communication error code

    pthread_mutex_lock(&mutex_);
    worked = bCapDriver_.SetMotorState(true);
    pthread_mutex_unlock(&mutex_);
    if (!worked)
    {
      ROS_ERROR_STREAM("  FAILED TO SetMotorState(true) IN FUNCTION "
                       "MotorOn(). Error code = "
                       << worked);
      return -1;
    }

    return 0;
  }

  int SetSpeed(const float &speed, const float &acceleration, const float &deacceleration)
  {
    bool worked; // bCap communication error code

    pthread_mutex_lock(&mutex_);
    worked = bCapDriver_.SetSpeed(speed, acceleration, deacceleration);
    pthread_mutex_unlock(&mutex_);
    if (!worked)
    {
      ROS_ERROR_STREAM(
          "  FAILED TO SetSpeed() IN FUNCTION SetSpeed(). Error code = " << worked);
      return -1;
    }

    return 0;
  }

  int SlaveModeOn(int mode)
  {
    bool worked; // bCap communication error code

    pthread_mutex_lock(&mutex_);
    worked = bCapDriver_.SetSlaveMode(mode);
    pthread_mutex_unlock(&mutex_);
    if (!worked)
    {
      ROS_ERROR_STREAM("  FAILED TO SetSlaveMode() IN FUNCTION "
                       "SlaveModeOn(). Error code = "
                       << worked);
      return -1;
    }

    return 0;
  }

  int SetRecvFormat(int mode)
  {
    bool worked; // bCap communication error code

    pthread_mutex_lock(&mutex_);
    worked = bCapDriver_.SetRecvFormat(mode);
    pthread_mutex_unlock(&mutex_);
    if (!worked)
    {
      ROS_ERROR_STREAM("  FAILED TO SetRecvFormat() IN FUNCTION "
                       "SetRecvFormat(). Error code = "
                       << worked);
      return -1;
    }

    return 0;
  }

  int MotorOff()
  {
    bool worked;

    pthread_mutex_lock(&mutex_);
    worked = bCapDriver_.SetMotorState(false);
    pthread_mutex_unlock(&mutex_);
    if (!worked)
    {
      ROS_ERROR_STREAM("  FAILED TO SetMotorState(true) IN FUNCTION "
                       "MotorOn(). Error code = "
                       << worked);
      return -1;
    }

    return 0;
  }

  int SlaveModeOff()
  {
    bool worked;

    pthread_mutex_lock(&mutex_);
    worked = bCapDriver_.SetSlaveMode(0);
    pthread_mutex_unlock(&mutex_);
    if (!worked)
    {
      ROS_ERROR_STREAM(
          "  FAILED TO SetSlaveMode() IN FUNCTION MotorOn(). Error code = " << worked);
      return -1;
    }

    return 0;
  }

  int Disconnect()
  {
    bool worked;

    pthread_mutex_lock(&mutex_);
    worked = bCapDriver_.GiveArm();
    pthread_mutex_unlock(&mutex_);
    if (!worked)
    {
      ROS_ERROR_STREAM(
          "  FAILED TO GiveArm() IN FUNCTION Disconnect(). Error code = " << worked);
      return -1;
    }

    pthread_mutex_lock(&mutex_);
    worked = bCapDriver_.ReleaseRobot();
    pthread_mutex_unlock(&mutex_);
    if (!worked)
    {
      ROS_ERROR_STREAM(
          "  FAILED TO ReleaseRobot() IN FUNCTION Disconnect(). Error code = " << worked);
      return -1;
    }

    pthread_mutex_lock(&mutex_);
    worked = bCapDriver_.ControllerDisconnect();
    pthread_mutex_unlock(&mutex_);
    if (!worked)
    {
      ROS_ERROR_STREAM("  FAILED TO ControllerDisconnect() IN FUNCTION "
                       "Disconnect(). Error code = "
                       << worked);
      return -1;
    }

    pthread_mutex_lock(&mutex_);
    worked = bCapDriver_.ServiceStop();
    pthread_mutex_unlock(&mutex_);
    if (!worked)
    {
      ROS_ERROR_STREAM(
          "  FAILED TO ServiceStop() IN FUNCTION Disconnect(). Error code = " << worked);
      return -1;
    }

    pthread_mutex_lock(&mutex_);
    worked = bCapDriver_.Close();
    pthread_mutex_unlock(&mutex_);
    if (!worked)
    {
      ROS_ERROR_STREAM(
          "  FAILED TO Close() IN FUNCTION Disconnect(). Error code = " << worked);
      return -1;
    }

    return 0;
  }
};

#endif
