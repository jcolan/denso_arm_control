/******************************************************************************
# denso_arm_planner.h:  DENSO Arm Control                               #
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

#ifndef DENSO_ARM_PLANNER_H
#define DENSO_ARM_PLANNER_H

// C++
#include <mutex>
#include <functional>
#include <memory>

// Ros related
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32MultiArray.h>

// Eigen
#include <Eigen/Dense>

// Orocos KDL
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

// KDL Parser
#include <kdl_parser/kdl_parser.hpp>

// Boost
#include <boost/scoped_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

// Track IK
#include <trac_ik/trac_ik.hpp>

// Trajectory Generator
#include <denso_arm_control/MotionPlanner.h>

// RT
#include <realtime_scheduler_interface/realtime_clock.h>
#include <realtime_scheduler_interface/realtime_scheduler_interface.h>

// Dynamic reconfigure
#include <denso_arm_control/DynVarConfig.h>
#include <dynamic_reconfigure/server.h>

// Eigen conversions
#include <eigen_conversions/eigen_kdl.h>
#include <eigen_conversions/eigen_msg.h>

// Actions
#include <actionlib/server/simple_action_server.h>
#include "denso_arm_control/FollowJointCmdAction.h"
#include "denso_arm_control/FollowToolCmdAction.h"

using namespace Eigen;
using namespace KDL;
using namespace rt_motion_planner;
using namespace actionlib;

namespace denso_nu
{

class DensoArmPlanner
{

  const int    NSEC2SEC = 1000000000;
  const int    NSEC2SEC_D = 1000000000.0;
  const double DEG2RAD = (M_PI) / 180;
  const double RAD2DEG = 180 / (M_PI);

  public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Constructor
  DensoArmPlanner(ros::NodeHandle &node_handle, bool *kill_this_node);
  ~DensoArmPlanner();

  // Callbacks
  void actFollowJointCmdCb(const denso_arm_control::FollowJointCmdGoalConstPtr &goal);
  void actFollowToolCmdCb(const denso_arm_control::FollowToolCmdGoalConstPtr &goal);
  void actCancelCb();
  void updateJointStateCb(const sensor_msgs::JointState::ConstPtr &msg);
  void setDynVarCb(denso_arm_control::DynVarConfig &config, uint32_t level);

  // Trajectory generation
  bool start_joint_traj(VectorXd des_joint_pos);
  int  generate_joint_traj(VectorXd &joint_traj_step);

  // Kinematics
  int solveFK(VectorXd joint_val, Vector3d &B_p_TOOL, Matrix3d &R_B_TOOL);
  int solveIK(const Vector3d pd, const Matrix3d Rd, VectorXd &joint_cmd);

  void update_act_vel();
  void update_act_acc();

  void control_loop();

  int  followJointCmd(VectorXd joint_cmd);
  int  followToolCmd(Vector3d pd, Matrix3d Rd);
  void sendCmdController(VectorXd joint_cmd_step);

  private:
  bool *kill_this_node_;

  // ROS
  ros::NodeHandle nh_;

  // Suscribers
  ros::Subscriber sub_sim_joint_state_;

  // Publishers
  ros::Publisher              pub_joint_cmd_;
  std_msgs::Float64MultiArray pub_joint_cmd_msg_;

  // Action Server
  std::shared_ptr<SimpleActionServer<denso_arm_control::FollowJointCmdAction>>
      actFollowJointCmd;
  std::shared_ptr<SimpleActionServer<denso_arm_control::FollowToolCmdAction>>
      actFollowToolCmd;

  // Robot Variables
  int         robot_id;
  std::string prefix_;
  int         n_joints_;

  // Time variables
  double cycle_t_;
  int    cycle_t_ns_;
  double cycle_t_ns_d_;

  // Trajectory Generation
  OnlineTrajGen *joint_traj_gen_;

  // Joint variables
  VectorXd act_joint_pos_;
  VectorXd act_joint_vel_;
  VectorXd act_joint_acc_;

  // Dynamic Reconfigure
  dynamic_reconfigure::Server<denso_arm_control::DynVarConfig>               dyn_srv_;
  dynamic_reconfigure::Server<denso_arm_control::DynVarConfig>::CallbackType dyn_cb_;

  double manip_vel_lim_;
  double manip_acc_lim_;

  // FK KDL
  boost::scoped_ptr<KDL::ChainFkSolverPos> fk_solver_;

  // TracIK
  boost::scoped_ptr<TRAC_IK::TRAC_IK> tracik_solver_;

  // Limits
  double max_joint_vel_;
  double max_joint_acc_;

  // Temporary variables for KDL
  KDL::Chain    kdl_chain_;
  KDL::JntArray nominal_;
  KDL::JntArray qtmp_;
  KDL::JntArray kdl_ll_, kdl_ul_;
  KDL::Frame    xtmp_;
  KDL::Twist    xdot_temp_;
  KDL::JntArray qdot_tmp_;

  std::string urdf_param_;

  // Mutex
  boost::mutex mtx_Act_;
  int          m_curAct;

  // Flags
  bool flag_joint_pos_initialized_;
};

} // namespace denso_nu

#endif