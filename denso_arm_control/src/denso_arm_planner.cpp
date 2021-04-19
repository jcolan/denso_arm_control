/******************************************************************************
# denso_arm_planner.cpp:  DENSO Arm Planner                                   #
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

#include <denso_arm_control/denso_arm_planner.h>
#include <signal.h>
#include <thread>

namespace denso_nu
{
enum
{
  ACT_RESET = -1,
  ACT_NONE = 0,
  ACT_FOLLOWJOINT,
  ACT_FOLLOWTOOL,
};

// Constructor
DensoArmPlanner::DensoArmPlanner(ros::NodeHandle &node_handle, bool *kill_this_node)
{
  nh_ = node_handle;
  kill_this_node_ = kill_this_node;

  //* ROS PARAMETER SERVER

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

  sub_sim_joint_state_ = nh_.subscribe(prefix_ + "/joint_states", 1,
                                       &DensoArmPlanner::updateJointStateCb, this);

  // Publishers
  pub_joint_cmd_ = nh_.advertise<std_msgs::Float64MultiArray>(
      prefix_ + "/position_controller/command", 1);

  // Action Server
  actFollowJointCmd =
      std::make_shared<SimpleActionServer<denso_arm_control::FollowJointCmdAction>>(
          nh_, prefix_ + "/FollowJoint",
          boost::bind(&DensoArmPlanner::actFollowJointCmdCb, this, _1), false);

  actFollowJointCmd->registerPreemptCallback(
      boost::bind(&DensoArmPlanner::actCancelCb, this));

  actFollowJointCmd->start();

  actFollowToolCmd =
      std::make_shared<SimpleActionServer<denso_arm_control::FollowToolCmdAction>>(
          nh_, prefix_ + "/FollowTool",
          boost::bind(&DensoArmPlanner::actFollowToolCmdCb, this, _1), false);

  actFollowToolCmd->registerPreemptCallback(
      boost::bind(&DensoArmPlanner::actCancelCb, this));

  actFollowToolCmd->start();

  // Control loop timekeeping
  cycle_t_ns_d_ = (double)cycle_t_ns_;
  cycle_t_ = cycle_t_ns_d_ / NSEC2SEC_D;

  //* TRAC IK *//
  urdf_param_ = prefix_ + "/robot_description";
  ROS_INFO_STREAM("Extracting robot model from " << urdf_param_);
  tracik_solver_.reset(new TRAC_IK::TRAC_IK("base_link", "J6", urdf_param_, 0.004, 1e-5,
                                            TRAC_IK::Distance));

  // Extract KDL chain
  if (!(tracik_solver_->getKDLChain(kdl_chain_)))
  {
    ROS_ERROR_STREAM("There was no valid KDL chain found");
  }

  // Get number of joints in model
  n_joints_ = kdl_chain_.getNrOfJoints();
  ROS_INFO_STREAM("Model contains " << n_joints_ << " joints");

  // Get joint position limits for IK
  if (!(tracik_solver_->getKDLLimits(kdl_ll_, kdl_ul_)))
  {
    ROS_ERROR_STREAM("There was no valid KDL joint limits found");
  }

  // Verify the number of joint position limits
  assert(kdl_chain_.getNrOfJoints() == kdl_ll_.data.size());
  assert(kdl_chain_.getNrOfJoints() == kdl_ul_.data.size());

  // Create FK solver with KDL
  fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));

  // Resize publishing messages
  pub_joint_cmd_msg_.data.resize(n_joints_);

  m_curAct = ACT_NONE;

  // Initialize temporal variables for KDL
  qtmp_.resize(n_joints_);
  nominal_.resize(n_joints_);
  for (uint j = 0; j < nominal_.data.size(); j++)
  {
    nominal_(j) = (kdl_ll_(j) + kdl_ul_(j)) / 2.0;
  }

  //* Dynamic Reconfigure Server *//
  manip_vel_lim_ = 50.0;
  manip_acc_lim_ = 50.0;

  max_joint_vel_ = 200; //[deg/s]
  max_joint_acc_ = 50;  //[deg/s2]

  dyn_cb_ = boost::bind(&DensoArmPlanner::setDynVarCb, this, _1, _2);
  dyn_srv_.setCallback(dyn_cb_);

  //* Trajectory Generator *//
  joint_traj_gen_ = new OnlineTrajGen(6, cycle_t_);

  flag_joint_pos_initialized_ = false;
}

DensoArmPlanner::~DensoArmPlanner()
{
  // spinner->stop();
  delete joint_traj_gen_;
  tracik_solver_.reset();
  fk_solver_.reset();
}

bool DensoArmPlanner::start_joint_traj(VectorXd des_joint_pos)
{
  joint_traj_gen_->setCurrIn(act_joint_pos_, act_joint_vel_, act_joint_acc_);

  joint_traj_gen_->setLimit((manip_vel_lim_ / 100) * max_joint_vel_ * VectorXd::Ones(6),
                            (manip_acc_lim_ / 100) * max_joint_acc_ * VectorXd::Ones(6),
                            200 * VectorXd::Ones(6));
  joint_traj_gen_->setTarget(des_joint_pos, VectorXd::Zero(6));
  joint_traj_gen_->setStateReset();
  return true;
}

int DensoArmPlanner::generate_joint_traj(VectorXd &joint_traj_step)
{

  if (joint_traj_gen_->getState() != ReflexxesAPI::RML_FINAL_STATE_REACHED)
  {
    int ResultValue = joint_traj_gen_->initRml();
    if (ResultValue < 0)
    {
      ROS_ERROR_STREAM(
          "Traj. Generation failed. Check following information: " << ResultValue);
      joint_traj_gen_->printTarget();
      joint_traj_gen_->printActual();
      return -1;
    }
    else
    {
      RMLPositionOutputParameters *OP;
      OP = joint_traj_gen_->get_output();
      joint_traj_gen_->updateCurrIn(OP);
      joint_traj_step = VectorXd::Map(&OP->NewPositionVector->VecData[0], 6);

      return 0;
    }
  }
  else
    return 1;
}

//* Callbacks

void DensoArmPlanner::actFollowJointCmdCb(
    const denso_arm_control::FollowJointCmdGoalConstPtr &goal)
{
  int                                     hr;
  denso_arm_control::FollowJointCmdResult res;

  // Set current action
  boost::mutex::scoped_lock lockAct(mtx_Act_);
  if (m_curAct != ACT_NONE)
  {
    if (m_curAct != ACT_RESET)
    {
      res.result = EXIT_FAILURE;
      actFollowJointCmd->setAborted(res);
    }
    return;
  }

  m_curAct = ACT_FOLLOWJOINT;
  lockAct.unlock();

  // Execute action
  sensor_msgs::JointState joint_target;
  joint_target = goal->joint_pos;

  VectorXd joint_cmd =
      VectorXd::Map(&goal->joint_pos.position[0], goal->joint_pos.position.size());
  ROS_INFO_STREAM(" FOLLOW JOINT action received " << joint_cmd.transpose());

  hr = followJointCmd(joint_cmd);

  // Reset current action
  mtx_Act_.lock();
  if (m_curAct == ACT_FOLLOWJOINT)
  {
    if (hr == EXIT_SUCCESS)
    {
      res.result = EXIT_SUCCESS;
      actFollowJointCmd->setSucceeded(res);
    }
    else
    {
      res.result = EXIT_FAILURE;
    }
    m_curAct = ACT_NONE;
  }
  mtx_Act_.unlock();
}

void DensoArmPlanner::actFollowToolCmdCb(
    const denso_arm_control::FollowToolCmdGoalConstPtr &goal)
{
  int                                    hr;
  denso_arm_control::FollowToolCmdResult res;

  // Set current action
  boost::mutex::scoped_lock lockAct(mtx_Act_);
  if (m_curAct != ACT_NONE)
  {
    if (m_curAct != ACT_RESET)
    {
      res.result = EXIT_FAILURE;
      actFollowToolCmd->setAborted(res);
    }
    return;
  }

  m_curAct = ACT_FOLLOWTOOL;
  lockAct.unlock();

  Vector3d    pd;
  Matrix3d    Rd;
  Quaterniond qd;

  tf::pointMsgToEigen(goal->tool_pose.position, pd);
  tf::quaternionMsgToEigen(goal->tool_pose.orientation, qd);
  Rd = qd.toRotationMatrix();
  ROS_INFO_STREAM(" FOLLOW TOOL action received ");

  hr = followToolCmd(pd, Rd);

  // Reset current action
  mtx_Act_.lock();
  if (m_curAct == ACT_FOLLOWTOOL)
  {
    if (hr == EXIT_SUCCESS)
    {
      res.result = EXIT_SUCCESS;
      actFollowToolCmd->setSucceeded(res);
    }
    else
    {
      res.result = EXIT_FAILURE;
    }
    m_curAct = ACT_NONE;
  }
  mtx_Act_.unlock();
}

void DensoArmPlanner::actCancelCb()
{
  boost::mutex::scoped_lock lockAct(mtx_Act_);

  if (m_curAct > ACT_NONE)
  {

    ROS_INFO_STREAM("Preempting Controller Actions");
    switch (m_curAct)
    {
    case ACT_FOLLOWJOINT:
    {
      denso_arm_control::FollowJointCmdResult res_joint;
      res_joint.result = EXIT_FAILURE;
      actFollowJointCmd->setPreempted(res_joint);
      break;
    }
    case ACT_FOLLOWTOOL:
    {
      denso_arm_control::FollowToolCmdResult res_tool;
      res_tool.result = EXIT_FAILURE;
      actFollowToolCmd->setPreempted(res_tool);
      break;
    }
    }
  }
}

void DensoArmPlanner::updateJointStateCb(const sensor_msgs::JointState::ConstPtr &msg)
{
  act_joint_pos_ = (VectorXd::Map(&msg->position[0], 6)); //[deg]

  if (!flag_joint_pos_initialized_)
  {
    ROS_INFO_STREAM("Initial pose obtained for Traj generation");
    flag_joint_pos_initialized_ = true;
    joint_traj_gen_->setCurrIn(act_joint_pos_, VectorXd::Zero(6), VectorXd::Zero(6));
    joint_traj_gen_->setLimit((manip_vel_lim_ / 100) * max_joint_vel_ * VectorXd::Ones(6),
                              (manip_acc_lim_ / 100) * max_joint_acc_ * VectorXd::Ones(6),
                              100 * VectorXd::Ones(6));
    joint_traj_gen_->setTarget(act_joint_pos_, VectorXd::Zero(6));
    joint_traj_gen_->setStateReset();
    // joint_traj_gen_->printLimits();

    ROS_INFO_STREAM("Intial pose " << act_joint_pos_.transpose());
  }

  update_act_vel();
  update_act_acc();
}

void DensoArmPlanner::setDynVarCb(denso_arm_control::DynVarConfig &config, uint32_t level)
{
  ROS_WARN("Reconfigure Request velLim: %f accLim: %f", config.manip_vel_lim,
           config.manip_acc_lim);
  manip_vel_lim_ = (double)config.manip_vel_lim;
  manip_acc_lim_ = (double)config.manip_acc_lim;
  return;
}

int DensoArmPlanner::followJointCmd(VectorXd joint_cmd)
{
  VectorXd  joint_cmd_step;
  int       traj_finished;
  ros::Rate loop_rate(1 / cycle_t_);

  traj_finished = 0;
  joint_cmd_step.resize(n_joints_);
  start_joint_traj(joint_cmd);

  while (actFollowJointCmd->isActive() and traj_finished != 1)
  {
    if (flag_joint_pos_initialized_)
    {
      traj_finished = generate_joint_traj(joint_cmd_step);
      if (traj_finished != -1)
      {
        sendCmdController(joint_cmd_step);
      }
    }
    loop_rate.sleep();
    ros::spinOnce();
  }
  if (traj_finished == 1)
    return 0;
  else
    return -1;
}

int DensoArmPlanner::followToolCmd(Vector3d pd, Matrix3d Rd)
{
  VectorXd  joint_target;
  VectorXd  joint_cmd_step;
  int       traj_finished;
  int       res_ik = 0;
  ros::Rate loop_rate(1 / cycle_t_);
  joint_target.resize(n_joints_);

  res_ik = solveIK(pd, Rd, joint_target);

  traj_finished = 0;
  joint_cmd_step.resize(n_joints_);
  start_joint_traj(joint_target);

  if (res_ik == 0)
  {
    while (actFollowToolCmd->isActive() and traj_finished != 1)
    {
      if (flag_joint_pos_initialized_)
      {
        traj_finished = generate_joint_traj(joint_cmd_step);
        // ROS_INFO_STREAM("CMD: " << joint_cmd_step.transpose());
        if (traj_finished != -1)
        {
          sendCmdController(joint_cmd_step);
        }
      }
      loop_rate.sleep();
      ros::spinOnce();
    }
  }

  if (traj_finished == 1)
    return 0;
  else
    return -1;
}

void DensoArmPlanner::sendCmdController(VectorXd joint_cmd_step)
{
  // ROS_INFO_STREAM("PUB: " << joint_cmd_step.transpose());

  VectorXd::Map(&pub_joint_cmd_msg_.data[0], pub_joint_cmd_msg_.data.size()) =
      joint_cmd_step;
  pub_joint_cmd_.publish(pub_joint_cmd_msg_);
}

//* Kinematics functions

int DensoArmPlanner::solveFK(VectorXd joint_val, Vector3d &B_p_TOOL, Matrix3d &R_B_TOOL)
{
  qtmp_.data = joint_val * DEG2RAD;

  bool kinematics_status;
  kinematics_status = fk_solver_->JntToCart(qtmp_, xtmp_);

  Affine3d T01;

  if (!kinematics_status)
  {
    tf::transformKDLToEigen(xtmp_, T01);
    R_B_TOOL = T01.rotation();
    B_p_TOOL = 1000 * T01.translation();

    return 0;
  }
  else
    ROS_ERROR_STREAM("No Forward Kinematics solution");
  return -1;
}

int DensoArmPlanner::solveIK(Vector3d pd, Matrix3d Rd, VectorXd &joint_cmd)
{
  int           rc;
  KDL::JntArray qd;
  KDL::Frame    ee;
  Affine3d      Td01;

  Td01.linear() = Rd;
  Td01.translation() = pd / 1000;

  tf::transformEigenToKDL(Td01, ee);

  // nominal_ = qd;
  nominal_.data = act_joint_pos_;
  rc = tracik_solver_->CartToJnt(nominal_, ee, qd);
  if (rc >= 0)
  {

    joint_cmd = qd.data * RAD2DEG;

    return 0;
  }
  else
  {
    ROS_WARN_STREAM("No IK solution found for \np:\n" << pd.transpose());
    ROS_WARN_STREAM("R: \n" << Rd);
    return -1;
  }
}

void DensoArmPlanner::update_act_vel()
{
  static VectorXd prev_joint_pos(VectorXd::Zero(6));
  act_joint_vel_ = (act_joint_pos_ - prev_joint_pos) / (cycle_t_);
  prev_joint_pos = act_joint_pos_;
}

void DensoArmPlanner::update_act_acc()
{
  static VectorXd prev_joint_vel(VectorXd::Zero(6));
  act_joint_acc_ = (act_joint_vel_ - prev_joint_vel) / (cycle_t_);
  prev_joint_vel = act_joint_vel_;
}

void DensoArmPlanner::control_loop()
{
  while (not(*kill_this_node_))
  {
  }
}

} // namespace denso_nu

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
  ros::init(argc, argv, "denso_arm_planner");
  ros::NodeHandle node_handle;
  signal(SIGINT, SigIntHandler);

  DensoArmPlanner dp(node_handle, &kill_this_process);

  RealtimeSchedulerInterface::activateRRScheduler(99);
  RealtimeSchedulerInterface::display_thread_sched_attr(
      "Trying to upgrade to real-time SCHED_DEADLINE scheduler...");

  ros::AsyncSpinner spinner(2);
  spinner.start();

  std::thread t(boost::bind(&DensoArmPlanner::control_loop, &dp));

  t.join();
  spinner.stop();

  return 0;
}