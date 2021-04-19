/******************************************************************************
# denso_arm_example.cpp: Denso Arm Example                                  # #
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

// C
#include <signal.h>

// ROS
#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>

// Eigen
#include <Eigen/Dense>

// Eigen conversions
#include <eigen_conversions/eigen_kdl.h>
#include <eigen_conversions/eigen_msg.h>

// Actions
#include <actionlib/client/simple_action_client.h>
#include "denso_arm_control/FollowJointCmdAction.h"
#include "denso_arm_control/FollowToolCmdAction.h"

#include <denso_arm_control/status.h>

using namespace Eigen;
using namespace denso_arm_control;

typedef actionlib::SimpleActionClient<FollowJointCmdAction> FollowJointCmdClient;
typedef actionlib::SimpleActionClient<FollowToolCmdAction>  FollowToolCmdClient;

class DensoArmDemo
{
  public:
  DensoArmDemo(ros::NodeHandle &node_handle, std::string prefix)
      : ac_joint_cmd_(prefix + "/FollowJoint", true),
        ac_tool_cmd_(prefix + "/FollowTool", true)
  {
    nh_ = node_handle;
    seq_state_ = 0;
    act_state_ = -1;
    n_joints_ = 6;
    robot_status_ = R_UNINITIALIZED;

    sub_robot_state_ = nh_.subscribe(prefix + "/robot/state", 1,
                                     &DensoArmDemo::updateRobotStateCb, this);
  }
  ~DensoArmDemo() {}

  // Callbacks
  void updateRobotStateCb(const std_msgs::Int32MultiArray::ConstPtr &msg)
  {
    robot_status_ = msg->data[0];
  }

  void actFollowJointCmdDoneCb(const actionlib::SimpleClientGoalState &state,
                               const FollowJointCmdResultConstPtr &    result)
  {
    bool res = result->result;
    if (res == EXIT_SUCCESS)
    {
      ROS_INFO_STREAM("Joint target reached");
      if (seq_state_ < 4)
        seq_state_ = seq_state_ + 1;
      else
        seq_state_ = -1;
    }
    else
    {
      ROS_WARN_STREAM("Joint target aborted ");
    }
  }

  void actFollowToolCmdDoneCb(const actionlib::SimpleClientGoalState &state,
                              const FollowToolCmdResultConstPtr &     result)
  {
    bool res = result->result;
    if (res == EXIT_SUCCESS)
    {
      ROS_INFO_STREAM("Tool target reached");
      if (seq_state_ < 4)
        seq_state_ = seq_state_ + 1;
      else
        seq_state_ = -1;
    }
    else
    {
      ROS_WARN_STREAM("Tool target aborted ");
    }
  }

  void sendJointGoal(VectorXd joint_seq)
  {
    FollowJointCmdGoal joint_goal;
    joint_goal.joint_pos.name.resize(n_joints_);
    joint_goal.joint_pos.position.resize(n_joints_);
    joint_goal.joint_pos.velocity.resize(n_joints_);
    joint_goal.joint_pos.effort.resize(n_joints_);

    VectorXd::Map(&joint_goal.joint_pos.position[0],
                  joint_goal.joint_pos.position.size()) = joint_seq;
    VectorXd::Map(&joint_goal.joint_pos.velocity[0],
                  joint_goal.joint_pos.velocity.size()) = VectorXd::Zero(n_joints_);
    VectorXd::Map(&joint_goal.joint_pos.effort[0], joint_goal.joint_pos.effort.size()) =
        VectorXd::Zero(n_joints_);

    ROS_INFO_STREAM("Waiting for action server to start");
    ac_joint_cmd_.waitForServer();
    ROS_INFO_STREAM("Action server started. Sending goal");
    ac_joint_cmd_.sendGoal(
        joint_goal, boost::bind(&DensoArmDemo::actFollowJointCmdDoneCb, this, _1, _2),
        FollowJointCmdClient::SimpleActiveCallback(),
        FollowJointCmdClient::SimpleFeedbackCallback());
  }

  void sendToolGoal(Vector3d pd, Matrix3d Rd)
  {
    FollowToolCmdGoal tool_goal;
    Quaterniond       qd(Rd);

    Affine3d Td;
    Td = Affine3d::Identity();
    Td.linear() = Rd;
    Td.translation() = pd;

    tf::poseEigenToMsg(Td, tool_goal.tool_pose);

    ROS_INFO_STREAM("Waiting for action server to start");
    ac_tool_cmd_.waitForServer();
    ROS_INFO_STREAM("Action server started. Sending goal");
    ac_tool_cmd_.sendGoal(
        tool_goal, boost::bind(&DensoArmDemo::actFollowToolCmdDoneCb, this, _1, _2),
        FollowToolCmdClient::SimpleActiveCallback(),
        FollowToolCmdClient::SimpleFeedbackCallback());
  }

  int sendJointSeq()
  {
    VectorXd joint_seq;
    joint_seq.resize(6);

    if (robot_status_ >= R_SLAVE and seq_state_ != act_state_)
    {
      switch (seq_state_)
      {
      case 0:
      {
        act_state_ = seq_state_;
        // joint_seq << 10, 10, 10, 10, 10, 10;
        joint_seq << -30, 10, 60, 1, 1, 1;

        ROS_WARN_STREAM("Sending Goal-0:");
        ROS_WARN_STREAM("J:" << joint_seq.transpose());
        sendJointGoal(joint_seq);
        break;
      }
      case 1:
      {
        act_state_ = seq_state_;
        // joint_seq << 30, 30, 30, 30, 30, 30;
        joint_seq << 0, 15, 75, 10, 10, 10;

        ROS_WARN_STREAM("Sending Goal-1:");
        ROS_WARN_STREAM("J:" << joint_seq.transpose());
        sendJointGoal(joint_seq);
        break;
      }
      case 2:
      {
        act_state_ = seq_state_;
        // joint_seq << 60, 60, 60, 60, 60, 60;
        joint_seq << 30, 30, 90, 30, 30, 30;

        ROS_WARN_STREAM("Sending Goal-2:");
        ROS_WARN_STREAM("J:" << joint_seq.transpose());
        sendJointGoal(joint_seq);
        break;
      }
      case 3:
      {
        act_state_ = seq_state_;
        // joint_seq << 30, 30, 30, 30, 30, 30;
        joint_seq << 0, 15, 75, 10, 10, 10;

        ROS_WARN_STREAM("Sending Goal-3:");
        ROS_WARN_STREAM("J:" << joint_seq.transpose());
        sendJointGoal(joint_seq);
        break;
      }
      case 4:
      {
        act_state_ = seq_state_;
        // joint_seq << 10, 10, 10, 10, 10, 10;
        joint_seq << -30, 10, 60, 1, 1, 1;

        ROS_WARN_STREAM("Sending Goal-4:");
        ROS_WARN_STREAM("J:" << joint_seq.transpose());
        sendJointGoal(joint_seq);
        break;
      }
      case -1:
        return -1;
      }
    }

    return 0;
  }

  void reset_demo_state() { seq_state_ = 0; }

  int sendToolSeq()
  {
    Matrix3d R_seq;
    Vector3d p_seq;

    if (robot_status_ >= R_SLAVE and seq_state_ != act_state_)
    {
      switch (seq_state_)
      {
      case 0:
      {
        act_state_ = seq_state_;
        // R_seq << 0, 0, 1, -1, 0, 0, 0, -1, 0;
        R_seq << -1, 0, 0, 0, 1, 0, 0, 0, -1;
        p_seq << 250, -100, 500;
        ROS_WARN_STREAM("Sending Goal-0");
        ROS_WARN_STREAM("p:\n" << p_seq.transpose());
        ROS_WARN_STREAM("R:\n" << R_seq);
        sendToolGoal(p_seq, R_seq);
        break;
      }
      case 1:
      {
        act_state_ = seq_state_;
        // R_seq << 0, 0, 1, -1, 0, 0, 0, -1, 0;
        R_seq << -1, 0, 0, 0, 1, 0, 0, 0, -1;
        p_seq << 300, -100, 500;
        ROS_WARN_STREAM("Sending Goal-1");
        ROS_WARN_STREAM("p:" << p_seq.transpose());
        ROS_WARN_STREAM("R:" << R_seq);
        sendToolGoal(p_seq, R_seq);
        break;
      }
      case 2:
      {
        act_state_ = seq_state_;
        // R_seq << 0, 0, 1, -1, 0, 0, 0, -1, 0;
        R_seq << -1, 0, 0, 0, 1, 0, 0, 0, -1;
        p_seq << 300, 100, 500;
        ROS_WARN_STREAM("Sending Goal-2");
        ROS_WARN_STREAM("p:" << p_seq.transpose());
        ROS_WARN_STREAM("R:" << R_seq);
        sendToolGoal(p_seq, R_seq);
        break;
      }
      case 3:
      {
        act_state_ = seq_state_;
        // R_seq << 0, 0, 1, -1, 0, 0, 0, -1, 0;
        R_seq << -1, 0, 0, 0, 1, 0, 0, 0, -1;
        p_seq << 250, 100, 500;
        ROS_WARN_STREAM("Sending Goal-3");
        ROS_WARN_STREAM("p:" << p_seq.transpose());
        ROS_WARN_STREAM("R:" << R_seq);
        sendToolGoal(p_seq, R_seq);
        break;
      }
      case 4:
      {
        act_state_ = seq_state_;
        // R_seq << 0, 0, 1, -1, 0, 0, 0, -1, 0;
        R_seq << -1, 0, 0, 0, 1, 0, 0, 0, -1;
        p_seq << 250, -100, 500;
        ROS_WARN_STREAM("Sending Goal-4");
        ROS_WARN_STREAM("p:" << p_seq.transpose());
        ROS_WARN_STREAM("R:" << R_seq);
        sendToolGoal(p_seq, R_seq);
        break;
      }
      case -1:
        return -1;
      }
    }

    return 0;
  }

  private:
  ros::NodeHandle nh_;

  // Subscriber
  ros::Subscriber sub_robot_state_;

  //   Action clients
  FollowJointCmdClient ac_joint_cmd_;
  FollowToolCmdClient  ac_tool_cmd_;

  int seq_state_;
  int act_state_;
  int n_joints_;
  int robot_status_;
};

bool kill_this_process = false;

void SigIntHandler(int signal)
{
  kill_this_process = true;
  ROS_INFO_STREAM("SHUTDOWN SIGNAL RECEIVED");
}

/**
 * Main function: Initialize the commun
 */
int main(int argc, char **argv)
{

  int demo = 0;
  // Initialize ROS
  ros::init(argc, argv, "denso_arm_example");
  ros::NodeHandle node_handle;

  std::string prefix;

  // Updating Parameters
  if (!node_handle.getParam("prefix", prefix))
  {
    prefix = "";
  }

  ros::Rate loop_rate(1);

  signal(SIGINT, SigIntHandler);
  DensoArmDemo dd(node_handle, prefix);

  while (ros::ok)
  {
    if (kill_this_process)
    {
      ROS_INFO_STREAM("Killing ROS");
      break;
    }

    if (demo == 0)
    {
      int res_joint = dd.sendJointSeq();
      if (res_joint == -1)
      {
        demo = 1;
        dd.reset_demo_state();
        ROS_INFO_STREAM("Denso arm JOINT example sequence finished.");
      }
    }
    else
    {
      int res_tool = dd.sendToolSeq();
      if (res_tool == -1)
      {
        ROS_INFO_STREAM("Denso arm TOOL example sequence finished.");
        break;
      }
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}