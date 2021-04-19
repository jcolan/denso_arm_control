/******************************************************************************
# MotionPlanner.h:  Motion planner based on Reflexxes library                 #
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

#ifndef ROBOT_MOTION_PLANNER_H
#define ROBOT_MOTION_PLANNER_H

// Eigen
#include <Eigen/Dense>

// Eigen Conversions
#include <eigen_conversions/eigen_msg.h>
#include <eigen_conversions/eigen_kdl.h>

// Orocos KDL
#include <kdl/velocityprofile_spline.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <boost/scoped_ptr.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>

// Boost
#include <boost/scoped_ptr.hpp>

// Track-IK
#include <trac_ik/trac_ik.hpp>

// Reflexxes
#include <RMLPositionFlags.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>
#include <ReflexxesAPI.h>

// C++ Std 11
#include <iostream>

using namespace Eigen;
using namespace KDL;
//
// template <typename T>
// struct Vector
// {
//
// } ;

namespace rt_motion_planner
{

class OnlineTrajGen
{

  public:
  OnlineTrajGen(int n_dofs, double cycle_time_s)
      : n_dofs_(n_dofs), cycle_time_s_(cycle_time_s)
  {

    RML = new ReflexxesAPI(n_dofs_, cycle_time_s);

    IP = new RMLPositionInputParameters(n_dofs_);

    OP = new RMLPositionOutputParameters(n_dofs_);

  }; // Constructor

  ~OnlineTrajGen()
  {
    delete RML;
    delete IP;
    delete OP;
  }; // Destructor

  // Functions
  void control_loop();

  int initRml()
  {
    // IP->CurrentPositionVector->VecData[0] = 100.0;
    // IP->CurrentPositionVector->VecData[1] = 0.0;
    // IP->CurrentPositionVector->VecData[2] = 50.0;
    //
    // IP->CurrentVelocityVector->VecData[0] = 100.0;
    // IP->CurrentVelocityVector->VecData[1] = -220.0;
    // IP->CurrentVelocityVector->VecData[2] = -50.0;
    //
    // IP->CurrentAccelerationVector->VecData[0] = -150.0;
    // IP->CurrentAccelerationVector->VecData[1] = 250.0;
    // IP->CurrentAccelerationVector->VecData[2] = -50.0;
    //
    // IP->MaxVelocityVector->VecData[0] = 300.0;
    // IP->MaxVelocityVector->VecData[1] = 100.0;
    // IP->MaxVelocityVector->VecData[2] = 300.0;
    //
    // IP->MaxAccelerationVector->VecData[0] = 300.0;
    // IP->MaxAccelerationVector->VecData[1] = 200.0;
    // IP->MaxAccelerationVector->VecData[2] = 100.0;
    //
    // IP->MaxJerkVector->VecData[0] = 400.0;
    // IP->MaxJerkVector->VecData[1] = 300.0;
    // IP->MaxJerkVector->VecData[2] = 200.0;
    //
    // IP->TargetPositionVector->VecData[0] = -600.0;
    // IP->TargetPositionVector->VecData[1] = -200.0;
    // IP->TargetPositionVector->VecData[2] = -350.0;
    //
    // IP->TargetVelocityVector->VecData[0] = 50.0;
    // IP->TargetVelocityVector->VecData[1] = -50.0;
    // IP->TargetVelocityVector->VecData[2] = -200.0;
    for (int i = 0; i < n_dofs_; i++)
    {
      IP->SelectionVector->VecData[i] = true;
    }

    int resultValue = RML->RMLPosition(*IP, OP, Flags);
    state_ = resultValue;
    return resultValue;
  };

  RMLPositionOutputParameters *get_output() const { return OP; };

  void updateCurrIn(const RMLPositionOutputParameters *OP)
  {
    *IP->CurrentPositionVector = *OP->NewPositionVector;
    *IP->CurrentVelocityVector = *OP->NewVelocityVector;
    *IP->CurrentAccelerationVector = *OP->NewAccelerationVector;
  };

  void setCurrIn(const VectorXd CurrPos, const VectorXd CurrVel, const VectorXd CurrAcc)
  {
    for (int i = 0; i < n_dofs_; i++)
    {
      IP->CurrentPositionVector->VecData[i] = CurrPos[i];
      IP->CurrentVelocityVector->VecData[i] = CurrVel[i];
      IP->CurrentAccelerationVector->VecData[i] = CurrAcc[i];
    }
  };

  void printActual() const
  {
    std::cout << "Printing Current Joint Values" << std::endl;
    for (int i = 0; i < n_dofs_; i++)
    {
      std::cout << "Joint " << i << std::endl;

      std::cout << " \n\t Cur. Pos: " << IP->CurrentPositionVector->VecData[i]
                << " \t Cur. Vel: " << IP->CurrentVelocityVector->VecData[i]
                << " \t Cur. Acc: " << IP->CurrentAccelerationVector->VecData[i] << "\n";
    }
  };

  void setTarget(const VectorXd TargetPos, VectorXd TargetVel)
  {

    for (int i = 0; i < n_dofs_; i++)
    {
      IP->TargetPositionVector->VecData[i] = TargetPos[i];
      IP->TargetVelocityVector->VecData[i] = TargetVel[i];
    }
    if (!(IP->CheckForValidity()))
      printf("Input target values are INVALID!\n");
    int resultValue = RML->RMLPosition(*IP, OP, Flags);
    state_ = resultValue;
    // ROS_WARN_STREAM("state_: " << state_ << " -> "
    //                          << ReflexxesAPI::RML_FINAL_STATE_REACHED);
  };

  void printTarget() const
  {
    std::cout << "Printing Target Joint Values" << std::endl;

    for (int i = 0; i < n_dofs_; i++)
    {
      std::cout << "Joint " << i << std::endl;

      std::cout << " \n\tDes. Pos: " << IP->TargetPositionVector->VecData[i]
                << " \tDes. Vel: " << IP->TargetVelocityVector->VecData[i] << "\n";
    }
  };

  void setLimit(const VectorXd MaxVel, VectorXd MaxAcc)
  {
    for (int i = 0; i < n_dofs_; i++)
    {
      IP->MaxVelocityVector->VecData[i] = MaxVel[i];
      IP->MaxAccelerationVector->VecData[i] = MaxAcc[i];
    }
    if (!(IP->CheckForValidity()))
      printf("Input Limits values are INVALID!\n");
  };

  void setLimit(const VectorXd MaxVel, VectorXd MaxAcc, VectorXd MaxJerk)
  {
    for (int i = 0; i < n_dofs_; i++)
    {
      IP->MaxVelocityVector->VecData[i] = MaxVel[i];
      IP->MaxAccelerationVector->VecData[i] = MaxAcc[i];
      IP->MaxJerkVector->VecData[i] = MaxJerk[i];
    }
    if (!(IP->CheckForValidity()))
      printf("Input Limits values are INVALID!\n");
  };

  void printLimits() const
  {
    std::cout << "Printing  Joint Limits" << std::endl;

    for (int i = 0; i < n_dofs_; i++)
    {
      std::cout << "Joint " << i << std::endl;

      std::cout << " \n\tMax. Vel: " << IP->MaxVelocityVector->VecData[i]
                << " \tMax. Acc: " << IP->MaxAccelerationVector->VecData[i]
                << " \tMax. Jerk: " << IP->MaxJerkVector->VecData[i] << "\n";
    }
  };

  void setStateReset() { state_ = 0; }

  int getState() { return state_; }

  private:
  int    n_dofs_;
  double cycle_time_s_;
  int    state_; //[0] Ready [1] Interpolation Ended

  // Reflexxes
  ReflexxesAPI *               RML;
  RMLPositionInputParameters * IP;
  RMLPositionOutputParameters *OP;
  RMLPositionFlags             Flags;
};

} // namespace rt_motion_planner

#endif
