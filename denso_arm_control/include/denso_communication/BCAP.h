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

#ifndef BCAP_CPP_WRAPPER_HEADER_GUARD
#define BCAP_CPP_WRAPPER_HEADER_GUARD

#define _USE_LINUX_API 1

#define E_BUF_FULL 0x83201483
#define S_BUF_FULL 0x0F200501

#include "bCAPClient/bcap_client.h"

#include <map>
#include <string>
#include <sstream>
#include <vector>

#include <sys/socket.h>

// I tried using constants but ow lord how C++ is annoying some times
#define EMPTY_CHARP std::string("").c_str()

namespace BCAP
{

class Driver
{
  // private:
  // I do not believe in private functions or variables.

  public:
  // Member variables
  // b-Cap communication
  std::string server_ip_address_;  // IP Address
  int         server_port_number_; // Port Number
  int         socket_;             // Socket (iSockFD in b-Cap.h)
  uint32_t    controller_handle_;  // Controller Handle (lhController in b-Cap.h)
  uint32_t    robot_handle_;       // Robot Handle (lhRobot in b-Cap.h)
  uint32_t    task_handle_;        // Task Handle

  long socket_timeout_;

  // Pointer for default results store locally (if for instance the user is not
  // interested them)
  long unwanted_result_;

  VARIANT unwanted_variant_result_;
  VARIANT empty_variant_;

  //        std::vector<double>& unwanted_t_pose_vector;

  // Variable handle
  std::map<std::string, u_int> controller_variable_handles_;

  // Common results stored for optimization
  //        double get_joint_positions_array_[8];
  //        double get_end_effector_pose_array_euler_[7];
  //        double get_end_effector_pose_array_homogenous_transformation_[10];

  //        double get_joint_positions_array_[8];
  //        double get_p_pose_array_[7];
  //        double get_t_pose_array_[10];

  VARIANT set_joint_positions_variant_;
  VARIANT set_end_effector_pose_variant_;

  HRESULT last_error_;

  Driver(std::string server_ip_address = std::string(""),
         const int server_port_number = 0, long socket_timeout = 2000000)
  {
    server_ip_address_ = server_ip_address;
    server_port_number_ = server_port_number;
    socket_timeout_ = socket_timeout;

    // Set joint positions structure used in bcap communication
    VariantInit(&set_joint_positions_variant_);

    VariantInit(&unwanted_variant_result_);
    VariantInit(&empty_variant_);
  }

  ~Driver()
  {
    VariantClear(&set_joint_positions_variant_);
    VariantClear(&unwanted_variant_result_);
    VariantClear(&empty_variant_);
  }

  HRESULT SetSocketTimeout(long micro_seconds)
  {
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;
    ROS_WARN("Setting send timeout...");
    if (setsockopt(socket_, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv)) < 0)
    {
      ROS_WARN("Failed to set send timeout");
      return E_FAIL;
    }
    ROS_WARN("Setting receive timeout...");
    if (setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0)
    {
      ROS_WARN("Failed to set recv timeout");
      return E_FAIL;
    }
    return S_OK;
  }

  // Error check function
  bool _ErrorCheck(const HRESULT &result)
  {
    last_error_ = result;
    std::string info;
    if (result < 0)
    {
      if (last_error_ == E_UNEXPECTED)
        info = std::string("E_UNEXPECTED");
      if (last_error_ == E_TIMEOUT)
        info = std::string("E_TIMEOUT");
      if (last_error_ == E_FAIL)
        info = std::string("E_FAIL");
      if (last_error_ == E_NOTIMPL)
        info = std::string("E_NOTIMPL");
      if (last_error_ == E_ALREADY_REGISTER)
        info = std::string("E_ALREADY_REGISTER");
      if (last_error_ == E_HANDLE)
        info = std::string("E_HANDLE");
      return false;
    }
    else if (last_error_ == E_BUF_FULL)
      info = std::string("E_BUF_FULL");
    else if (last_error_ == S_BUF_FULL)
      info = std::string("S_BUF_FULL");
    else
      return true;
  }

  // Methods

  bool Open()
  {
    std::ostringstream s;
    s << server_port_number_;
    std::string port_number_string(s.str());
    // std::string command_string(std::string("tcp:") + server_ip_address_ +
    // std::string(":") +
    //                            port_number_string);
    std::string command_string(std::string("tcp:") + server_ip_address_ +
                               std::string(":") + port_number_string);
    const char *command = command_string.c_str();
    return _ErrorCheck(bCap_Open_Client(command, 100000, 3, &socket_));
  }

  bool Close() { return _ErrorCheck(bCap_Close_Client(&socket_)); }

  bool ServiceStart() { return _ErrorCheck(bCap_ServiceStart(socket_, NULL)); }

  bool ServiceStop() { return _ErrorCheck(bCap_ServiceStop(socket_)); }

  bool ControllerConnect()
  {
    BSTR controller_name = SysAllocString(L"");
    BSTR controller_provider = SysAllocString(L"caoProv.DENSO.VRC");
    BSTR controller_machine = SysAllocString(L"localhost");
    BSTR controller_options = SysAllocString(L"");

    bool return_value = _ErrorCheck(bCap_ControllerConnect(
        socket_, controller_name, controller_provider, controller_machine,
        controller_options, &controller_handle_));

    SysFreeString(controller_name);
    SysFreeString(controller_provider);
    SysFreeString(controller_machine);
    SysFreeString(controller_options);

    return return_value;
  }

  bool ControllerDisconnect()
  {
    return _ErrorCheck(bCap_ControllerDisconnect(socket_, &controller_handle_));
  }

  bool GetRobot()
  {
    BSTR robot_name = SysAllocString(L"");
    BSTR options = SysAllocString(L"");

    bool return_value = _ErrorCheck(bCap_ControllerGetRobot(
        socket_, controller_handle_, robot_name, options, &robot_handle_));

    SysFreeString(robot_name);
    SysFreeString(options);

    return return_value;
  }

  bool ReleaseRobot() { _ErrorCheck(bCap_RobotRelease(socket_, &robot_handle_)); }

  bool GetControllerVariableHandle(const std::wstring &variable_name, uint32_t &handle)
  {
    BSTR variable_name_bstr = SysAllocString(variable_name.c_str());
    BSTR options = SysAllocString(L"");

    bool return_value = _ErrorCheck(bCap_ControllerGetVariable(
        socket_, controller_handle_, variable_name_bstr, options, &handle));

    SysFreeString(variable_name_bstr);
    SysFreeString(options);

    return return_value;
  }

  bool GetControllerVariableValueInt(const uint32_t &variable_handle, int &value_int)
  {
    VARIANT value;
    VariantInit(&value);

    bool return_value =
        _ErrorCheck(bCap_VariableGetValue(socket_, variable_handle, &value));

    value_int = value.lVal;

    VariantClear(&value);

    return return_value;
  }

  inline bool _RobotExecute(const std::wstring &command, const VARIANT &option,
                            VARIANT &result)
  {
    BSTR command_bstr = SysAllocString(command.c_str());

    bool return_value = _ErrorCheck(
        bCap_RobotExecute(socket_, robot_handle_, command_bstr, option, &result));

    SysFreeString(command_bstr);
    return return_value;
  }

  HRESULT _RobotExecute2(const std::wstring &command, const VARIANT &option,
                         VARIANT &result)
  {
    BSTR command_bstr = SysAllocString(command.c_str());

    HRESULT return_value =
        bCap_RobotExecute(socket_, robot_handle_, command_bstr, option, &result);

    SysFreeString(command_bstr);
    return return_value;
  }

  HRESULT InitilizeControllerVariableHandles()
  {
    uint handle;

    GetControllerVariableHandle(std::wstring(L"@ERROR_CODE"), handle);
    std::map<std::string, u_int>::value_type ERROR_CODE("ERROR_CODE", handle);
    controller_variable_handles_.insert(ERROR_CODE);

    GetControllerVariableHandle(std::wstring(L"@ERROR_DESCRIPTION"), handle);
    std::map<std::string, u_int>::value_type ERROR_DESCRIPTION("ERROR_DESCRIPTION",
                                                               handle);
    controller_variable_handles_.insert(ERROR_DESCRIPTION);

    GetControllerVariableHandle(std::wstring(L"@MODE"), handle);
    std::map<std::string, u_int>::value_type MODE("MODE", handle);
    controller_variable_handles_.insert(MODE);

    GetControllerVariableHandle(std::wstring(L"@EMERGENCY_STOP"), handle);
    std::map<std::string, u_int>::value_type EMERGENCY_STOP("EMERGENCY_STOP", handle);
    controller_variable_handles_.insert(EMERGENCY_STOP);
  }

  /********************************************************************************
   ********** COMMON FUNCTIONS THAT USE ROBOT GETCONTROLLERVARIABLEVALUE
   ***********
   ********************************************************************************/

  bool GetMode(int &mode)
  {
    return _ErrorCheck(
        GetControllerVariableValueInt(controller_variable_handles_["@MODE"], mode));
  }

  bool GetEmergencyStop(int &status)
  {
    return _ErrorCheck(GetControllerVariableValueInt(
        controller_variable_handles_["@EMERGENCY_STOP"], status));
  }

  bool GetControllerStatus(int &status)
  {
    return _ErrorCheck(GetControllerVariableValueInt(
        controller_variable_handles_["@ERROR_CODE"], status));
  }

  /*************************************************************
   ********** COMMON FUNCTIONS THAT USE ROBOT EXECUTE **********
   *************************************************************/

  bool ClearError(VARIANT &result)
  {
    return _RobotExecute(std::wstring(L"ClearError"), empty_variant_, result);
  }
  // Overload with no arguments
  inline bool ClearError() { return ClearError(unwanted_variant_result_); }

  bool TakeArm(VARIANT &result)
  {
    return _RobotExecute(std::wstring(L"TakeArm"), empty_variant_, result);
  }
  // Overload with no arguments
  inline bool TakeArm() { return TakeArm(unwanted_variant_result_); }

  bool GiveArm(VARIANT &result)
  {
    return _RobotExecute(std::wstring(L"GiveArm"), empty_variant_, result);
  }
  // Overload with no arguments
  inline bool GiveArm() { return GiveArm(unwanted_variant_result_); }

  bool SetMotorState(bool state, VARIANT &result)
  {
    VARIANT param;
    VariantInit(&param);
    u_int32_t *param_data;
    param.vt = VT_I4 | VT_ARRAY;
    param.parray = SafeArrayCreateVector(VT_I4, 0, 2);

    bool result_value;

    if (state)
    {
      SafeArrayAccessData(param.parray, (void **)&param_data);
      param_data[0] = 1;
      param_data[1] = 0;
      SafeArrayUnaccessData(param.parray);
      result_value = _RobotExecute(std::wstring(L"Motor"), param, result);
    }
    else
    {
      SafeArrayAccessData(param.parray, (void **)&param_data);
      param_data[0] = 0;
      param_data[1] = 1;
      SafeArrayUnaccessData(param.parray);
      result_value = _RobotExecute(std::wstring(L"Motor"), param, result);
    }

    VariantClear(&param);
    return result_value;
  }
  // Overload with one argument
  inline bool SetMotorState(bool state)
  {
    return SetMotorState(state, unwanted_variant_result_);
  }

  bool GetJointPositions(std::vector<double> &get_joint_positions_vector)
  {
    VARIANT result;
    VariantInit(&result);

    bool return_value = _RobotExecute(std::wstring(L"CurJnt"), empty_variant_, result);

    double *data_pointer;

    SafeArrayAccessData(result.parray, (void **)&data_pointer);
    get_joint_positions_vector.assign(data_pointer, data_pointer + 8);
    SafeArrayUnaccessData(result.parray);

    VariantClear(&result);

    return return_value;
  }

  bool GetPPose(std::vector<double> &get_p_pose_vector)
  {
    VARIANT result;
    VariantInit(&result);

    bool return_value = _RobotExecute(std::wstring(L"CurPos"), empty_variant_, result);

    double *data_pointer;

    SafeArrayAccessData(result.parray, (void **)&data_pointer);
    get_p_pose_vector.assign(data_pointer, data_pointer + 7);
    SafeArrayUnaccessData(result.parray);

    VariantClear(&result);

    return return_value;
  }

  bool GetTPose(std::vector<double> &get_t_pose_vector)
  {
    VARIANT result;
    VariantInit(&result);

    bool return_value = _RobotExecute(std::wstring(L"CurTrn"), empty_variant_, result);

    double *data_pointer;

    SafeArrayAccessData(result.parray, (void **)&data_pointer);
    get_t_pose_vector.assign(data_pointer, data_pointer + 10);
    SafeArrayUnaccessData(result.parray);

    VariantClear(&result);

    return return_value;
  }

  bool GetFValue(std::vector<double> &get_f_value)
  {

    int      fd;
    uint32_t hCtrl, hVar;
    HRESULT  hr;

    VARIANT vntResult;
    VARIANT vntResult2;

    BSTR bstrVarName, bstrVarOpt;

    VariantInit(&vntResult);
    VariantInit(&vntResult2);

    bstrVarName = SysAllocString(L"IO128");
    bstrVarOpt = SysAllocString(L"");
    hr = bCap_ControllerGetVariable(socket_, controller_handle_, bstrVarName, bstrVarOpt,
                                    &hVar);

    SysFreeString(bstrVarName);
    SysFreeString(bstrVarOpt);

    bCap_VariableGetValue(socket_, hVar, &vntResult);

    ROS_INFO("IO128: %i", vntResult.boolVal);

    VariantClear(&vntResult);
    bCap_VariableRelease(socket_, &hVar);

    bstrVarName = SysAllocString(L"V1");
    bstrVarOpt = SysAllocString(L"");
    hr = bCap_ControllerGetVariable(socket_, controller_handle_, bstrVarName, bstrVarOpt,
                                    &hVar);

    SysFreeString(bstrVarName);
    SysFreeString(bstrVarOpt);

    bCap_VariableGetValue(socket_, hVar, &vntResult2);

    float *pfData, fval[3];
    //                printf("P1");
    SafeArrayAccessData(vntResult2.parray, (void **)&pfData);
    memcpy(fval, pfData, 3 * sizeof(float));
    SafeArrayUnaccessData(vntResult2.parray);

    VariantClear(&vntResult2);
    bCap_VariableRelease(socket_, &hVar);

    ROS_INFO("V1  %f %f %f", fval[0], fval[1], fval[2]);

    return 0;
  }

  bool StartTask()
  {
    HRESULT hr;
    int32_t lMode;

    /* Get task handle */
    BSTR bstrTskName, bstrTskOpt;
    bstrTskName = SysAllocString(L"LogClear");
    bstrTskOpt = SysAllocString(L"");
    hr = bCap_ControllerGetTask(socket_, controller_handle_, bstrTskName, bstrTskOpt,
                                &task_handle_);

    SysFreeString(bstrTskName);
    SysFreeString(bstrTskOpt);
    if FAILED (hr)
      return (hr);

    /* Start task */
    lMode = 1L;
    bCap_TaskStart(socket_, task_handle_, lMode, bstrTskOpt);

    /* Release task handle */
    bCap_TaskRelease(socket_, &task_handle_);
  }

  bool StopTask()
  {
    HRESULT hr;
    int32_t lMode;

    /* Get task handle */
    BSTR bstrTskName, bstrTskOpt;
    bstrTskName = SysAllocString(L"LogStop");
    bstrTskOpt = SysAllocString(L"");
    hr = bCap_ControllerGetTask(socket_, controller_handle_, bstrTskName, bstrTskOpt,
                                &task_handle_);

    SysFreeString(bstrTskName);
    SysFreeString(bstrTskOpt);
    if FAILED (hr)
      return (hr);

    /* Start task */
    lMode = 1L;
    bCap_TaskStart(socket_, task_handle_, lMode, bstrTskOpt);

    /* Release task handle */
    bCap_TaskRelease(socket_, &task_handle_);
  }

  /*************************************************************
  ********** COMMON FUNCTIONS THAT USE ROBOT EXECUTE 2 ********
  *************************************************************/

  bool SetSpeed(const float &speed, const float &acceleration,
                const float &deacceleration)
  {
    float * param_data;
    VARIANT param;
    param.vt = VT_R4 | VT_ARRAY;
    param.parray = SafeArrayCreateVector(VT_R4, 0, 3);
    SafeArrayAccessData(param.parray, (void **)&param_data);
    param_data[0] = speed;
    param_data[1] = acceleration;
    param_data[2] = deacceleration;
    SafeArrayUnaccessData(param.parray);

    bool return_value = _ErrorCheck(
        _RobotExecute(std::wstring(L"ExtSpeed"), param, unwanted_variant_result_));

    VariantClear(&param);

    return return_value;
  }
  // Overload with one argument
  //    inline HRESULT SetSpeed(float value) {return
  //    SetSpeed(value,&unwanted_variant_result_);}

  HRESULT
  SetJointPositions(const std::vector<double> &set_joint_positions_vector,
                    std::vector<double> &      get_joint_positions_vector)
  {
    VARIANT variant_result;
    VARIANT request;
    VariantInit(&request);
    VariantInit(&variant_result);

    request.vt = VT_R8 | VT_ARRAY;
    request.parray = SafeArrayCreateVector(VT_R8, 0, 8);
    double *param_data;

    SafeArrayAccessData(request.parray, (void **)&param_data);
    param_data[0] = set_joint_positions_vector[0];
    param_data[1] = set_joint_positions_vector[1];
    param_data[2] = set_joint_positions_vector[2];
    param_data[3] = set_joint_positions_vector[3];
    param_data[4] = set_joint_positions_vector[4];
    param_data[5] = set_joint_positions_vector[5];
    param_data[6] = 0;
    param_data[7] = 0;
    SafeArrayUnaccessData(request.parray);

    HRESULT return_value =
        _RobotExecute2(std::wstring(L"slvMove"), request, variant_result);

    double *data_pointer;

    SafeArrayAccessData(variant_result.parray, (void **)&data_pointer);
    get_joint_positions_vector.assign(data_pointer, data_pointer + 8);
    // printf("Get J1: %f", get_joint_positions_vector[1]);
    // printf("Get Poi J1: %f", data_pointer);

    SafeArrayUnaccessData(variant_result.parray);

    VariantClear(&request);
    VariantClear(&variant_result);

    return return_value;
  }

  bool SetPPose(const std::vector<double> &set_p_pose_vector, VARIANT &variant_result)
  {
    VARIANT request;
    VariantInit(&request);
    request.vt = VT_R8 | VT_ARRAY;
    request.parray = SafeArrayCreateVector(VT_R8, 0, 7);
    double *param_data;

    SafeArrayAccessData(request.parray, (void **)&param_data);
    param_data[0] = set_p_pose_vector[0];
    param_data[1] = set_p_pose_vector[1];
    param_data[2] = set_p_pose_vector[2];
    param_data[3] = set_p_pose_vector[3];
    param_data[4] = set_p_pose_vector[4];
    param_data[5] = set_p_pose_vector[5];
    param_data[6] = set_p_pose_vector[6];

    SafeArrayUnaccessData(request.parray);

    bool return_value = _RobotExecute(std::wstring(L"slvMove"), request, variant_result);

    VariantClear(&request);

    return return_value;
  }
  // Overload with one argument
  inline bool SetPPose(std::vector<double> &set_p_pose_vector)
  {
    return SetPPose(set_p_pose_vector, unwanted_variant_result_);
  }

  HRESULT SetTPose(const std::vector<double> &set_t_pose_vector,
                   std::vector<double> &      get_t_pose_vector)
  {
    VARIANT variant_result;
    VARIANT request;
    VariantInit(&request);
    VariantInit(&variant_result);

    request.vt = VT_R8 | VT_ARRAY;
    request.parray = SafeArrayCreateVector(VT_R8, 0, 10);
    double *param_data;

    SafeArrayAccessData(request.parray, (void **)&param_data);
    param_data[0] = set_t_pose_vector[0];
    param_data[1] = set_t_pose_vector[1];
    param_data[2] = set_t_pose_vector[2];
    param_data[3] = set_t_pose_vector[3];
    param_data[4] = set_t_pose_vector[4];
    param_data[5] = set_t_pose_vector[5];
    param_data[6] = set_t_pose_vector[6];
    param_data[7] = set_t_pose_vector[7];
    param_data[8] = set_t_pose_vector[8];
    param_data[9] = set_t_pose_vector[9];

    SafeArrayUnaccessData(request.parray);

    HRESULT return_value =
        _RobotExecute2(std::wstring(L"slvMove"), request, variant_result);

    double *data_pointer;

    SafeArrayAccessData(variant_result.parray, (void **)&data_pointer);
    get_t_pose_vector.assign(data_pointer, data_pointer + 10);
    SafeArrayUnaccessData(variant_result.parray);

    VariantClear(&request);
    VariantClear(&variant_result);

    return return_value;
  }

  bool SetSlaveMode(const int32_t &value, VARIANT &result)
  {
    // This function is rarely used so it doesn't need to be optimized
    VARIANT argument;
    VariantInit(&argument);
    argument.vt = VT_I4;
    argument.lVal = value;

    bool return_value = _RobotExecute(std::wstring(L"slvChangeMode"), argument, result);

    VariantClear(&argument);

    return return_value;
  }
  // Overload with one argument
  inline bool SetSlaveMode(long value)
  {
    return SetSlaveMode(value, unwanted_variant_result_);
  }

  bool SetRecvFormat(const int32_t &value, VARIANT &result)
  {
    // This function is rarely used so it doesn't need to be optimized
    VARIANT argument;
    VariantInit(&argument);
    argument.vt = VT_I4;
    argument.lVal = value;

    bool return_value = _RobotExecute(std::wstring(L"slvRecvFormat"), argument, result);

    VariantClear(&argument);

    return return_value;
  }
  // Overload with one argument
  inline bool SetRecvFormat(long value)
  {
    return SetRecvFormat(value, unwanted_variant_result_);
  }
};
} // namespace BCAP

#endif
