cmake_minimum_required(VERSION 2.8.3)
project(denso_arm_control)

add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  trac_ik_lib
  sensor_msgs
  std_msgs
  geometry_msgs
  realtime_scheduler_interface
  kdl_parser
  eigen_conversions
  dynamic_reconfigure
  controller_manager
  joint_limits_interface
  actionlib_msgs
  actionlib
  message_generation
)

# Boost
find_package(Boost REQUIRED COMPONENTS date_time)
# Eigen
find_package(Eigen3 REQUIRED NO_MODULE)
# Reflexxes
set(EXTRALIB_BIN1 ${PROJECT_SOURCE_DIR}/lib/libReflexxesTypeII.a)

# RELEASE TYPE
set(CMAKE_BUILD_TYPE Debug)

## System dependencies are found with CMake's conventions
find_package(Qt5Widgets REQUIRED)

set (QT_PROJECT_SOURCE_DIR src/qt)

set (QtInterfaceLib_src ${QT_PROJECT_SOURCE_DIR}/mainwindow.cpp)
set (QtInterfaceLib_hdr ${QT_PROJECT_SOURCE_DIR}/mainwindow.h)
set (QtInterfaceLib_ui  ${QT_PROJECT_SOURCE_DIR}/mainwindow.ui)
set (QtInterfaceLib_qrc ${QT_PROJECT_SOURCE_DIR}/resource_file.qrc)

qt5_wrap_cpp(QtInterfaceLib_hdr_moc ${QtInterfaceLib_hdr})
qt5_wrap_ui (QtInterfaceLib_ui_moc  ${QtInterfaceLib_ui})
qt5_add_resources(QtInterfaceLib_qrc_moc ${QtInterfaceLib_qrc})

include_directories (${QT_PROJECT_SOURCE_DIR})
include_directories (${PROJECT_BINARY_DIR})

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   robot_command.srv
 )

# Generate actions in the 'action' folder
add_action_files(
  FILES
  FollowJointCmd.action
  FollowToolCmd.action
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   sensor_msgs
   actionlib_msgs
   std_msgs  
   geometry_msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  config/DynVar.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime trac_ik_lib actionlib actionlib_msgs roscpp
  DEPENDS
    Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  include/Reflexxes
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(denso_arm_control_lib  src/denso_arm_control.cpp )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(denso_arm_control_lib ${${PROJECT_NAME}_generate_messages} c${${PROJECT_NAME}_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(denso_arm_planner_exec
    src/denso_arm_planner.cpp
    )

add_executable(denso_arm_example_exec
  src/denso_arm_example.cpp
  )


add_executable(denso_arm_control_exec
    src/main.cpp
    src/bcap/bCAPClient/bcap_client.c
    src/bcap/bcap_common.c
    src/bcap/dn_com.c
    src/bcap/dn_common.c
    src/bcap/dn_device.c
    src/bcap/dn_tcp.c
    src/bcap/dn_udp.c
    src/bcap/dn_thread.c
    src/bcap/dn_socket.c
    )

add_executable(denso_arm_gui_exec
  src/denso_arm_gui.cpp
  ${QT_PROJECT_SOURCE_DIR}/mainwindow.cpp
  ${QtInterfaceLib_src}
  ${QtInterfaceLib_hdr_moc}
  ${QtInterfaceLib_ui_moc}
  ${QtInterfaceLib_qrc_moc}
  )
## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(denso_vs050_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(denso_arm_control_exec
  denso_arm_control_lib
  ${catkin_LIBRARIES}
 )

 target_link_libraries(denso_arm_planner_exec
 ${catkin_LIBRARIES}
 ${EXTRALIB_BIN1}
)

target_link_libraries(denso_arm_example_exec
 ${catkin_LIBRARIES}
)

target_link_libraries(denso_arm_gui_exec
 ${catkin_LIBRARIES}
 ${CMAKE_THREAD_LIBS_INIT} # needed for the qrc file compilation
 Qt5::Widgets
)
