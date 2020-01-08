cmake_minimum_required(VERSION 2.8.3)
project(denso_ethercat)

find_package(catkin REQUIRED COMPONENTS controller_manager  diagnostic_updater transmission_interface hardware_interface sensor_msgs joint_limits_interface trajectory_msgs ethercat_manager soem kdl_parser eigen_conversions)
find_package(orocos_kdl)

set(CMAKE_BUILD_TYPE Debug)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS diagnostic_updater trajectory_msgs sensor_msgs)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ## The following work around allows SOEM headers to include other SOEM headers.
  ## SOEM headers assume all headers are installed in a flat directory structure
  ## See https://github.com/smits/soem/issues/4 for more information.
  ${soem_INCLUDE_DIRS}/soem
)

## Declare a C++ library
add_library(denso_client src/denso_client.cpp)

## Declare a C++ executable for ROS_CONTROL
add_executable(main src/main.cpp src/denso_hardware_interface.cpp)

## Declare a C++ executable
add_executable(slaveinf src/slaveinfo.cpp)
add_executable(reset src/reset.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(main ${catkin_LIBRARIES} denso_client)
target_link_libraries(slaveinf ${catkin_LIBRARIES})
target_link_libraries(reset ${catkin_LIBRARIES} denso_client)

function(set_sudo_binary tgt)
  get_target_property(loc ${tgt} "LOCATION")
  string(ASCII 27 ESCAPE)
  add_custom_target(sudo_${tgt}
    COMMAND sudo -n sh -c 'setcap cap_net_raw,cap_ipc_lock=+ep ${loc}' || (>&2 echo "${ESCAPE}[31m#\ type\ following\ command\ before\ execute\ reset...\ sudo\ setcap cap_net_raw,cap_ipc_lock=+ep\ ${loc}${ESCAPE}[0m")
    DEPENDS bin/${tgt})
  add_custom_target(sudo_${tgt}_all ALL DEPENDS sudo_${tgt})
endfunction()

set_sudo_binary(main)
set_sudo_binary(slaveinfo)
set_sudo_binary(reset)

#############
## Install ##
#############
