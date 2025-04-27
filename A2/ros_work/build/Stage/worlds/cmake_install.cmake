# Install script for directory: /home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/install/Stage")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "RELEASE")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set path to fallback-tool for dependency-resolution.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/stage/worlds" TYPE FILE FILES
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/amcl-sonar.cfg"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/autolab.cfg"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/camera.cfg"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/everything.cfg"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/lsp_test.cfg"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/mbicp.cfg"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/nd.cfg"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/roomba.cfg"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/simple.cfg"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/test.cfg"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/uoa_robotics_lab.cfg"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/vfh.cfg"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/wavefront-remote.cfg"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/wavefront.cfg"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/wifi.cfg"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/SFU.world"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/autolab.world"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/camera.world"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/circuit.world"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/everything.world"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/fasr.world"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/fasr2.world"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/fasr_plan.world"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/large.world"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/lsp_test.world"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/mbicp.world"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/pioneer_flocking.world"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/pioneer_follow.world"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/pioneer_walle.world"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/roomba.world"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/sensor_noise_demo.world"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/sensor_noise_module_demo.world"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/simple.world"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/uoa_robotics_lab.world"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/wifi.world"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/beacons.inc"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/chatterbox.inc"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/hokuyo.inc"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/irobot.inc"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/map.inc"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/objects.inc"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/pantilt.inc"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/pioneer.inc"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/sick.inc"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/ubot.inc"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/uoa_robotics_lab_models.inc"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/walle.inc"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/cfggen.sh"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/test.sh"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/worldgen.sh"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/build/Stage/worlds/benchmark/cmake_install.cmake")
  include("/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/build/Stage/worlds/bitmaps/cmake_install.cmake")
  include("/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/build/Stage/worlds/wifi/cmake_install.cmake")

endif()

