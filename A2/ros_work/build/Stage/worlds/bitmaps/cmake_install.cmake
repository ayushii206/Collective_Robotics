# Install script for directory: /home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/bitmaps

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/stage/worlds/bitmaps" TYPE FILE FILES
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/bitmaps/889_05.png"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/bitmaps/SFU_1200x615.png"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/bitmaps/SRI-AIC-kwing.png"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/bitmaps/autolab.png"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/bitmaps/cave.png"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/bitmaps/cave_compact.png"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/bitmaps/cave_filled.png"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/bitmaps/frieburg.png"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/bitmaps/ghost.png"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/bitmaps/hospital.png"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/bitmaps/hospital_section.png"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/bitmaps/human_outline.png"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/bitmaps/mbicp.png"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/bitmaps/rink.png"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/bitmaps/sal2.png"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/bitmaps/simple_rooms.png"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/bitmaps/space_invader.png"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/bitmaps/submarine.png"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/bitmaps/submarine_small.png"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/bitmaps/table.png"
    "/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/src/stage_ros2/worlds/bitmaps/uoa_robotics_lab.png"
    )
endif()

