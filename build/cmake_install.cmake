# Install script for directory: /home/leonardonavacastellanos/Documents/Tec/SextoSemestre/RoboticaInteligente/actividadesManchester/Travel2PointPuzzlebot/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/leonardonavacastellanos/Documents/Tec/SextoSemestre/RoboticaInteligente/actividadesManchester/Travel2PointPuzzlebot/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
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

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/leonardonavacastellanos/Documents/Tec/SextoSemestre/RoboticaInteligente/actividadesManchester/Travel2PointPuzzlebot/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/leonardonavacastellanos/Documents/Tec/SextoSemestre/RoboticaInteligente/actividadesManchester/Travel2PointPuzzlebot/install" TYPE PROGRAM FILES "/home/leonardonavacastellanos/Documents/Tec/SextoSemestre/RoboticaInteligente/actividadesManchester/Travel2PointPuzzlebot/build/catkin_generated/installspace/_setup_util.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/leonardonavacastellanos/Documents/Tec/SextoSemestre/RoboticaInteligente/actividadesManchester/Travel2PointPuzzlebot/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/leonardonavacastellanos/Documents/Tec/SextoSemestre/RoboticaInteligente/actividadesManchester/Travel2PointPuzzlebot/install" TYPE PROGRAM FILES "/home/leonardonavacastellanos/Documents/Tec/SextoSemestre/RoboticaInteligente/actividadesManchester/Travel2PointPuzzlebot/build/catkin_generated/installspace/env.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/leonardonavacastellanos/Documents/Tec/SextoSemestre/RoboticaInteligente/actividadesManchester/Travel2PointPuzzlebot/install/setup.bash;/home/leonardonavacastellanos/Documents/Tec/SextoSemestre/RoboticaInteligente/actividadesManchester/Travel2PointPuzzlebot/install/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/leonardonavacastellanos/Documents/Tec/SextoSemestre/RoboticaInteligente/actividadesManchester/Travel2PointPuzzlebot/install" TYPE FILE FILES
    "/home/leonardonavacastellanos/Documents/Tec/SextoSemestre/RoboticaInteligente/actividadesManchester/Travel2PointPuzzlebot/build/catkin_generated/installspace/setup.bash"
    "/home/leonardonavacastellanos/Documents/Tec/SextoSemestre/RoboticaInteligente/actividadesManchester/Travel2PointPuzzlebot/build/catkin_generated/installspace/local_setup.bash"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/leonardonavacastellanos/Documents/Tec/SextoSemestre/RoboticaInteligente/actividadesManchester/Travel2PointPuzzlebot/install/setup.sh;/home/leonardonavacastellanos/Documents/Tec/SextoSemestre/RoboticaInteligente/actividadesManchester/Travel2PointPuzzlebot/install/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/leonardonavacastellanos/Documents/Tec/SextoSemestre/RoboticaInteligente/actividadesManchester/Travel2PointPuzzlebot/install" TYPE FILE FILES
    "/home/leonardonavacastellanos/Documents/Tec/SextoSemestre/RoboticaInteligente/actividadesManchester/Travel2PointPuzzlebot/build/catkin_generated/installspace/setup.sh"
    "/home/leonardonavacastellanos/Documents/Tec/SextoSemestre/RoboticaInteligente/actividadesManchester/Travel2PointPuzzlebot/build/catkin_generated/installspace/local_setup.sh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/leonardonavacastellanos/Documents/Tec/SextoSemestre/RoboticaInteligente/actividadesManchester/Travel2PointPuzzlebot/install/setup.zsh;/home/leonardonavacastellanos/Documents/Tec/SextoSemestre/RoboticaInteligente/actividadesManchester/Travel2PointPuzzlebot/install/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/leonardonavacastellanos/Documents/Tec/SextoSemestre/RoboticaInteligente/actividadesManchester/Travel2PointPuzzlebot/install" TYPE FILE FILES
    "/home/leonardonavacastellanos/Documents/Tec/SextoSemestre/RoboticaInteligente/actividadesManchester/Travel2PointPuzzlebot/build/catkin_generated/installspace/setup.zsh"
    "/home/leonardonavacastellanos/Documents/Tec/SextoSemestre/RoboticaInteligente/actividadesManchester/Travel2PointPuzzlebot/build/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/leonardonavacastellanos/Documents/Tec/SextoSemestre/RoboticaInteligente/actividadesManchester/Travel2PointPuzzlebot/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/leonardonavacastellanos/Documents/Tec/SextoSemestre/RoboticaInteligente/actividadesManchester/Travel2PointPuzzlebot/install" TYPE FILE FILES "/home/leonardonavacastellanos/Documents/Tec/SextoSemestre/RoboticaInteligente/actividadesManchester/Travel2PointPuzzlebot/build/catkin_generated/installspace/.rosinstall")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/leonardonavacastellanos/Documents/Tec/SextoSemestre/RoboticaInteligente/actividadesManchester/Travel2PointPuzzlebot/build/gtest/cmake_install.cmake")
  include("/home/leonardonavacastellanos/Documents/Tec/SextoSemestre/RoboticaInteligente/actividadesManchester/Travel2PointPuzzlebot/build/chrisAct1_1/cmake_install.cmake")
  include("/home/leonardonavacastellanos/Documents/Tec/SextoSemestre/RoboticaInteligente/actividadesManchester/Travel2PointPuzzlebot/build/puzzlebot_world/cmake_install.cmake")
  include("/home/leonardonavacastellanos/Documents/Tec/SextoSemestre/RoboticaInteligente/actividadesManchester/Travel2PointPuzzlebot/build/odometry/cmake_install.cmake")
  include("/home/leonardonavacastellanos/Documents/Tec/SextoSemestre/RoboticaInteligente/actividadesManchester/Travel2PointPuzzlebot/build/challenge_autonomous_nav/cmake_install.cmake")
  include("/home/leonardonavacastellanos/Documents/Tec/SextoSemestre/RoboticaInteligente/actividadesManchester/Travel2PointPuzzlebot/build/puzzlebot_gazebo/cmake_install.cmake")
  include("/home/leonardonavacastellanos/Documents/Tec/SextoSemestre/RoboticaInteligente/actividadesManchester/Travel2PointPuzzlebot/build/puzzlebot_control/cmake_install.cmake")
  include("/home/leonardonavacastellanos/Documents/Tec/SextoSemestre/RoboticaInteligente/actividadesManchester/Travel2PointPuzzlebot/build/puzzlebot_nav2d/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/leonardonavacastellanos/Documents/Tec/SextoSemestre/RoboticaInteligente/actividadesManchester/Travel2PointPuzzlebot/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
