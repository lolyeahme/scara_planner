cmake_minimum_required(VERSION 3.16)

project(rmvl_demos LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(RMVL REQUIRED)

rmvl_add_exe(
  scara_planner
  SOURCES src/scara_planner_core.cpp
          src/scara_planner_trt.cpp  
          src/scara_planner_bl.cpp  
          src/main.cpp
          third_party/scara/robo_base.cpp
          third_party/scara/TrajectoryPlanner.cpp
  DEPENDS opcua lpss
)

rmvl_add_exe(
  scara_scheduler
  SOURCES src/scara_scheduler.cpp
  DEPENDS opcua lpss
)

rmvl_add_exe(
  scara_remote
  SOURCES src/scara_remote.cpp
  DEPENDS opcua lpss 
)
# remote uses fmt::println; fmt is pulled in via lpss in this project

target_include_directories(scara_planner PRIVATE 
  ${CMAKE_SOURCE_DIR}/include 
  ${CMAKE_SOURCE_DIR}/src
  ${CMAKE_SOURCE_DIR}/third_party/scara
)

