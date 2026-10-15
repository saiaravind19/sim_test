cmake_minimum_required(VERSION 3.16)
project(SimRvoFleet LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# OpenMP is optional (worlds are planned in parallel when it is found).
# Apple clang has none: install it with `brew install libomp`.
if(APPLE AND NOT OpenMP_ROOT)
  execute_process(COMMAND brew --prefix libomp OUTPUT_VARIABLE _libomp
                  OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
  if(_libomp)
    set(OpenMP_ROOT ${_libomp})
  endif()
endif()
find_package(OpenMP COMPONENTS CXX)
if(NOT OpenMP_CXX_FOUND)
  message(WARNING "OpenMP not found: planning runs on one thread")
endif()
find_package(nlohmann_json REQUIRED)

# zeromq: pkg-config first, then plain find (Homebrew hints for macOS).
find_path(ZMQ_INCLUDE_DIR zmq.h HINTS /opt/homebrew/include /usr/local/include)
find_library(ZMQ_LIBRARY zmq HINTS /opt/homebrew/lib /usr/local/lib)
if(NOT ZMQ_INCLUDE_DIR OR NOT ZMQ_LIBRARY)
  message(FATAL_ERROR "ZeroMQ not found (apt install libzmq3-dev / brew install zeromq)")
endif()

# RVO2 (ORCA, 2D), built from source. Its own CMakeLists is skipped.
include(FetchContent)
FetchContent_Declare(rvo2
  GIT_REPOSITORY https://github.com/snape/RVO2.git
  GIT_TAG v2.0.2
  SOURCE_SUBDIR does_not_exist)
FetchContent_MakeAvailable(rvo2)
add_library(rvo2 STATIC
  ${rvo2_SOURCE_DIR}/src/Agent.cpp
  ${rvo2_SOURCE_DIR}/src/KdTree.cpp
  ${rvo2_SOURCE_DIR}/src/Obstacle.cpp
  ${rvo2_SOURCE_DIR}/src/RVOSimulator.cpp)
target_include_directories(rvo2 PUBLIC ${rvo2_SOURCE_DIR}/src)
set_target_properties(rvo2 PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Planner core: no transport, no ROS. Used by every front end below.
add_subdirectory(core)

# Message code (nanopb) shared with the simulator.
add_library(sim2d_msgs STATIC
  gen/sim2d_msgs.pb.c
  third_party/nanopb/pb_common.c
  third_party/nanopb/pb_decode.c
  third_party/nanopb/pb_encode.c)
target_include_directories(sim2d_msgs PUBLIC gen third_party/nanopb)

add_executable(rvo_planner_zmq app/zmq_planner.cpp)
target_include_directories(rvo_planner_zmq PRIVATE ${ZMQ_INCLUDE_DIR})
target_link_libraries(rvo_planner_zmq PRIVATE fleet_core sim2d_msgs ${ZMQ_LIBRARY}
  nlohmann_json::nlohmann_json)
if(OpenMP_CXX_FOUND)
  target_link_libraries(rvo_planner_zmq PRIVATE OpenMP::OpenMP_CXX)
endif()
